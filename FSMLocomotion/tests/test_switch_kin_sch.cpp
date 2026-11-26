#include "RobotFSM.hpp"
#include "JointCommandHandoff.hpp"
#include "LcmDataExchange.hpp"
#include "LocomotionControllerMPC/LocomotionControllerMPC.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

using namespace mors::fsm;

namespace {

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

constexpr std::array schemes{KneeScheme::inside, KneeScheme::outside, KneeScheme::back, KneeScheme::front};
constexpr std::array<std::array<int, 4>, 4> knee_signs{{
    {1, -1, -1, 1}, {-1, 1, 1, -1}, {1, -1, 1, -1}, {-1, 1, -1, 1}}};

JointVector pose(unsigned mask) {
    JointVector result;
    for (int leg = 0; leg < 4; ++leg) {
        const double sign = mask & (1u << leg) ? 1.0 : -1.0;
        result.segment<3>(3 * leg) << (leg - 1.5) * 0.08,
            -sign * (0.5 + leg * 0.13), sign * (1.1 + leg * 0.21);
    }
    return result;
}

unsigned scheme_mask(std::size_t scheme) {
    unsigned mask = 0;
    for (int leg = 0; leg < 4; ++leg) if (knee_signs[scheme][leg] > 0) mask |= 1u << leg;
    return mask;
}

JointLimitList limits();

class Controller : public LocomotionControlBase {
public:
    Controller() { set_data_input_mode(robot_input); }
    void begin_input_handoff(std::uint8_t mode, double duration, double speed) override {
        require(duration > 0.0 && speed > 0.0, "Invalid handoff request");
        LocomotionControlBase::begin_input_handoff(mode, duration, speed);
        ++handoffs;
        if (mode == joints_input) { joint_ready = false; height.reset(); }
        else robot_ready = false;
    }
    bool joints_control_ready() const override { return outputs_enabled() && joint_ready; }
    bool robot_control_ready() const override { return outputs_enabled() && robot_ready; }
    void set_joints_cmd(const JointCommand& value) override {
        require(input_mode == joints_input && outputs_enabled(), "Unexpected joint command");
        joints = value;
        ++joint_commands;
    }
    void set_robot_cmd(const RobotData& value) override {
        LocomotionControlBase::set_robot_cmd(value);
        body = value;
        ++robot_commands;
    }
    void hold_measured_pose() override {
        require(input_mode == robot_input, "Hold outside robot_input");
        ++holds;
    }
    void set_body_height_override(std::optional<BodyHeightCommand> value) override {
        height = value;
        if (value) heights.emplace_back(robot_state()->received_at_s, *value);
    }
    void disable_outputs() override {
        height.reset();
        LocomotionControlBase::disable_outputs();
    }
    std::optional<SafetyFault> controller_fault() override { return fault; }
    std::uint8_t source() const { return input_mode.value_or(0); }

    JointCommand joints;
    RobotData body;
    bool joint_ready = false, robot_ready = true;
    int handoffs = 0, joint_commands = 0, robot_commands = 0, holds = 0;
    std::optional<SafetyFault> fault;
    std::optional<BodyHeightCommand> height;
    std::vector<std::pair<double, BodyHeightCommand>> heights;
};

RobotStateSnapshot observation(const JointVector& position) {
    RobotStateSnapshot state;
    state.joints.position = position;
    state.body.pos.z() = 0.20;
    state.received_at_s = monotonic_seconds();
    state.readiness.fill(true);
    state.legs.contacts.assign(4, true);
    return state;
}

void all_plans_and_trajectories() {
    // All 16 initial knee sign combinations include the four named schemes and mixed pairs.
    for (unsigned mask = 0; mask < 16; ++mask) {
        for (std::size_t goal = 0; goal < schemes.size(); ++goal) {
            const JointVector initial = pose(mask);
            const auto result = plan_switch_kin_sch(initial, schemes[goal]);
            require(std::holds_alternative<SwitchKinSchPlan>(result), "Valid pose rejected");
            const auto plan = std::get<SwitchKinSchPlan>(result);
            for (int leg = 0; leg < 4; ++leg) {
                const int j = 3 * leg;
                const bool flip = (initial(j + 2) > 0.0) != (knee_signs[goal][leg] > 0);
                require(plan.target(j) == initial(j), "Abad changed");
                require(plan.target(j + 1) == (flip ? -initial(j + 1) : initial(j + 1)), "Hip symmetry lost");
                require(plan.target(j + 2) == (flip ? -initial(j + 2) : initial(j + 2)), "Knee symmetry lost");
            }
            Controller controller;
            controller.enable_outputs();
            auto state = observation(initial);
            state.body.pos.z() = 0.27;
            controller.set_robot_state(state);
            SwitchKinSchConfig config;
            SafetyMonitor safety(limits(), 1.0);
            SwitchKinSchAction action(controller, config);
            action.reset(plan);
            LocomotionSetpoints setpoints;
            setpoints.robot_cmd_received_at_s = state.received_at_s;
            setpoints.robot_cmd.pos << 0.02, -0.03, 0.22;
            setpoints.robot_cmd.lin_vel.setOnes();
            setpoints.robot_cmd.ang_vel.setOnes();
            auto step = [&](double at) {
                state.received_at_s = at;
                controller.set_robot_state(state);
                const auto update = action.step(setpoints, safety);
                require(std::holds_alternative<SwitchKinSchAction::Progress>(update), "Unexpected action fault");
                return std::get<SwitchKinSchAction::Progress>(update);
            };
            if (mask == scheme_mask(goal)) {
                require(step(state.received_at_s) == SwitchKinSchAction::Progress::finished, "No-op did not finish");
                require(controller.handoffs == 0 && controller.joint_commands == 0 && controller.robot_commands == 0,
                        "No-op emitted commands");
                continue;
            }
            double now = state.received_at_s;
            require(step(now) == SwitchKinSchAction::Progress::running, "Motion finished at entry");
            for (int i = 0; i < 20 && controller.handoffs == 0; ++i) step(now += 0.01);
            require(controller.handoffs == 1 && controller.joints.position.isApprox(initial), "Entry pose lost");
            step(now);
            require(controller.joint_commands == 1, "Duplicate telemetry advanced action");
            for (int i = 0; i < 3; ++i) step(now += 0.01);
            require(controller.joint_commands == 1, "Motion started before joint handoff completed");
            controller.joint_ready = true;
            step(now += 0.01);
            JointVector from = initial;
            for (int pair = 0; pair < 2; ++pair) {
                JointVector target = plan.target;
                if (pair == 0) target.tail<6>() = initial.tail<6>();
                const JointVector delta = target - from;
                const double duration = 1.5 * delta.cwiseAbs().maxCoeff() / config.max_joint_speed_rad_s;
                const double start = now;
                for (double fraction : {0.25, 0.5, 0.75, 1.0}) {
                    now = start + duration * fraction + (fraction == 1.0 ? 1e-6 : 0.0);
                    if (duration == 0.0 && fraction != 1.0) continue;
                    step(now);
                    const double u = fraction;
                    const auto& cmd = controller.joints;
                    const JointVector expected = from + u * u * (3.0 - 2.0 * u) * delta;
                    const JointVector velocity = duration == 0.0 ? JointVector::Zero().eval()
                        : (6.0 * u * (1.0 - u) / duration * delta).eval();
                    require(cmd.position.isApprox(expected, 1e-8), "Pair order or cubic position incorrect");
                    require((cmd.velocity - velocity).norm() < 1e-8, "Cubic velocity incorrect");
                    require(cmd.velocity.cwiseAbs().maxCoeff() <= config.max_joint_speed_rad_s + 1e-8,
                            "Joint speed limit exceeded");
                    require(cmd.kp.isConstant(config.kp) && cmd.kd.isConstant(config.kd), "Action gains changed");
                    for (int leg = 0; leg < 4; ++leg) {
                        if (initial(3 * leg + 2) == plan.target(3 * leg + 2)) {
                            require(cmd.position.segment<3>(3 * leg).isApprox(initial.segment<3>(3 * leg)),
                                    "Correctly oriented leg moved");
                        }
                    }
                }
                require(controller.joints.position.isApprox(target) && controller.joints.velocity.isZero(0.0),
                        "Pair endpoint was not published at rest");
                from = target;
            }
            step(now += 0.01);
            require(controller.handoffs == 2 && controller.source() == Controller::robot_input,
                    "Robot handoff missing");
            require(step(now += 0.01) == SwitchKinSchAction::Progress::running, "Did not wait for WBIC readiness");
            controller.robot_ready = true;
            bool finished = false;
            for (int i = 0; i < 500 && !finished; ++i) {
                if (controller.height) {
                    state.body.pos.z() = controller.height->world_z_m;
                    state.body.lin_vel.z() = controller.height->velocity_mps;
                }
                finished = step(now += 0.01) == SwitchKinSchAction::Progress::finished;
            }
            require(finished && !controller.height, "Return height did not settle or override leaked");
            require(std::abs(state.body.pos.z() - setpoints.robot_cmd.pos.z()) < 0.005 &&
                    controller.body.pos.isApprox(setpoints.robot_cmd.pos) && controller.body.lin_vel.isZero() &&
                    controller.body.ang_vel.isZero(), "Standing command not restored");
        }
    }
    for (std::size_t source = 0; source < schemes.size(); ++source) {
        for (auto goal : schemes) {
            const auto initial = pose(scheme_mask(source));
            const auto first = std::get<SwitchKinSchPlan>(plan_switch_kin_sch(initial, goal));
            const auto back = std::get<SwitchKinSchPlan>(plan_switch_kin_sch(first.target, schemes[source]));
            require(back.target == initial, "Round trip changed measured angle magnitudes");
        }
    }
}

JointLimitList limits() {
    JointLimitList result;
    for (int i = 0; i < 12; ++i) result.emplace_back(std::to_string(i), JointLimits{-3.1, 3.1});
    return result;
}

struct Fixture {
    explicit Fixture(JointLimitList joint_limits = limits())
        : safety(std::move(joint_limits), 1.0), fsm(safety, &status, &controller) {
        state = observation(pose(scheme_mask(0)));
        controller.set_robot_state(state);
        controller.enable_outputs();
        fsm.state_id = StateID::STANDING;
    }
    void tick() {
        state.received_at_s += 0.025;
        if (track_height && controller.height) {
            state.body.pos.z() = controller.height->world_z_m;
            state.body.lin_vel.z() = controller.height->velocity_mps;
        }
        controller.set_robot_state(state);
        fsm.update(setpoints);
    }
    Controller controller;
    RobotStatus status;
    SafetyMonitor safety;
    RobotFSM fsm;
    RobotStateSnapshot state;
    LocomotionSetpoints setpoints;
    bool track_height = true;
};

void fsm_guards_and_completion() {
    for (auto state : {StateID::IDLE, StateID::STANDUP, StateID::LOCOMOTION, StateID::LAYDOWN,
                       StateID::ACTION, StateID::EXTERNAL, StateID::FAILURE}) {
        Fixture test;
        test.fsm.state_id = state;
        require(test.fsm.start_switch_kinematics(KneeScheme::outside).has_value(), "Started outside STANDING");
        require(test.controller.handoffs == 0, "Rejected start changed controller");
    }
    Fixture test;
    require(!test.fsm.request_state(StateID::ACTION), "Generic route entered ACTION");
    require(!test.fsm.handle_event(Event::START_ACTION), "Unprepared START_ACTION accepted");
    require(!test.fsm.start_switch_kinematics(KneeScheme::outside), "Valid FSM start rejected");
    require(test.fsm.state_id == StateID::ACTION && test.fsm.target_state == StateID::STANDING, "Action target wrong");
    require(test.fsm.start_switch_kinematics(KneeScheme::inside).has_value(), "Repeated action accepted");
    require(!test.fsm.handle_event(Event::START_STANDING), "Action exited early");
    require(!test.fsm.request_state(StateID::IDLE), "Mode change accepted during action");
    for (int i = 0; i < 1500 && test.controller.handoffs < 2; ++i) {
        if (test.controller.handoffs == 1) test.controller.joint_ready = true;
        test.tick();
    }
    require(test.controller.handoffs == 2 && test.fsm.state_id == StateID::ACTION, "Return handoff not awaited");
    require(test.controller.holds > 0, "Missing external command did not hold measured pose");
    test.controller.robot_ready = true;
    for (int i = 0; i < 200 && test.fsm.state_id == StateID::ACTION; ++i) test.tick();
    require(test.fsm.state_id == StateID::STANDING && test.controller.source() == Controller::robot_input,
            "Normal completion did not return to robot_input/STANDING");
    Fixture noop;
    require(!noop.fsm.start_switch_kinematics(KneeScheme::inside), "No-op rejected");
    noop.tick();
    require(noop.fsm.state_id == StateID::STANDING && noop.controller.handoffs == 0, "No-op moved or did not finish");
}

void invalid_input() {
    for (double bad : {0.0, -0.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        Fixture test;
        test.state.joints.position(2) = bad;
        test.controller.set_robot_state(test.state);
        require(test.fsm.start_switch_kinematics(KneeScheme::outside).has_value(), "Invalid knee accepted");
        require(test.controller.handoffs == 0, "Invalid knee caused handoff");
    }
    Fixture invalid;
    require(invalid.fsm.start_switch_kinematics(static_cast<KneeScheme>(99)).has_value(), "Invalid scheme accepted");
    auto asymmetric = limits();
    asymmetric[1].second = JointLimits{-3.1, 0.1};
    Fixture target_limit(std::move(asymmetric));
    const auto reason = target_limit.fsm.start_switch_kinematics(KneeScheme::outside);
    require(reason && reason->find("Target joint limit") != std::string::npos, "Target limit not rejected before start");
    require(target_limit.fsm.state_id == StateID::STANDING && target_limit.controller.handoffs == 0,
            "Target rejection moved robot");
    for (auto field : {&SwitchKinSchConfig::kp, &SwitchKinSchConfig::kd,
                       &SwitchKinSchConfig::max_joint_speed_rad_s, &SwitchKinSchConfig::handoff_s,
                       &SwitchKinSchConfig::preparation_height_m, &SwitchKinSchConfig::max_body_height_speed_mps,
                       &SwitchKinSchConfig::height_motion_min_s}) {
        for (double value : {0.0, -1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
            if (field == &SwitchKinSchConfig::kd && value == 0.0) continue;
            auto config = SwitchKinSchConfig{};
            config.*field = value;
            bool rejected = false;
            try { SwitchKinSchAction action(invalid.controller, config); }
            catch (const std::invalid_argument&) { rejected = true; }
            require(rejected, "Invalid action configuration accepted");
        }
    }
}

void faults_at_each_phase() {
    for (int phase = 0; phase < 8; ++phase) {
        for (int fault = 0; fault < 4; ++fault) {
            Fixture test;
            require(!test.fsm.start_switch_kinematics(KneeScheme::outside), "Cannot start fault test");
            const auto target = std::get<SwitchKinSchPlan>(plan_switch_kin_sch(
                test.state.joints.position, KneeScheme::outside)).target;
            bool reached = false;
            for (int i = 0; i < 1500; ++i) {
                test.tick();
                const auto& c = test.controller;
                reached = phase == 0 ? c.handoffs == 0 && c.height && c.height->velocity_mps > 0.01
                    : phase == 1 ? c.handoffs == 0 && c.height && c.height->world_z_m == 0.27 && c.height->velocity_mps == 0.0
                    : phase == 2 ? c.handoffs == 1
                    : phase == 3 ? c.joints.velocity.head<6>().norm() > 0.01
                    : phase == 4 ? c.joints.position.head<6>().isApprox(target.head<6>()) && c.joints.velocity.tail<6>().norm() > 0.01
                    : phase == 5 ? c.handoffs == 2
                    : phase == 6 ? c.handoffs == 2 && c.height && c.height->velocity_mps < -0.01
                    : c.handoffs == 2 && c.height && c.height->world_z_m == 0.20 && c.height->velocity_mps == 0.0;
                if (reached) break;
                test.controller.joint_ready = true;
                if (phase >= 6 && test.controller.handoffs == 2) test.controller.robot_ready = true;
            }
            require(reached, "Fault test phase not reached");
            const int commands = test.controller.joint_commands + test.controller.robot_commands + test.controller.holds;
            const SafetyFault error{"TEST", "Injected fault", monotonic_seconds()};
            if (fault == 0) test.fsm.handle_event(Event::ESTOP);
            if (fault == 1) test.status.safety.telemetry_fault = error;
            if (fault == 2) test.status.safety.joint_fault = error;
            if (fault == 3) test.controller.fault = error;
            test.tick();
            require(test.fsm.state_id == StateID::FAILURE && !test.controller.outputs_enabled(), "Fault did not stop action");
            require(!test.controller.height, "Aborted action retained height override");
            require(commands == test.controller.joint_commands + test.controller.robot_commands + test.controller.holds,
                    "Aborted action emitted another control command");
        }
    }
}

void height_profiles_and_gates() {
    for (auto adaptation : {NOADAPT, HEIGHT_ADAPT, INCL_ADAPT}) {
        for (double start_z : {0.20, 0.34}) {
            Controller controller;
            controller.enable_outputs();
            auto state = observation(pose(scheme_mask(2)));
            state.received_at_s = 0.0;
            state.body.pos.z() = start_z;
            for (auto foot : {&LegData::r1_pos, &LegData::l1_pos, &LegData::r2_pos, &LegData::l2_pos})
                (state.legs.*foot).z() = 0.04;
            controller.set_robot_state(state);
            SwitchKinSchConfig config;
            SwitchKinSchAction action(controller, config);
            action.reset(std::get<SwitchKinSchPlan>(plan_switch_kin_sch(state.joints.position, KneeScheme::outside)));
            SafetyMonitor safety(limits(), 1.0);
            LocomotionSetpoints setpoints;
            setpoints.adaptation_type = adaptation;
            const double target = config.preparation_height_m + (adaptation == NOADAPT ? 0.0 : 0.04);
            const double duration = std::max(config.height_motion_min_s,
                1.5 * std::abs(target - start_z) / config.max_body_height_speed_mps);
            for (double u : {0.0, 0.25, 0.5, 0.75, 1.0}) {
                state.received_at_s = u * duration;
                controller.set_robot_state(state);
                const auto result = action.step(setpoints, safety);
                require(std::holds_alternative<SwitchKinSchAction::Progress>(result), "Height trajectory faulted");
                const double z = start_z + u * u * (3.0 - 2.0 * u) * (target - start_z);
                const double v = 6.0 * u * (1.0 - u) * (target - start_z) / duration;
                require(controller.height && std::abs(controller.height->world_z_m - z) < 1e-12 &&
                        std::abs(controller.height->velocity_mps - v) < 1e-12, "Wrong body cubic position or velocity");
                require(std::abs(v) <= config.max_body_height_speed_mps + 1e-12 && controller.handoffs == 0,
                        "Exceeded body speed or switched before measured height reached target");
            }
            state.received_at_s += 0.2;
            controller.set_robot_state(state);
            action.step(setpoints, safety);
            require(controller.handoffs == 0, "Elapsed trajectory alone allowed joint takeover");
            state.body.pos.z() = target;
            state.body.lin_vel.z() = 0.03;
            for (int i = 0; i < 20; ++i) {
                state.received_at_s += .01;
                controller.set_robot_state(state);
                action.step(setpoints, safety);
            }
            require(controller.handoffs == 0, "Moving body allowed joint takeover");
        }
    }
    Fixture timeout;
    timeout.track_height = false;
    require(!timeout.fsm.start_switch_kinematics(KneeScheme::outside), "Cannot start height timeout test");
    for (int i = 0; i < 250 && timeout.fsm.state_id == StateID::ACTION; ++i) timeout.tick();
    require(timeout.fsm.state_id == StateID::FAILURE &&
            timeout.safety.fault()->code == "SWITCH_KIN_SCH_HEIGHT_TIMEOUT" && timeout.controller.handoffs == 0,
            "Unreached preparation height did not time out safely");

    Fixture contact;
    require(!contact.fsm.start_switch_kinematics(KneeScheme::outside), "Cannot start contact test");
    contact.state.legs.contacts[0] = false;
    for (int i = 0; i < 8 && contact.fsm.state_id == StateID::ACTION; ++i) contact.tick();
    require(contact.fsm.state_id == StateID::FAILURE && contact.safety.fault()->code == "SWITCH_KIN_SCH_CONTACT_LOST",
            "Height motion ignored lost support");
}

void replan_and_latest_height() {
    Fixture test;
    test.setpoints.robot_cmd_received_at_s = test.state.received_at_s;
    test.setpoints.adaptation_type = NOADAPT;
    test.setpoints.robot_cmd.pos.z() = .20;
    for (auto foot : {&LegData::r1_pos, &LegData::l1_pos, &LegData::r2_pos, &LegData::l2_pos})
        (test.state.legs.*foot).z() = .04;
    const JointVector raised = 0.75 * test.state.joints.position;
    require(!test.fsm.start_switch_kinematics(KneeScheme::outside), "Cannot start replanning test");
    for (int i = 0; i < 200 && test.controller.handoffs == 0; ++i) {
        if (test.controller.height && test.controller.height->world_z_m == .27) test.state.joints.position = raised;
        test.tick();
    }
    require(test.controller.handoffs == 1 && test.controller.joints.position.isApprox(raised),
            "Joint takeover reused angles measured before body lift");
    test.setpoints.robot_cmd.pos.z() = .23;
    for (int i = 0; i < 1500 && test.controller.handoffs < 2; ++i) {
        test.controller.joint_ready = true;
        test.tick();
    }
    const auto target = std::get<SwitchKinSchPlan>(plan_switch_kin_sch(raised, KneeScheme::outside)).target;
    require(test.controller.joints.position.isApprox(target), "Raised joint target was not mirrored");
    test.controller.robot_ready = true;
    bool updated = false, reached_old_target = false;
    for (int i = 0; i < 400 && test.fsm.state_id == StateID::ACTION; ++i) {
        if (!updated && test.controller.height && test.controller.height->velocity_mps < -.01) {
            test.setpoints.robot_cmd.pos.z() = .25;
            test.setpoints.adaptation_type = HEIGHT_ADAPT;
            updated = true;
        }
        if (test.controller.height && test.controller.height->world_z_m == .23 && test.controller.height->velocity_mps == 0.0)
            reached_old_target = true;
        test.tick();
    }
    require(updated && reached_old_target, "Return target changed before completing its cubic segment");
    require(test.fsm.state_id == StateID::STANDING && !test.controller.height &&
            std::abs(test.state.body.pos.z() - .29) < .005, "Action did not restore latest external height");
    for (const auto& [time, height] : test.controller.heights) {
        (void)time;
        require(std::abs(height.velocity_mps) <= .05 + 1e-12, "Body speed limit exceeded after retargeting");
    }

    auto asymmetric = limits();
    asymmetric[1].second = JointLimits{-3.1, .6};
    Fixture invalid_target(std::move(asymmetric));
    require(!invalid_target.fsm.start_switch_kinematics(KneeScheme::outside), "Initial target should be valid");
    for (int i = 0; i < 250 && invalid_target.fsm.state_id == StateID::ACTION; ++i) {
        if (invalid_target.controller.height && invalid_target.controller.height->world_z_m == .27)
            invalid_target.state.joints.position(1) = -.8;
        invalid_target.tick();
    }
    require(invalid_target.fsm.state_id == StateID::FAILURE && invalid_target.controller.handoffs == 0 &&
            invalid_target.safety.fault()->code == "JOINT_LIMIT", "Replanned target limits were not checked");
}

void return_height_reference_and_invalid_state() {
    for (auto adaptation : {NOADAPT, HEIGHT_ADAPT, INCL_ADAPT}) {
        for (bool has_command : {false, true}) {
            Fixture test;
            test.state.body.pos.z() = .24;
            for (auto foot : {&LegData::r1_pos, &LegData::l1_pos, &LegData::r2_pos, &LegData::l2_pos})
                (test.state.legs.*foot).z() = .04;
            test.controller.set_robot_state(test.state);
            test.setpoints.adaptation_type = adaptation;
            test.setpoints.robot_cmd.pos.z() = .21;
            if (has_command) test.setpoints.robot_cmd_received_at_s = test.state.received_at_s;
            require(!test.fsm.start_switch_kinematics(KneeScheme::outside), "Cannot start return reference test");
            for (int i = 0; i < 1500 && test.fsm.state_id == StateID::ACTION; ++i) {
                if (test.controller.handoffs == 1) test.controller.joint_ready = true;
                if (test.controller.handoffs == 2) test.controller.robot_ready = true;
                test.tick();
            }
            const double expected = has_command ? .21 + (adaptation == NOADAPT ? 0.0 : .04) : .24;
            require(test.fsm.state_id == StateID::STANDING && !test.controller.height &&
                    std::abs(test.state.body.pos.z() - expected) < .005, "Wrong return height reference");
        }
    }
    for (int invalid = 0; invalid < 3; ++invalid) {
        Fixture test;
        require(!test.fsm.start_switch_kinematics(KneeScheme::outside), "Cannot start invalid state test");
        if (invalid == 0) test.setpoints.adaptation_type = 255;
        if (invalid == 1) test.state.legs.r1_pos.z() = std::numeric_limits<double>::quiet_NaN();
        if (invalid == 2) test.state.body.lin_vel.z() = std::numeric_limits<double>::quiet_NaN();
        test.tick();
        require(test.fsm.state_id == StateID::FAILURE && !test.controller.height,
                "Invalid height observation was not safely rejected");
    }
}

void handoff_profiles() {
    for (bool moving_destination : {false, true}) {
        JointCommand from, to;
        from.position.setConstant(-0.3);
        from.velocity.setConstant(0.02);
        from.kp.setConstant(2.0);
        from.kd.setConstant(0.7);
        from.torque.setConstant(3.0);
        to.position.setConstant(0.4);
        to.velocity.setConstant(moving_destination ? 0.03 : 0.0);
        to.kp.setConstant(12.0);
        to.kd.setConstant(0.4);
        to.torque.setConstant(-2.0);
        JointCommandHandoff handoff(from, to, 0.01, 1.0);
        require(handoff.duration() > 0.01, "Handoff did not account for peak joint speed");
        auto destination = [&](double time) {
            auto value = to;
            value.position += time * to.velocity;
            return value;
        };
        auto at = [&](double time) { return handoff.sample(destination(time), time); };
        const auto start = at(0.0);
        require(start.position.isApprox(from.position) && start.velocity.isApprox(from.velocity) &&
                start.kp.isApprox(from.kp) && start.kd.isApprox(from.kd) && start.torque.isApprox(from.torque),
                "Handoff did not start from full published command");
        for (int i = 1; i < 1000; ++i) {
            const double time = i * handoff.duration() / 1000.0;
            const auto cmd = at(time);
            const double h = 1e-6;
            const JointVector derivative = (at(time + h).position - at(time - h).position) / (2.0 * h);
            require((cmd.velocity - derivative).norm() < 1e-7, "Handoff velocity inconsistent with position");
            require(cmd.velocity.cwiseAbs().maxCoeff() <= 1.0 + 1e-9, "Handoff exceeded speed limit");
            require(cmd.kp.minCoeff() >= 2.0 && cmd.kp.maxCoeff() <= 12.0 &&
                    cmd.torque.minCoeff() >= -2.0 && cmd.torque.maxCoeff() <= 3.0, "Gains or torque overshot");
        }
        const auto final = at(handoff.duration());
        const auto target = destination(handoff.duration());
        require(final.position == target.position && final.velocity == target.velocity &&
                final.kp == to.kp && final.kd == to.kd && final.torque == to.torque, "Handoff endpoint discontinuous");
    }
    JointCommand zero;
    JointCommandHandoff stationary(zero, zero, 0.4, 1.0);
    require(stationary.sample(zero, 0.2).position.allFinite(), "Zero displacement handoff divided by zero");
    require(stationary.sample(zero, 0.2).velocity.isZero(), "Zero displacement handoff moved");
}

void controller_timeout_and_abort() {
    // No sockets or external consumers are needed to exercise the real controller deadline.
    setenv("LCM_LOCOMOTION_URL", "memq://", 1);
    setenv("LCM_SERVO_URL", "memq://", 1);
    const auto config = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "config";
    LCMExchanger exchange(config);
    LocomotionControllerMPC controller(config, exchange);
    JointCommand command;
    command.position = pose(scheme_mask(0));
    command.kp.setConstant(12.0);
    controller.enable_outputs();
    controller.begin_input_handoff(Controller::joints_input, 0.4, 1.0);
    controller.set_joints_cmd(command);
    require(!controller.joints_control_ready(), "Unpublished joint handoff reported ready");
    controller.disable_outputs();
    controller.enable_outputs();
    controller.set_data_input_mode(Controller::joints_input);
    controller.set_joints_cmd(command);
    require(controller.joints_control_ready(), "Aborted handoff leaked into next activation");
    controller.begin_input_handoff(Controller::robot_input, 0.4, 1.0);
    controller.hold_measured_pose();
    require(!controller.robot_control_ready(), "Robot ready before first WBIC command");
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    const auto fault = controller.controller_fault();
    require(fault && fault->code == "WBIC_HANDOFF_TIMEOUT" && !controller.outputs_enabled(),
            "First-WBIC deadline was replaced by blend duration");
    require(!controller.robot_control_ready(), "Timed out controller reported ready");

    controller.disable_outputs();
    controller.enable_outputs();
    controller.set_data_input_mode(Controller::wbc_input);
    auto state = observation(command.position);
    state.body.pos.x() = std::numeric_limits<double>::quiet_NaN();
    controller.set_robot_state(state);
    controller.set_wbc_cmd(WbcDesiredCommand{});
    controller.start_thread();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!controller.controller_fault() && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const auto wbic_fault = controller.controller_fault();
    require(wbic_fault && wbic_fault->code == "WBIC" && !controller.outputs_enabled(), "WBIC error was not propagated");
    controller.stop_thread();
}

} // namespace

int main() {
    try {
        all_plans_and_trajectories();
        fsm_guards_and_completion();
        invalid_input();
        height_profiles_and_gates();
        replan_and_latest_height();
        return_height_reference_and_invalid_state();
        faults_at_each_phase();
        handoff_profiles();
        controller_timeout_and_abort();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
