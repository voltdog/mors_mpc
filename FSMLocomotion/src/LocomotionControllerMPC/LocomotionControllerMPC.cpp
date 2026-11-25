#include "LocomotionControllerMPC.hpp"
#include "LcmDataExchange.hpp"
#include "VisionConfig.hpp"
#include "ContactStateFSM.hpp"
#include "ConvexMpcThread.hpp"
#include "GaitTransition.hpp"
#include "ReferenceGenerator.hpp"
#include "SwingController.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mors::fsm {
namespace {

using Clock = std::chrono::steady_clock;
const std::vector<int> all_stance(NUM_LEGS, STANCE);
constexpr std::array positions{&LegData::r1_pos, &LegData::l1_pos, &LegData::r2_pos, &LegData::l2_pos};
constexpr std::array velocities{&LegData::r1_vel, &LegData::l1_vel, &LegData::r2_vel, &LegData::l2_vel};
constexpr std::array accelerations{&LegData::r1_acc, &LegData::l1_acc, &LegData::r2_acc, &LegData::l2_acc};
constexpr std::array forces{&LegData::r1_grf, &LegData::l1_grf, &LegData::r2_grf, &LegData::l2_grf};

bool support(int phase) { return phase == STANCE || phase == EARLY_CONTACT; }

// Всё состояние шага принадлежит потоку 2 мс. Новый цикл включения начинает его со стойки.
struct GaitLoop {
    GaitLoop(const VisionConfig& cfg, const LocomotionSetpoints& cmd)
        : scheduler(cfg.dt),
          contacts(cfg.touchdown_phase, cfg.contact_debounce, cfg.contact_velocity),
          reference(cfg.dt, cfg.filter_hz, cfg.zero_velocity, cfg.foot_radius),
          swing(cfg.dt, cfg.robot.bx, cfg.robot.by, cfg.robot.l1,
                std::hypot(cfg.robot.d1 + cfg.robot.d2 + cfg.robot.d3, cfg.robot.l2 + cfg.robot.l3),
                cfg.interleave_x, cfg.interleave_y, cfg.near_ground_velocity,
                cfg.rising_proportion, cfg.footstep_gain) {
        scheduler.set_gait_params(cmd.t_st, cmd.t_sw, cmd.gait_type, all_stance);
        scheduler.reset();
        transition.set_gait_params(cmd.t_st, cmd.t_sw, cmd.gait_type, cmd.stride_height);
        transition.set_transition_duration(cfg.transition_s);
    }

    SimpleGaitScheduler scheduler;
    ContactStateFSM contacts;
    ReferenceGenerator reference;
    SwingController swing;
    GaitTransition transition;
    std::vector<int> phase = all_stance, scheduled = all_stance;
    std::vector<double> phi = std::vector<double>(4);
    std::vector<Eigen::Vector3d> feet = std::vector<Eigen::Vector3d>(4);
    std::vector<Eigen::Vector3d> landing = std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero());
    std::array<Eigen::Vector3d, 4> stance_feet{};
    bool was_standing = false;
    std::uint64_t map_revision = 0;
    double t = 0;
};

struct RobotInput {
    RobotInput() { setpoints.mode = LocomotionMode::STANDING; }
    LocomotionSetpoints setpoints;
    std::optional<RobotStateSnapshot> standing_pose;
    std::optional<Eigen::Vector2d> standing_xy;
    bool hold = false;
    bool stop = false;
};

} // namespace

struct LocomotionControllerMPC::Impl {
    Impl(const std::filesystem::path& path, LCMExchanger& exchange)
        : exchange(exchange), config(path) {
        auto model = path / "../common/urdf/mors.urdf";
        if (!std::filesystem::exists(model))
            model = std::filesystem::path(FSM_DEFAULT_CONFIG_DIR) / "../common/urdf/mors.urdf";
        wbic.configure(config.robot, config.wbic, model.string());
        mpc.set_physical_params(config.robot);
        mpc.set_mpc_params(config.mpc_dt, config.horizon, config.friction, config.f_min, config.f_max,
                           config.Q, config.R);
        idle_command.kd.setConstant(config.kd_idle);
    }

    void publish(const JointCommand& command) {
        exchange.sendServoCmd(command, wbic.actuator_torque(command.torque));
    }

    LCMExchanger& exchange;
    VisionConfig config;
    ConvexMPCThread mpc;
    WBICThread wbic;
    std::mutex workers;
    mutable std::mutex mutex;
    RobotInput pending, active;
    std::optional<uint8_t> source;
    std::uint64_t generation = 0;
    std::optional<WbcDesiredCommand> desired;
    JointCommand joints;
    JointCommand idle_command;
    std::optional<Clock::time_point> handoff_deadline;
    std::optional<SafetyFault> fault;
    bool stopped = false;
};

LocomotionControllerMPC::LocomotionControllerMPC(const std::filesystem::path& path, LCMExchanger& exchange)
    : impl_(std::make_unique<Impl>(path, exchange)) {}

LocomotionControllerMPC::~LocomotionControllerMPC() {
    stop_thread();
}

void LocomotionControllerMPC::start_thread() {
    auto& c = *impl_;
    std::lock_guard lock(c.workers);
    try {
        c.mpc.start_thread();
        c.wbic.start_thread([this] { wbic_tick(); });
        LocomotionControlBase::start_thread();
    } catch (...) {
        disable_outputs();
        c.wbic.stop_thread();
        c.mpc.stop_thread();
        throw;
    }
}

void LocomotionControllerMPC::stop_thread() {
    auto& c = *impl_;
    std::lock_guard lock(c.workers);
    disable_outputs();
    LocomotionControlBase::stop_thread();
    c.wbic.stop_thread();
    c.mpc.stop_thread();
}

void LocomotionControllerMPC::disable_outputs() {
    auto& c = *impl_;
    std::lock_guard lock(c.mutex);
    LocomotionControlBase::disable_outputs();
    ++c.generation;
    c.source.reset();
    c.desired.reset();
    c.handoff_deadline.reset();
    c.joints = {};
    c.active = c.pending = {};
    c.stopped = false;
    c.fault.reset();
    c.mpc.reset();
    // Тот же mutex охватывает последнюю проверку разрешения и реальный publish().
    c.publish(c.idle_command);
}

void LocomotionControllerMPC::enable_outputs() {
    std::lock_guard lock(impl_->mutex);
    LocomotionControlBase::enable_outputs();
}

void LocomotionControllerMPC::set_data_input_mode(const uint8_t& mode) {
    if (mode < robot_input || mode > joints_input) throw std::invalid_argument("input mode");
    std::lock_guard lock(impl_->mutex);
    input_mode = mode;
}

// Вызывается под mutex вместе с записью первой уставки выбранного типа.
bool LocomotionControllerMPC::accept_input(uint8_t mode) {
    auto& c = *impl_;
    check_deadline();
    if (!outputs_enabled() || c.fault) return false;
    if (input_mode != mode) throw std::logic_error("Command does not match requested input mode");
    if (c.source != mode) {
        if (c.source == joints_input) {
            c.handoff_deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(c.config.handoff_s));
        }
        if (mode == joints_input) c.handoff_deadline.reset();
        ++c.generation;
        c.desired.reset();
        c.mpc.reset();
        c.source = mode;
    }
    return true;
}

void LocomotionControllerMPC::set_joints_cmd(const JointCommand& cmd) {
    auto& c = *impl_;
    std::lock_guard lock(c.mutex);
    if (!accept_input(joints_input)) return;
    c.joints = cmd;
}

void LocomotionControllerMPC::set_wbc_cmd(const WbcDesiredCommand& cmd) {
    std::lock_guard lock(impl_->mutex);
    if (accept_input(wbc_input)) impl_->desired = cmd;
}

void LocomotionControllerMPC::set_locomotion_mode(const uint8_t& mode) {
    auto& c = *impl_;
    std::lock_guard lock(c.mutex);
    switch (mode) {
    case standing_mode:
        c.pending.setpoints.mode = LocomotionMode::STANDING;
        c.pending.standing_pose = robot_state();
        break;
    case walking_mode: c.pending.setpoints.mode = LocomotionMode::WALKING; break;
    case recovery_mode: c.pending.setpoints.mode = LocomotionMode::RECOVERY; break;
    default: throw std::invalid_argument("locomotion mode");
    }
}

void LocomotionControllerMPC::set_standing_xy(const Eigen::Vector2d& xy) {
    std::lock_guard lock(impl_->mutex);
    impl_->pending.standing_xy = xy;
}

void LocomotionControllerMPC::set_gait_params(const double& stance, const double& swing,
                                               const std::vector<double>& offsets, const double& height) {
    if (!(std::isfinite(stance) && stance > 0 && std::isfinite(swing) && swing > 0 &&
          std::isfinite(height) && height >= 0 && offsets.size() == NUM_LEGS &&
          std::ranges::all_of(offsets, [](double x) { return x >= 0 && x < 1; }))) return;
    std::lock_guard lock(impl_->mutex);
    auto& gait = impl_->pending.setpoints;
    gait.t_st = stance;
    gait.t_sw = swing;
    gait.gait_type = offsets;
    gait.stride_height = height;
}

void LocomotionControllerMPC::set_active_legs(const std::vector<bool>& legs) {
    if (legs.size() != NUM_LEGS) return;
    std::lock_guard lock(impl_->mutex);
    impl_->pending.setpoints.active_legs = legs;
}

void LocomotionControllerMPC::set_adaptation_type(const uint8_t& mode) {
    std::lock_guard lock(impl_->mutex);
    impl_->pending.setpoints.adaptation_type = mode;
}

void LocomotionControllerMPC::hold_measured_pose() { commit_robot_command(nullptr); }
void LocomotionControllerMPC::set_robot_cmd(const RobotData& cmd) { commit_robot_command(&cmd); }

bool LocomotionControllerMPC::robot_control_ready() const {
    const auto& c = *impl_;
    std::lock_guard lock(c.mutex);
    return outputs_enabled() && c.source == robot_input && !c.handoff_deadline && !c.fault;
}

void LocomotionControllerMPC::commit_robot_command(const RobotData* cmd) {
    auto& c = *impl_;
    std::lock_guard lock(c.mutex);
    if (!accept_input(robot_input)) return;
    c.pending.hold = !cmd;
    if (cmd) c.pending.setpoints.robot_cmd = *cmd;
    c.active = c.pending;
    c.stopped = false;
}

void LocomotionControllerMPC::request_gait_stop() {
    auto& c = *impl_;
    std::lock_guard lock(c.mutex);
    c.active.stop = true;
    c.active.setpoints.robot_cmd.lin_vel.setZero();
    c.active.setpoints.robot_cmd.ang_vel.setZero();
}

bool LocomotionControllerMPC::gait_stopped() const {
    std::lock_guard lock(impl_->mutex);
    return impl_->stopped;
}

void LocomotionControllerMPC::check_deadline() {
    auto& c = *impl_;
    if (c.handoff_deadline && Clock::now() >= *c.handoff_deadline) {
        LocomotionControlBase::disable_outputs();
        c.handoff_deadline.reset();
        c.fault = SafetyFault{"WBIC_HANDOFF_TIMEOUT", "First WBIC command timed out", monotonic_seconds()};
        c.publish(c.idle_command);
    }
}

std::optional<SafetyFault> LocomotionControllerMPC::controller_fault() {
    std::lock_guard lock(impl_->mutex);
    check_deadline();
    return impl_->fault;
}

void LocomotionControllerMPC::wbic_tick() {
    auto& c = *impl_;
    std::optional<WbcDesiredCommand> desired;
    std::uint64_t generation;
    {
        std::lock_guard lock(c.mutex);
        check_deadline();
        generation = c.generation;
        if (outputs_enabled()) desired = c.desired;
    }
    auto state = robot_state();
    std::optional<JointCommand> result;
    std::string error;
    if (desired && state) {
        try {
            result = c.wbic.step(state->body, state->joints, *desired);
            if (!result->torque.allFinite() || !result->position.allFinite() || !result->velocity.allFinite())
                throw std::runtime_error("Non-finite WBIC command");
        } catch (const std::exception& e) {
            result.reset();
            error = e.what();
        }
    }
    std::lock_guard lock(c.mutex);
    check_deadline();
    const bool current = outputs_enabled() && generation == c.generation;
    if (current && !error.empty() && !c.handoff_deadline) {
        c.fault = SafetyFault{"WBIC", error, monotonic_seconds()};
        LocomotionControlBase::disable_outputs();
    }
    if (current && result) {
        c.handoff_deadline.reset();
        c.publish(*result);
        if (c.config.wbic.debug_mode) c.exchange.sendWbcState(c.wbic.forces());
    } else if (!outputs_enabled()) {
        c.publish(c.idle_command);
    } else if (c.source == joints_input || c.handoff_deadline) {
        c.publish(c.joints);
    } else {
        c.publish({});
    }
}

void LocomotionControllerMPC::run(std::stop_token stop) {
    auto& c = *impl_;
    const auto& cfg = c.config;
    const auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(cfg.dt));
    auto next = Clock::now();
    std::optional<GaitLoop> loop;
    std::uint64_t last_generation = 0;
    VisionBasedMap map;
    while (!stop.stop_requested()) {
        std::this_thread::sleep_until(next);
        next += period;
        const auto now = Clock::now();
        if (next <= now) next += period * ((now - next) / period + 1);

        RobotInput input;
        std::uint64_t generation;
        std::optional<RobotStateSnapshot> state;
        {
            std::lock_guard lock(c.mutex);
            if (!outputs_enabled() || c.source != robot_input) continue;
            state = robot_state();
            if (!state) continue;
            // Удержание можно запросить до прихода первого ROBOT_STATE.
            if (c.active.hold && !c.active.standing_pose)
                c.pending.standing_pose = c.active.standing_pose = state;
            input = c.active;
            generation = c.generation;
        }
        auto& cmd = input.setpoints;
        const bool recovery = cmd.mode == LocomotionMode::RECOVERY;
        const bool standing = cmd.mode == LocomotionMode::STANDING;
        if (recovery) {
            cmd.t_st = cfg.recovery_stance;
            cmd.t_sw = cfg.recovery_swing;
            cmd.robot_cmd.lin_vel.setZero();
            cmd.robot_cmd.ang_vel.setZero();
        }
        if (!loop || generation != last_generation) {
            loop.emplace(cfg, cmd);
            last_generation = generation;
        }
        auto& g = *loop;
        if (c.exchange.read_heightmap(g.map_revision, map)) g.swing.set_heightmap(map);

        g.transition.set_gait_params(cmd.t_st, cmd.t_sw, cmd.gait_type, cmd.stride_height);
        g.transition.make_transition(g.t, cmd.t_st, cmd.t_sw, cmd.gait_type, cmd.stride_height);
        g.scheduler.set_gait_params(cmd.t_st, cmd.t_sw, cmd.gait_type, all_stance);
        // В режиме остановки планировщик доводит уже поднятые ноги до касания.
        {
            std::lock_guard lock(c.mutex);
            if (!outputs_enabled() || generation != c.generation) continue;
            input.stop = c.active.stop;
            g.scheduler.step(g.t, standing || input.stop, g.scheduled, g.phi);
        }
        if (input.stop) {
            cmd.robot_cmd.lin_vel.setZero();
            cmd.robot_cmd.ang_vel.setZero();
        }
        const auto previous = g.phase;
        std::vector<double> vz(NUM_LEGS);
        const auto& legs = state->legs;
        for (int i = 0; i < NUM_LEGS; ++i) {
            g.feet[i] = legs.*positions[i];
            vz[i] = (legs.*velocities[i]).z();
        }
        g.phase = g.contacts.step(legs.contacts, g.phi, g.scheduled, vz);
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (standing && !g.was_standing && input.standing_pose)
                g.stance_feet[i] = input.standing_pose->legs.*positions[i];
            else if (g.t == 0 || (support(g.phase[i]) && !support(previous[i])))
                g.stance_feet[i] = g.feet[i];
        }
        g.was_standing = standing;
        const auto& body = state->body;
        const Eigen::Matrix3d yaw = Eigen::AngleAxisd(body.orientation.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        if (cfg.local_velocity) cmd.robot_cmd.lin_vel = yaw * cmd.robot_cmd.lin_vel;
        if (input.hold && input.standing_pose) {
            const auto& start = *input.standing_pose;
            cmd.robot_cmd = {};
            cmd.robot_cmd.pos.z() = start.body.pos.z();
            if (cmd.adaptation_type != NOADAPT) {
                for (auto pos : positions) cmd.robot_cmd.pos.z() -= (start.legs.*pos).z() / NUM_LEGS;
            }
        }
        g.reference.set_body_adaptation_mode(cmd.adaptation_type);
        auto ref = g.reference.step(g.phase, g.feet, g.landing, cmd.robot_cmd, body);
        if (input.hold && input.standing_pose) {
            // Абсолютная начальная поза уже включает yaw, уклон и высоту опор.
            ref.head<3>() = input.standing_pose->body.orientation;
            ref.segment<3>(3) = input.standing_pose->body.pos;
        }
        if (recovery && input.standing_xy) ref.segment<2>(3) = *input.standing_xy;
        if (recovery || input.hold) ref.segment<6>(6).setZero();

        std::optional<Eigen::VectorXd> grf;
        {
            std::lock_guard lock(c.mutex);
            if (generation != c.generation || !outputs_enabled()) continue;
            c.mpc.set_gait_params(cmd.t_st, cmd.t_sw, cmd.gait_type, all_stance);
            c.mpc.set_observation_data(body, legs, ref, yaw, true, standing || input.stop,
                                       g.phase, g.scheduler.get_phi(), cmd.active_legs);
            grf = c.mpc.solution();
        }
        g.swing.set_gait_params(cmd.t_sw, cmd.t_st, cmd.stride_height);
        auto [pos, vel, acc] = g.swing.step(g.phase, g.phi, ref(5), ref(8), ref.segment<3>(9),
                                             body.pos, body.lin_vel, body.ang_vel, yaw, g.feet);
        g.landing = g.swing.get_p_finish();

        WbcDesiredCommand wbc;
        auto& target = wbc.body_cmd;
        target.orientation = ref.head<3>();
        target.pos = ref.segment<3>(3);
        target.ang_vel = ref.segment<3>(6);
        target.lin_vel = ref.segment<3>(9);
        const Eigen::Quaterniond q = Eigen::AngleAxisd(ref(2), Eigen::Vector3d::UnitZ()) *
                                     Eigen::AngleAxisd(ref(1), Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(ref(0), Eigen::Vector3d::UnitX());
        target.orientation_quaternion = q.coeffs();
        for (int i = 0; i < NUM_LEGS; ++i) {
            wbc.phase_signal[i] = g.phase[i];
            wbc.active_legs[i] = cmd.active_legs[i];
            auto& foot = wbc.leg_cmd;
            if (grf && cmd.active_legs[i]) foot.*forces[i] = grf->segment<3>(3*i);
            if (support(g.phase[i])) {
                foot.*positions[i] = g.stance_feet[i];
            } else {
                foot.*positions[i] = pos[i];
                foot.*velocities[i] = vel[i];
                foot.*accelerations[i] = acc[i];
            }
        }
        {
            std::lock_guard lock(c.mutex);
            if (generation != c.generation || !outputs_enabled()) continue;
            if (grf) c.desired = wbc;
            c.stopped = c.active.stop && input.stop && std::ranges::all_of(g.phase, support);
        }
        if (cfg.wbic.debug_mode) {
            c.exchange.sendMpcCmd(target, cmd.active_legs);
            c.exchange.sendWbcCmd(target, wbc.leg_cmd);
            c.exchange.sendPhaseSig(g.phase, g.phi, g.t);
        }
        g.t += cfg.dt;
    }
}

} // namespace mors::fsm
