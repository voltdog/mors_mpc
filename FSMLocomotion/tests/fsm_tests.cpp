#include "LocomotionTransitionMonitor.hpp"
#include "RobotFSM.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <type_traits>
#include <vector>

namespace {

using namespace mors::fsm;
using namespace std::chrono_literals;

static_assert(!std::is_copy_constructible_v<RobotFSM>);
static_assert(!std::is_move_constructible_v<RobotFSM>);
static_assert(!std::is_copy_assignable_v<RobotFSM>);
static_assert(!std::is_move_assignable_v<RobotFSM>);

void require(bool condition, std::string_view expression) {
    if (!condition) {
        throw std::runtime_error(std::string(expression));
    }
}

#define CHECK(...) require(static_cast<bool>((__VA_ARGS__)), #__VA_ARGS__)

template<class Function>
void expect_invalid_argument(Function function) {
    try {
        function();
    } catch (const std::invalid_argument&) {
        return;
    }
    throw std::runtime_error("Expected std::invalid_argument");
}

class RecordingControl : public LocomotionControlBase {
public:
    void set_locomotion_mode(const u_int8_t& value) override {
        mode = value;
        ++mode_calls;
        LocomotionControlBase::set_locomotion_mode(value);
    }

    void set_robot_cmd(const RobotData& command) override {
        last_cmd = command;
        mode_at_last_cmd = mode;
        ++command_calls;
        LocomotionControlBase::set_robot_cmd(command);
    }

    void set_gait_params(const double& stance, const double& swing,
                         const std::vector<double>& offsets, const double& height) override {
        gait_times = {stance, swing, height};
        gait_offsets = offsets;
        ++gait_calls;
    }

    void set_active_legs(const std::vector<bool>& value) override {
        active_legs = value;
    }

    void set_adaptation_type(const u_int8_t& value) override {
        adaptation_type = value;
    }

    void request_gait_stop() override {
        ++stop_calls;
        LocomotionControlBase::request_gait_stop();
    }

    bool gait_stopped() const override {
        return confirm_stop && LocomotionControlBase::gait_stopped();
    }

    void disable_outputs() override {
        if (fsm) {
            disabled_in.push_back(fsm->state_id);
        }
        LocomotionControlBase::disable_outputs();
    }

    RobotFSM* fsm = nullptr;
    int command_calls = 0;
    int mode_calls = 0;
    int gait_calls = 0;
    int stop_calls = 0;
    bool confirm_stop = true;
    u_int8_t mode = 0;
    u_int8_t mode_at_last_cmd = 0;
    u_int8_t adaptation_type = 0;
    std::array<double, 3> gait_times{};
    std::vector<double> gait_offsets;
    std::vector<bool> active_legs;
    std::vector<StateID> disabled_in;
    RobotData last_cmd;
};

struct Fixture {
    SafetyMonitor monitor{{{"hip", JointLimits(-1.0, 1.0)}, {"knee", JointLimits(-2.0, 2.0)}}, 0.05};
    ReadinessChecker readiness;
    RobotStatus status;
    RecordingControl controller;
    RobotFSM fsm{monitor, &status, &controller, &readiness};
    std::optional<RobotStateSnapshot> observation{std::in_place};
    LocomotionSetpoints setpoints;

    explicit Fixture(double stable_time_s = 0.0) : readiness(stable_time_s) {
        controller.fsm = &fsm;
        observation->readiness.fill(true);
        observation->readiness[18] = false;
        status.contacts.fill(true);
    }

    void diagnose(double now = 1.0, double age = 0.0) {
        if (observation) {
            observation->received_at_s = now - age;
        }
        status.safety = monitor.evaluate(observation, now);
        status.ready_to_move = readiness.is_ready(status, now);
    }

    void tick(double now = 1.0, double age = 0.0) {
        diagnose(now, age);
        fsm.update(setpoints);
    }

    void standing(double now = 1.0) {
        CHECK(fsm.request_state(StateID::STANDING));
        tick(now);
        CHECK(fsm.state_id == StateID::STANDUP);
        tick(now);
        tick(now);
        CHECK(fsm.state_id == StateID::STANDUP);
        tick(now);
        CHECK(fsm.state_id == StateID::STANDING);
    }

    void locomotion() {
        standing();
        CHECK(fsm.request_state(StateID::LOCOMOTION));
        tick();
        CHECK(fsm.state_id == StateID::LOCOMOTION);
    }
};

void configuration() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    expect_invalid_argument([] { JointLimits limits(0.0, 0.0); });
    expect_invalid_argument([] { JointLimits limits(1.0, -1.0); });
    expect_invalid_argument([&] { JointLimits limits(nan, 1.0); });
    expect_invalid_argument([&] { JointLimits limits(-1.0, inf); });
    expect_invalid_argument([] { SafetyMonitor monitor({}, 0.05); });
    for (double value : {0.0, -1.0, nan, inf}) {
        expect_invalid_argument([&] { SafetyMonitor monitor({{"joint", JointLimits(-1.0, 1.0)}}, value); });
    }
    for (double value : {-1.0, nan, inf}) {
        expect_invalid_argument([&] { ReadinessChecker readiness(value); });
    }
    for (double value : {0.0, -1.0, nan, inf}) {
        expect_invalid_argument([&] { LocomotionTransitionMonitor monitor({.command_timeout_s = value}); });
    }
    expect_invalid_argument([] { LocomotionTransitionMonitor monitor({.stop_linear_mps = 0.05}); });
    expect_invalid_argument([] { LocomotionTransitionMonitor monitor({.stop_yaw_radps = 0.11}); });
    expect_invalid_argument([] { LocomotionTransitionMonitor monitor({.recovery_exit_walking = {0.10, 0.12}}); });
    expect_invalid_argument([] { LocomotionTransitionMonitor monitor({.recovery_exit_standing = {0.04, 0.21}}); });
    Fixture fixture;
    expect_invalid_argument([&] { RobotFSM fsm(fixture.monitor, nullptr, nullptr, nullptr, {.hold_s = inf}); });
    RobotFSM defaults(fixture.monitor);
    CHECK(defaults.state_id == StateID::IDLE);
    CHECK(!defaults.controller.outputs_enabled());
    defaults.update({});
    CHECK(!fixture.monitor.fault());
}

void automatic_walking_and_stop() {
    using enum LocomotionMode;
    LocomotionTransitionMonitor monitor({.stop_hold_s = 0.25});
    RobotData estimate, command;
    monitor.on_state_entered(StateID::STANDING, &estimate);
    const auto standing = [&](bool allowed) {
        return monitor.recommend(StateID::STANDING, allowed, false, &estimate, command, 1.0);
    };
    command.lin_vel.x() = 0.05;
    CHECK(standing(true) == STANDING);
    for (const Eigen::Vector3d& velocity : {Eigen::Vector3d(-0.06, 0.0, 0.0),
                                           Eigen::Vector3d(0.0, -0.06, 0.0),
                                           Eigen::Vector3d(0.04, -0.04, 0.0)}) {
        command.lin_vel = velocity;
        CHECK(standing(true) == WALKING);
        CHECK(standing(false) == STANDING);
    }
    command.lin_vel.setZero();
    command.ang_vel.z() = -0.11;
    CHECK(standing(true) == WALKING);

    monitor.on_state_entered(StateID::LOCOMOTION, &estimate);
    const auto walking = [&](double now_s, bool allowed = true) {
        return monitor.recommend(StateID::LOCOMOTION, allowed, false, &estimate, command, now_s);
    };
    command.ang_vel.setZero();
    CHECK(walking(2.0) == WALKING);
    command.ang_vel.z() = 0.05;
    CHECK(walking(2.125) == WALKING); // Равенство порогу сбрасывает выдержку.
    command.ang_vel.setZero();
    CHECK(walking(2.25) == WALKING);
    CHECK(walking(2.375) == WALKING);
    CHECK(walking(2.50) == STANDING);
    command.lin_vel.x() = 0.04;
    CHECK(walking(2.625) == STANDING);
    command.lin_vel.x() = -0.08;
    CHECK(walking(2.75) == WALKING);
    CHECK(walking(2.875, false) == STANDING);
    CHECK(walking(3.0, false) == STANDING);
    command.lin_vel.setZero();
    CHECK(walking(3.125) == STANDING);
}

void command_freshness() {
    LocomotionTransitionMonitor monitor;
    LocomotionSetpoints setpoints;
    setpoints.robot_cmd.lin_vel.x() = 0.2;
    CHECK(!monitor.velocity_is_fresh(setpoints, 2.0));
    setpoints.robot_cmd_received_at_s = 2.0;
    CHECK(!monitor.velocity_is_fresh(setpoints, 1.99));
    CHECK(monitor.velocity_is_fresh(setpoints, 2.0));
    CHECK(monitor.velocity_is_fresh(setpoints, 2.50));
    CHECK(!monitor.velocity_is_fresh(setpoints, 2.51));
    setpoints.robot_cmd.ang_vel.y() = std::numeric_limits<double>::quiet_NaN();
    CHECK(!monitor.velocity_is_fresh(setpoints, 2.1));
}

void recovery_thresholds() {
    using enum LocomotionMode;
    LocomotionTransitionMonitor monitor;
    RobotData estimate, command;
    const Eigen::Vector3d origin(3.0, -2.0, 0.25);
    estimate.pos = origin;
    const auto standing = [&](bool allowed = false) {
        return monitor.recommend(StateID::STANDING, allowed, false, &estimate, command, 1.0);
    };
    const auto recovery = [&](bool allowed = true, bool valid = true) {
        return monitor.recommend(StateID::LOCOMOTION, allowed, true, valid ? &estimate : nullptr, command, 2.0);
    };
    monitor.on_state_entered(StateID::STANDING, nullptr);
    command.pos << -4.0, 5.0, 0.28;
    CHECK(standing() == STANDING); // Первый валидный отсчёт, независимо от cmd_pose.
    estimate.pos = origin + Eigen::Vector3d(0.08, -0.08, 0.0);
    estimate.lin_vel.x() = 0.20;
    CHECK(standing() == STANDING);
    estimate.pos = origin;
    estimate.lin_vel << -0.16, 0.16, 0.0;
    CHECK(standing() == STANDING);
    estimate.pos = origin + Eigen::Vector3d(0.08, -0.08, 0.0);
    CHECK(standing() == RECOVERY);
    command.ang_vel.z() = 0.3;
    CHECK(standing(true) == RECOVERY); // Приоритет над автостартом.

    monitor.on_state_entered(StateID::LOCOMOTION, &estimate);
    estimate.pos = origin + Eigen::Vector3d(0.09, 0.0, 0.0);
    estimate.lin_vel.x() = estimate.lin_vel.y() = 0.02;
    CHECK(recovery() == RECOVERY);
    estimate.pos.x() = origin.x() + 0.055;
    estimate.lin_vel.x() = 0.13;
    CHECK(recovery() == RECOVERY);
    estimate.lin_vel.x() = 0.095;
    CHECK(recovery() == WALKING);
    CHECK(recovery(false) == RECOVERY);
    command.ang_vel.setZero();
    CHECK(recovery() == RECOVERY);
    estimate.pos.x() = origin.x() + 0.02;
    CHECK(recovery() == RECOVERY);
    estimate.lin_vel.x() = 0.025;
    CHECK(recovery(true, false) == RECOVERY);
    CHECK(recovery() == STANDING);

    monitor.on_state_entered(StateID::STANDING, &estimate);
    estimate.pos.x() += 0.09; // Отсчёт от новой стойки, а не от исходной точки.
    estimate.lin_vel.x() = 0.25;
    CHECK(standing() == STANDING);
}

void safety_diagnostics() {
    Fixture fixture;
    const auto& monitor = fixture.monitor;
    auto& observation = fixture.observation;
    constexpr double now = 1.0;
    CHECK(monitor.evaluate(std::nullopt, now).telemetry_fault->code == "ROBOT_STATE_MISSING");
    for (double stamp : {0.949, 1.001, std::numeric_limits<double>::quiet_NaN()}) {
        observation->received_at_s = stamp;
        CHECK(monitor.evaluate(observation, now).telemetry_fault->code == "ROBOT_STATE_STALE");
    }
    observation->received_at_s = 0.951;
    CHECK(!monitor.evaluate(observation, now).telemetry_fault);
    for (std::size_t bit = 0; bit < 18; ++bit) {
        observation->readiness[bit] = false;
        const auto diagnostics = monitor.evaluate(observation, now);
        CHECK(diagnostics.telemetry_fault->code == "ROBOT_STATE_NOT_READY");
        CHECK(diagnostics.telemetry_fault->description == "Required readiness bit " +
              std::to_string(bit) + " is false");
        CHECK(diagnostics.contact_sensors_ready == (bit < 14));
        observation->readiness[bit] = true;
    }
    CHECK(!observation->readiness[18]);
    CHECK(!monitor.evaluate(observation, now).telemetry_fault);

    observation->joints.position[0] = -1.0;
    observation->joints.position[1] = 2.0;
    CHECK(!monitor.evaluate(observation, now).joint_fault);
    for (double angle : {2.01, -2.01, std::numeric_limits<double>::quiet_NaN()}) {
        observation->joints.position[1] = angle;
        const auto diagnostics = monitor.evaluate(observation, now);
        CHECK(diagnostics.joint_fault->code == "JOINT_LIMIT");
        CHECK(diagnostics.joint_fault->description.starts_with("knee: "));
    }
    observation->joints.position[0] = 1.01; // При двух нарушениях важен порядок индексов.
    const auto fault = monitor.evaluate(observation, now).joint_fault;
    CHECK(fault->description == "hip: q=1.0100 rad outside [-1.0000, 1.0000] rad");
    CHECK(!monitor.fault());
    fixture.monitor.latch_fault(*fault);
    fixture.monitor.latch_fault({"ESTOP", "", now + 1.0});
    CHECK(monitor.fault()->timestamp_monotonic_s == now);
    CHECK(monitor.fault()->code == "JOINT_LIMIT");
}

void readiness_stability() {
    Fixture fixture(0.20);
    CHECK(!fixture.readiness.is_ready(RobotStatus{}, 0.0));
    fixture.status.contacts.fill(false); // Касание стоп не нужно для готовности датчиков.
    fixture.observation->joints.position[0] = 1.5;
    fixture.diagnose(1.0);
    CHECK(!fixture.status.ready_to_move);
    fixture.diagnose(1.19);
    CHECK(!fixture.status.ready_to_move);
    fixture.diagnose(1.21);
    CHECK(fixture.status.ready_to_move);

    fixture.observation->readiness[2] = false;
    fixture.diagnose(1.22);
    CHECK(!fixture.status.ready_to_move);
    fixture.observation->readiness[2] = true;
    fixture.diagnose(1.23);
    CHECK(!fixture.status.ready_to_move);
    fixture.diagnose(1.44);
    CHECK(fixture.status.ready_to_move);
    fixture.readiness.reset();
    fixture.diagnose(1.45);
    CHECK(!fixture.status.ready_to_move);
}

void routes_and_targets() {
    Fixture fixture;
    constexpr std::array states{
        StateID::IDLE, StateID::STANDUP, StateID::STANDING, StateID::LOCOMOTION,
        StateID::LAYDOWN, StateID::ACTION, StateID::EXTERNAL, StateID::FAILURE,
    };
    for (StateID start : states) {
        for (StateID goal : states) {
            const auto path = fixture.fsm.find_path(start, goal);
            if (start == StateID::FAILURE || goal == StateID::FAILURE) {
                CHECK(path.has_value() == (start == goal));
            } else {
                CHECK(path && path->front() == start && path->back() == goal);
            }
        }
    }
    CHECK(fixture.fsm.find_path(StateID::IDLE, StateID::EXTERNAL)
          == std::vector{StateID::IDLE, StateID::STANDUP, StateID::STANDING, StateID::EXTERNAL});
    CHECK(fixture.fsm.find_path(StateID::LOCOMOTION, StateID::IDLE)
          == std::vector{StateID::LOCOMOTION, StateID::STANDING, StateID::LAYDOWN, StateID::IDLE});
    CHECK(!fixture.fsm.target_state);
    for (StateID target : {StateID::STANDUP, StateID::LAYDOWN, StateID::FAILURE,
                           static_cast<StateID>(99)}) {
        expect_invalid_argument([&] { fixture.fsm.request_state(target); });
    }
    CHECK(!fixture.fsm.handle_event(Event::START_ACTION));
}

void transitions() {
    Fixture fixture;
    const auto observation = fixture.observation;
    fixture.observation.reset();
    CHECK(fixture.fsm.request_state(StateID::STANDING));
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::IDLE && !fixture.monitor.fault());
    CHECK(!fixture.controller.outputs_enabled());
    CHECK(!fixture.fsm.handle_event(Event::CLEAR_FAULT));

    fixture.observation = observation;
    fixture.observation->joints.position[0] = 1.5;
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::STANDUP);
    CHECK(fixture.controller.outputs_enabled());

    CHECK(fixture.fsm.request_state(StateID::IDLE));
    fixture.tick();
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::STANDUP && !fixture.monitor.fault());
    fixture.observation->joints.position[0] = 0.0;
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::STANDING);
    CHECK(fixture.controller.command_calls == 0);
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::LAYDOWN);
    fixture.tick();
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::LAYDOWN);
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::IDLE);
    CHECK(!fixture.controller.outputs_enabled());
    fixture.standing();

    for (StateID target : {StateID::ACTION, StateID::EXTERNAL}) {
        CHECK(fixture.fsm.request_state(target));
        fixture.tick();
        CHECK(fixture.fsm.state_id == target);
        CHECK(fixture.fsm.request_state(StateID::STANDING));
        fixture.tick();
        CHECK(fixture.fsm.state_id == StateID::STANDING);
    }
}

void emergency_stop_and_reset() {
    Fixture fixture;
    fixture.locomotion();
    fixture.tick();
    const auto commands = fixture.controller.command_calls;
    CHECK(fixture.fsm.handle_event(Event::ESTOP));
    CHECK(fixture.fsm.state_id == StateID::FAILURE);
    CHECK(fixture.controller.disabled_in == std::vector{StateID::LOCOMOTION});
    CHECK(!fixture.controller.outputs_enabled() && !fixture.fsm.target_state);
    CHECK(fixture.monitor.fault()->code == "ESTOP");
    CHECK(!fixture.fsm.request_state(StateID::STANDING));
    fixture.tick();
    CHECK(fixture.controller.command_calls == commands);

    Fixture guarded;
    guarded.standing();
    guarded.observation->joints.position[0] = 1.3;
    guarded.tick();
    CHECK(guarded.fsm.state_id == StateID::FAILURE);
    CHECK(guarded.monitor.fault()->code == "JOINT_LIMIT");
    CHECK(guarded.controller.command_calls == 0);
    CHECK(guarded.fsm.try_clear_fault()->starts_with("hip: "));
    CHECK(!guarded.controller.outputs_enabled());
    guarded.observation->joints.position[0] = 0.0;
    guarded.diagnose(2.0, 0.051);
    CHECK(guarded.fsm.try_clear_fault());
    CHECK(guarded.fsm.state_id == StateID::FAILURE);

    guarded.diagnose(2.05);
    CHECK(!guarded.fsm.try_clear_fault());
    CHECK(guarded.fsm.state_id == StateID::IDLE && !guarded.monitor.fault());
    CHECK(guarded.controller.outputs_enabled() && !guarded.fsm.target_state);
    guarded.tick(2.06);
    CHECK(guarded.fsm.state_id == StateID::IDLE);
    guarded.standing(2.07);

    Fixture stale;
    stale.standing();
    stale.tick(1.0, 0.051);
    CHECK(stale.fsm.state_id == StateID::FAILURE);
    CHECK(stale.monitor.fault()->code == "ROBOT_STATE_STALE");

    Fixture on_entry;
    CHECK(on_entry.fsm.request_state(StateID::STANDING));
    on_entry.observation->joints.position[0] = 1.3;
    on_entry.tick();
    on_entry.tick();
    on_entry.tick();
    CHECK(on_entry.fsm.state_id == StateID::STANDUP);
    on_entry.tick();
    CHECK(on_entry.fsm.state_id == StateID::FAILURE);
    CHECK(on_entry.monitor.fault()->code == "JOINT_LIMIT");
    CHECK(on_entry.controller.mode == 0); // Вход в STANDING не состоялся.
}

void clear_with_stable_readiness() {
    Fixture fixture(0.20);
    CHECK(fixture.fsm.handle_event(Event::ESTOP));
    fixture.diagnose(1.0);
    CHECK(fixture.fsm.try_clear_fault() == "Readiness has not been confirmed");
    CHECK(!fixture.controller.outputs_enabled());
    fixture.diagnose(1.21);
    CHECK(!fixture.fsm.try_clear_fault());
    CHECK(fixture.fsm.state_id == StateID::IDLE && fixture.controller.outputs_enabled());
    CHECK(!fixture.fsm.target_state);

    CHECK(fixture.fsm.request_state(StateID::STANDING));
    fixture.tick(1.22);
    CHECK(fixture.fsm.state_id == StateID::IDLE);
    fixture.tick(1.43);
    CHECK(fixture.fsm.state_id == StateID::STANDUP);
}

void locomotion_setpoints_and_stop() {
    Fixture fixture;
    auto& command = fixture.setpoints.robot_cmd;
    command.pos.z() = 0.25;
    command.lin_vel.x() = 0.2;
    command.ang_vel.z() = 0.5;
    fixture.setpoints.t_st = 0.5;
    fixture.setpoints.t_sw = 0.15;
    fixture.setpoints.stride_height = 0.04;
    fixture.setpoints.gait_type = {0.0, 0.5, 0.5, 0.0};
    fixture.setpoints.active_legs[1] = false;
    fixture.setpoints.adaptation_type = NOADAPT;

    fixture.locomotion();
    auto& controller = fixture.controller;
    CHECK(controller.mode == LocomotionControlBase::standing_mode && controller.mode_calls == 1);
    CHECK(controller.command_calls == 1); // Уставка STANDING на такте перехода.
    CHECK(controller.last_cmd.pos.z() == command.pos.z());
    CHECK(controller.last_cmd.lin_vel.isZero() && controller.last_cmd.ang_vel.isZero());

    fixture.tick();
    CHECK(controller.mode_at_last_cmd == LocomotionControlBase::walking_mode && controller.mode_calls == 2);
    CHECK(controller.last_cmd.lin_vel == command.lin_vel);
    CHECK(controller.last_cmd.ang_vel == command.ang_vel);
    CHECK(controller.gait_times == std::array{0.5, 0.15, 0.04});
    CHECK(controller.gait_offsets == fixture.setpoints.gait_type);
    CHECK(controller.active_legs == fixture.setpoints.active_legs);
    CHECK(controller.adaptation_type == NOADAPT);

    CHECK(fixture.fsm.request_state(StateID::IDLE));
    CHECK(!fixture.fsm.handle_event(Event::START_STANDING));
    int stop_calls = controller.stop_calls;
    int command_calls = controller.command_calls;
    int gait_calls = controller.gait_calls;
    const auto blocked = [&] {
        fixture.tick();
        CHECK(fixture.fsm.state_id == StateID::LOCOMOTION);
        CHECK(controller.stop_calls == ++stop_calls);
        CHECK(controller.command_calls == command_calls && controller.gait_calls == gait_calls);
    };
    fixture.status.vx = 0.05;
    blocked();
    fixture.status.vx = fixture.status.vy = 0.04; // Проверяется модуль скорости.
    blocked();
    fixture.status.vx = fixture.status.vy = 0.0;
    fixture.status.yaw_rate = -0.10;
    blocked();
    fixture.status.yaw_rate = std::numeric_limits<double>::quiet_NaN();
    blocked();
    fixture.status.yaw_rate = 0.0;
    fixture.status.contacts[0] = false;
    blocked();
    fixture.status.contacts.fill(true);
    controller.confirm_stop = false;
    blocked();
    CHECK(!fixture.status.gait_stopped);
    controller.confirm_stop = true;
    blocked();
    CHECK(fixture.status.gait_stopped);

    CHECK(fixture.fsm.request_state(StateID::LOCOMOTION));
    fixture.tick();
    CHECK(controller.stop_calls == stop_calls);
    CHECK(controller.command_calls == ++command_calls && controller.gait_calls == ++gait_calls);
    CHECK(!fixture.status.gait_stopped);
    CHECK(controller.last_cmd.lin_vel == command.lin_vel);

    CHECK(fixture.fsm.request_state(StateID::IDLE));
    blocked();
    std::this_thread::sleep_for(160ms);
    fixture.status.contacts[1] = false;
    blocked();
    fixture.status.contacts.fill(true);
    blocked();
    std::this_thread::sleep_for(160ms);
    blocked();
    std::this_thread::sleep_for(160ms);
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::STANDING);
    CHECK(controller.command_calls == command_calls);
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::LAYDOWN);
    CHECK(controller.last_cmd.lin_vel.isZero());
    fixture.tick();
    fixture.tick();
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::IDLE);
}

void recovery_controller_mode() {
    Fixture fixture;
    fixture.setpoints.mode = LocomotionMode::RECOVERY;
    auto& command = fixture.setpoints.robot_cmd;
    command.pos.z() = 0.28;
    command.lin_vel.x() = 0.2;
    command.ang_vel.z() = 0.3;

    fixture.locomotion();
    auto& controller = fixture.controller;
    CHECK(controller.mode == LocomotionControlBase::standing_mode);
    fixture.tick();
    CHECK(controller.mode_at_last_cmd == LocomotionControlBase::recovery_mode && controller.mode_calls == 2);
    CHECK(controller.last_cmd.lin_vel.isZero() && controller.last_cmd.ang_vel.isZero());
    CHECK(controller.last_cmd.pos.z() == command.pos.z());
    CHECK(controller.gait_calls == 0);
    fixture.tick();
    CHECK(controller.mode_calls == 2);

    CHECK(fixture.fsm.request_state(StateID::STANDING));
    controller.confirm_stop = false;
    fixture.tick();
    CHECK(controller.mode == LocomotionControlBase::recovery_mode && controller.mode_calls == 2);
    CHECK(controller.stop_calls == 1 && !fixture.status.gait_stopped);
    CHECK(fixture.fsm.state_id == StateID::LOCOMOTION);

    CHECK(fixture.fsm.request_state(StateID::LOCOMOTION));
    fixture.setpoints.mode = LocomotionMode::WALKING;
    fixture.tick();
    CHECK(controller.mode_at_last_cmd == LocomotionControlBase::walking_mode && controller.mode_calls == 3);
    CHECK(controller.last_cmd.lin_vel == command.lin_vel);
    CHECK(controller.last_cmd.ang_vel == command.ang_vel);
    CHECK(controller.gait_calls == 1 && controller.stop_calls == 1);
}

void controller_stub() {
    LocomotionControlBase controller;
    CHECK(!controller.input_mode && !controller.outputs_enabled());
    controller.start_thread();
    controller.stop_thread();
    controller.set_data_input_mode(2);
    controller.set_wbc_cmd(WbcDesiredCommand{});
    controller.set_data_input_mode(3);
    controller.set_joints_cmd(JointVector::Zero());
    controller.set_data_input_mode(LocomotionControlBase::robot_input);
    controller.enable_outputs();
    controller.set_robot_cmd(RobotData{});
    CHECK(controller.outputs_enabled() && !controller.gait_stopped());
    controller.request_gait_stop();
    CHECK(controller.gait_stopped());
    controller.disable_outputs();
    controller.disable_outputs();
    CHECK(!controller.outputs_enabled() && !controller.gait_stopped());
}

} // namespace

int main() {
    struct Test {
        std::string_view name;
        void (*run)();
    };
    const std::array tests{
        Test{"configuration", configuration},
        Test{"automatic_walking_and_stop", automatic_walking_and_stop},
        Test{"command_freshness", command_freshness},
        Test{"recovery_thresholds", recovery_thresholds},
        Test{"safety_diagnostics", safety_diagnostics},
        Test{"readiness_stability", readiness_stability},
        Test{"routes_and_targets", routes_and_targets},
        Test{"transitions", transitions},
        Test{"emergency_stop_and_reset", emergency_stop_and_reset},
        Test{"clear_with_stable_readiness", clear_with_stable_readiness},
        Test{"locomotion_setpoints_and_stop", locomotion_setpoints_and_stop},
        Test{"recovery_controller_mode", recovery_controller_mode},
        Test{"controller_stub", controller_stub},
    };
    for (const auto& [name, test] : tests) {
        try {
            test();
            std::cout << "PASS " << name << '\n';
        } catch (const std::exception& error) {
            std::cerr << "FAIL " << name << ": " << error.what() << '\n';
            return 1;
        }
    }
    return 0;
}
