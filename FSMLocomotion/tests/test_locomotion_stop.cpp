#include "RobotFSM.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

using namespace mors::fsm;
using namespace std::chrono_literals;

namespace {

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

class TestController : public LocomotionControlBase {
public:
    void set_locomotion_mode(const std::uint8_t& value) override { mode = value; }

    void set_robot_cmd(const RobotData& value) override {
        command = value;
        ++command_count;
        LocomotionControlBase::set_robot_cmd(value);
    }

    void hold_measured_pose() override {
        ++hold_count;
        LocomotionControlBase::set_robot_cmd(RobotData{});
    }

    void request_gait_stop() override {
        ++stop_count;
        command.lin_vel.setZero();
        command.ang_vel.setZero();
        LocomotionControlBase::request_gait_stop();
    }

    void set_gait_params(const double&, const double&, const std::vector<double>&,
                         const double&) override { ++gait_count; }

    RobotData command;
    std::uint8_t mode = standing_mode;
    int command_count = 0;
    int hold_count = 0;
    int stop_count = 0;
    int gait_count = 0;
};

GaitStopConfig short_hold() {
    GaitStopConfig config;
    config.hold_s = 0.025;
    return config;
}

struct Fixture {
    explicit Fixture(GaitStopConfig config = short_hold(), LocomotionAccelerationConfig acceleration = {})
        : safety({{"joint", JointLimits{-3.0, 3.0}}}, 0.05),
          fsm(safety, &status, &controller, nullptr, config, {}, {}, acceleration) {
        status.contacts.fill(true);
        status.safety.contact_sensors_ready = true;
        setpoints.robot_cmd_received_at_s = monotonic_seconds();
        setpoints.robot_cmd.pos << 0.01, -0.02, 0.23;
        setpoints.robot_cmd.orientation << 0.02, -0.01, 0.1;
        controller.enable_outputs();
        // Вход в тестируемое состояние через обычное ребро FSM.
        fsm.state_id = StateID::STANDING;
        require(fsm.request_state(StateID::LOCOMOTION), "Cannot request locomotion");
        last_tick_before_s = monotonic_seconds();
        require(fsm.handle_event(Event::START_LOCOMOTION), "Cannot enter locomotion");
    }

    void tick(bool fresh = true) {
        if (fresh) {
            received_s += 0.01;
            status.robot_state_received_at_s = received_s;
        }
        last_tick_before_s = monotonic_seconds();
        fsm.update(setpoints);
        last_tick_after_s = monotonic_seconds();
    }

    void samples(int count) {
        for (int i = 0; i < count; ++i) tick();
    }

    void stop() {
        require(fsm.request_state(StateID::STANDING), "Cannot request standing");
    }

    bool command_reached() const {
        return controller.command.lin_vel.head<2>().isApprox(setpoints.robot_cmd.lin_vel.head<2>(), 1e-10)
            && std::abs(controller.command.ang_vel.z() - setpoints.robot_cmd.ang_vel.z()) < 1e-10;
    }

    void reach_command() {
        const double deadline = monotonic_seconds() + 3.0;
        while (!command_reached()) {
            require(monotonic_seconds() < deadline, "Command did not reach the requested velocity");
            std::this_thread::sleep_for(5ms);
            tick();
        }
    }

    RobotStatus status;
    TestController controller;
    SafetyMonitor safety;
    RobotFSM fsm;
    LocomotionSetpoints setpoints;
    double received_s = 0.0;
    double last_tick_before_s = 0.0;
    double last_tick_after_s = 0.0;
};

void check_acceleration_step(Fixture& test, LocomotionAccelerationConfig limits) {
    const auto previous = test.controller.command;
    const double previous_before = test.last_tick_before_s;
    test.tick();
    const double elapsed_upper = test.last_tick_after_s - previous_before;
    const auto& command = test.controller.command;
    require((command.lin_vel.head<2>() - previous.lin_vel.head<2>()).norm()
                <= limits.linear_mps2 * elapsed_upper + 1e-9,
            "Planar command change exceeded the acceleration limit");
    require(std::abs(command.ang_vel.z() - previous.ang_vel.z())
                <= limits.yaw_radps2 * elapsed_upper + 1e-9,
            "Yaw command change exceeded the acceleration limit");
    const Eigen::Vector2d to_target = test.setpoints.robot_cmd.lin_vel.head<2>() - previous.lin_vel.head<2>();
    const Eigen::Vector2d remaining = test.setpoints.robot_cmd.lin_vel.head<2>() - command.lin_vel.head<2>();
    require(remaining.norm() <= to_target.norm() + 1e-12 && remaining.dot(to_target) >= -1e-12,
            "Planar limiter moved away from or overshot the requested velocity");
    const double old_yaw_error = test.setpoints.robot_cmd.ang_vel.z() - previous.ang_vel.z();
    const double yaw_error = test.setpoints.robot_cmd.ang_vel.z() - command.ang_vel.z();
    require(std::abs(yaw_error) <= std::abs(old_yaw_error) + 1e-12 && yaw_error * old_yaw_error >= -1e-12,
            "Yaw limiter moved away from or overshot the requested velocity");
    require(command.pos.isApprox(test.setpoints.robot_cmd.pos)
            && command.orientation.isApprox(test.setpoints.robot_cmd.orientation),
            "Acceleration limiting changed the body pose command");
}

void acceleration_limits(LocomotionAccelerationConfig limits = {}) {
    Fixture test(short_hold(), limits);
    test.setpoints.robot_cmd.lin_vel << 0.03, 0.04, 0.0;
    test.setpoints.robot_cmd.ang_vel.z() = -0.08;
    check_acceleration_step(test, limits);
    require(!test.command_reached(), "First walking command jumped to the target velocity");
    // Старт, поворот вектора при той же норме и остановка обычной командой.
    for (const auto& target : {Eigen::Vector3d{0.03, 0.04, -0.08}, Eigen::Vector3d{-0.04, 0.03, 0.08},
                               Eigen::Vector3d{0.0, 0.0, 0.0}}) {
        test.setpoints.robot_cmd.lin_vel.head<2>() = target.head<2>();
        test.setpoints.robot_cmd.ang_vel.z() = target.z();
        const double deadline = monotonic_seconds() + 1.0;
        while (!test.command_reached()) {
            require(monotonic_seconds() < deadline, "Acceleration limiter did not reach the target");
            std::this_thread::sleep_for(5ms);
            check_acceleration_step(test, limits);
        }
    }
}

void braking_limits(double vx, double vy, double yaw, GaitStopConfig config = short_hold()) {
    Fixture test(config);
    test.setpoints.robot_cmd.lin_vel << vx, vy, 0.0;
    test.setpoints.robot_cmd.ang_vel.z() = yaw;
    test.status.vx = 0.2;
    test.samples(50);
    test.reach_command();
    double previous_before = test.last_tick_before_s;
    const RobotData original = test.controller.command;
    test.stop();
    // Новая команда во время остановки не должна менять позу и скорость торможения.
    test.setpoints.robot_cmd.pos.z() = 0.8;
    test.setpoints.robot_cmd.lin_vel << 2.0, -1.0, 0.0;
    test.setpoints.robot_cmd.ang_vel.z() = 2.0;
    const int gait_count = test.controller.gait_count;
    const double deadline = monotonic_seconds() + 1.0;
    while (!test.controller.command.lin_vel.isZero(0.0)
           || test.controller.command.ang_vel.z() != 0.0) {
        require(monotonic_seconds() < deadline, "Braking did not reach zero");
        const auto previous = test.controller.command;
        std::this_thread::sleep_for(10ms);
        const double before = monotonic_seconds();
        test.tick();
        const double elapsed_upper = monotonic_seconds() - previous_before;
        const auto& command = test.controller.command;
        const double speed = command.lin_vel.head<2>().norm();
        const double old_speed = previous.lin_vel.head<2>().norm();
        require(speed <= old_speed + 1e-12, "Linear speed increased during braking");
        require(old_speed - speed <= config.linear_decel_mps2 * elapsed_upper + 1e-9,
                "Linear deceleration exceeded limit");
        const double old_yaw = std::abs(previous.ang_vel.z());
        const double new_yaw = std::abs(command.ang_vel.z());
        require(new_yaw <= old_yaw + 1e-12, "Yaw speed increased during braking");
        require(old_yaw - new_yaw <= config.yaw_decel_radps2 * elapsed_upper + 1e-9,
                "Yaw deceleration exceeded limit");
        require(command.ang_vel.z() * yaw >= 0.0, "Yaw direction reversed");
        require(command.lin_vel.head<2>().dot(original.lin_vel.head<2>()) >= 0.0,
                "Linear direction reversed");
        require(std::abs(command.lin_vel.x() * vy - command.lin_vel.y() * vx) < 1e-12,
                "Planar braking changed direction");
        require(command.pos.isApprox(original.pos) && command.orientation.isApprox(original.orientation),
                "Stopping changed body pose command");
        require(test.controller.stop_count == 0, "Gait stopped while measured speed was high");
        previous_before = before;
    }
    require(test.controller.gait_count == gait_count, "Stopping changed gait parameters");
    test.status.vx = 0.0;
    test.samples(50);
    require(test.controller.stop_count > 0, "Gait did not stop after deceleration");
}

void finish_command_before_gait_stop() {
    Fixture test;
    test.setpoints.robot_cmd.lin_vel.y() = -0.02;
    test.setpoints.robot_cmd.ang_vel.z() = 0.02;
    test.samples(50);
    test.reach_command();
    test.stop();
    test.tick();
    require(test.controller.stop_count == 0, "Gait stopped before velocity commands reached zero");
    std::this_thread::sleep_for(50ms);
    test.tick();
    require(test.controller.stop_count == 1, "Zero velocity commands did not allow gait stop");
}

void window_and_duplicates() {
    Fixture test;
    test.samples(49);
    test.stop();
    for (int i = 0; i < 100; ++i) test.tick(false);
    require(test.controller.stop_count == 0, "Repeated snapshot filled the speed window");
    test.tick();
    require(test.controller.stop_count == 1, "Full speed window did not allow stopping");
}

void average_speed_magnitudes() {
    Fixture test;
    for (int i = 0; i < 50; ++i) {
        test.status.vx = (i % 2 == 0) ? 0.2 : -0.2;
        test.tick();
    }
    test.stop();
    test.status.vx = 0.0;
    test.samples(25);
    require(test.controller.stop_count == 0, "Opposite velocities canceled in the speed window");
    test.samples(25);
    require(test.controller.stop_count > 0, "Old speeds did not leave the circular window");
}

void invalid_measurements() {
    for (auto component : {&RobotStatus::vx, &RobotStatus::vy, &RobotStatus::yaw_rate}) {
        Fixture test;
        test.samples(50);
        test.stop();
        test.tick();
        const int stopped = test.controller.stop_count;
        test.status.*component = std::numeric_limits<double>::quiet_NaN();
        test.tick();
        require(!test.controller.gait_stopped(), "Invalid measurement did not resume stepping");
        test.status.*component = 0.0;
        test.samples(49);
        require(test.controller.stop_count == stopped, "Invalid measurement did not clear the window");
        test.tick();
        require(test.controller.stop_count == stopped + 1, "Window did not refill after invalid data");
    }
    Fixture test;
    test.samples(50);
    test.stop();
    test.status.robot_state_received_at_s.reset();
    test.tick(false);
    test.samples(49);
    require(test.controller.stop_count == 0, "Missing snapshot did not clear the window");
    test.tick();
    require(test.controller.stop_count == 1, "Window did not refill after missing snapshot");

    test.status.robot_state_received_at_s = std::numeric_limits<double>::infinity();
    test.tick(false);
    test.samples(49);
    require(test.controller.stop_count == 1, "Invalid timestamp did not clear the window");
    test.tick();
    require(test.controller.stop_count == 2, "Window did not refill after invalid timestamp");
}

void stopping_without_robot_command() {
    Fixture test;
    test.setpoints.robot_cmd_received_at_s.reset();
    test.samples(50);
    test.stop();
    test.tick();
    require(test.controller.gait_stopped(), "Holding pose did not allow gait stop");
    const int holds = test.controller.hold_count;
    test.status.vx = 0.2;
    test.tick();
    require(test.controller.hold_count == holds + 1 && !test.controller.gait_stopped(),
            "Holding pose did not resume stepping after a speed increase");
}

void resume_and_hold() {
    Fixture test;
    test.samples(50);
    test.stop();
    test.tick();
    require(test.controller.gait_stopped(), "Gait stop was not requested");
    std::this_thread::sleep_for(15ms);
    test.status.vx = 0.05;
    test.tick();
    require(!test.controller.gait_stopped(), "Speed at threshold did not resume stepping");
    require(test.controller.command.lin_vel.isZero(0.0), "Resumed gait received nonzero velocity");
    test.status.vx = 0.0;
    test.status.yaw_rate = -0.1;
    test.tick();
    require(!test.controller.gait_stopped(), "Yaw at threshold allowed gait stop");
    test.status.yaw_rate = 0.0;
    test.status.contacts[2] = false;
    test.tick();
    std::this_thread::sleep_for(35ms);
    test.tick();
    require(test.fsm.state_id == StateID::LOCOMOTION, "Missing contact allowed standing");
    test.status.contacts[2] = true;
    test.status.safety.contact_sensors_ready = false;
    test.tick();
    require(test.fsm.state_id == StateID::LOCOMOTION, "Unready contacts allowed standing");
    test.status.safety.contact_sensors_ready = true;
    test.tick();
    require(test.fsm.state_id == StateID::LOCOMOTION, "Standing skipped the stability hold");
    std::this_thread::sleep_for(35ms);
    test.tick();
    require(test.fsm.state_id == StateID::STANDING, "Stable stop did not enter standing");
    require(test.controller.mode == LocomotionControlBase::standing_mode, "Standing mode was not applied");

    require(test.fsm.request_state(StateID::LOCOMOTION), "Cannot restart walking");
    test.tick();
    test.stop();
    const int stopped = test.controller.stop_count;
    test.samples(49);
    require(test.controller.stop_count == stopped, "Reentry reused the old speed window");
    test.tick();
    require(test.controller.stop_count == stopped + 1, "Reentry window did not fill");
}

void cancellation_and_recovery() {
    Fixture braking;
    braking.setpoints.robot_cmd.lin_vel.x() = 0.1;
    braking.setpoints.robot_cmd.ang_vel.z() = 0.2;
    braking.samples(50);
    braking.reach_command();
    braking.stop();
    braking.tick();
    require(braking.controller.stop_count == 0, "Braking skipped straight to gait stop");
    require(braking.fsm.request_state(StateID::LOCOMOTION), "Cannot cancel braking");
    braking.setpoints.robot_cmd.lin_vel << 0.0, -0.2, 0.0;
    braking.setpoints.robot_cmd.ang_vel.z() = -0.3;
    check_acceleration_step(braking, {});
    require(!braking.command_reached(), "Cancel jumped directly to the new velocity");
    braking.reach_command();

    Fixture test;
    test.samples(50);
    test.stop();
    test.tick();
    std::this_thread::sleep_for(35ms);
    require(test.fsm.request_state(StateID::LOCOMOTION), "Cannot cancel stopping");
    test.setpoints.robot_cmd.lin_vel.y() = 0.2;
    test.setpoints.robot_cmd.ang_vel.z() = -0.3;
    check_acceleration_step(test, {});
    require(!test.controller.gait_stopped(), "Cancel did not resume gait");
    require(test.controller.command.lin_vel.y() > 0.0 && test.controller.command.ang_vel.z() < 0.0,
            "Cancel did not start approaching the current command");
    test.setpoints.robot_cmd.lin_vel.setZero();
    test.setpoints.robot_cmd.ang_vel.setZero();
    test.tick();
    test.reach_command();
    const int stopped = test.controller.stop_count;
    test.stop();
    test.tick();
    require(test.controller.stop_count == stopped + 1, "Cancellation cleared the speed history");
    require(test.fsm.state_id == StateID::LOCOMOTION, "Cancellation did not reset stability hold");

    Fixture recovery;
    recovery.setpoints.robot_cmd.lin_vel.x() = 0.03;
    recovery.setpoints.robot_cmd.ang_vel.z() = 0.08;
    recovery.reach_command();
    recovery.setpoints.mode = LocomotionMode::RECOVERY;
    recovery.tick();
    require(recovery.controller.command.lin_vel.isZero(0.0) && recovery.controller.command.ang_vel.isZero(0.0),
            "Entering recovery did not immediately zero velocity commands");
    recovery.setpoints.mode = LocomotionMode::WALKING;
    check_acceleration_step(recovery, {});
    require(!recovery.command_reached(), "Exiting recovery jumped to the walking velocity");
    recovery.setpoints.mode = LocomotionMode::RECOVERY;
    recovery.samples(50);
    recovery.stop();
    recovery.tick();
    require(recovery.controller.mode == LocomotionControlBase::recovery_mode, "Recovery mode was lost");
    require(recovery.controller.command.lin_vel.isZero(0.0) && recovery.controller.command.ang_vel.isZero(0.0),
            "Recovery received nonzero velocities");
}

void faults_interrupt_braking() {
    for (bool estop : {false, true}) {
        Fixture test;
        test.setpoints.robot_cmd.lin_vel.x() = 1.0;
        test.status.vx = 1.0;
        test.tick();
        test.stop();
        test.tick();
        const int commands = test.controller.command_count;
        if (estop) {
            test.fsm.handle_event(Event::ESTOP);
        } else {
            test.status.safety.telemetry_fault = SafetyFault{"TEST_STALE", "Test telemetry fault", monotonic_seconds()};
        }
        test.tick();
        require(test.fsm.state_id == StateID::FAILURE && !test.controller.outputs_enabled(),
                "Fault waited for braking to finish");
        require(test.controller.command_count == commands, "Fault emitted another walking command");
    }
}

void config_validation() {
    for (auto field : {&GaitStopConfig::linear_decel_mps2, &GaitStopConfig::yaw_decel_radps2}) {
        for (double value : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::quiet_NaN()}) {
            auto config = short_hold();
            config.*field = value;
            bool rejected = false;
            try {
                Fixture test(config);
            } catch (const std::invalid_argument&) {
                rejected = true;
            }
            require(rejected, "Invalid deceleration was accepted");
        }
    }
    for (auto field : {&LocomotionAccelerationConfig::linear_mps2, &LocomotionAccelerationConfig::yaw_radps2}) {
        for (double value : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::quiet_NaN()}) {
            LocomotionAccelerationConfig config;
            config.*field = value;
            bool rejected = false;
            try {
                Fixture test(short_hold(), config);
            } catch (const std::invalid_argument&) {
                rejected = true;
            }
            require(rejected, "Invalid acceleration limit was accepted");
        }
    }
}

} // namespace

int main() {
    try {
        acceleration_limits();
        acceleration_limits({0.8, 1.6});
        braking_limits(-0.03, 0.04, -0.08);
        braking_limits(0.0, -0.05, 0.08);
        braking_limits(0.0, 0.0, -0.08);
        auto config = short_hold();
        config.linear_decel_mps2 = 1.5;
        config.yaw_decel_radps2 = 2.0;
        braking_limits(0.03, -0.04, 0.08, config);
        finish_command_before_gait_stop();
        window_and_duplicates();
        average_speed_magnitudes();
        invalid_measurements();
        stopping_without_robot_command();
        resume_and_hold();
        cancellation_and_recovery();
        faults_interrupt_braking();
        config_validation();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
