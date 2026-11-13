#include "RobotFSM.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
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

class CaptureOutput {
public:
    CaptureOutput() : previous_(std::cout.rdbuf(stream.rdbuf())) {}
    ~CaptureOutput() { std::cout.rdbuf(previous_); }

    std::ostringstream stream;

private:
    std::streambuf* previous_;
};

class RecordingControl : public LocomotionControl {
public:
    void step() override {
        ++steps;
        LocomotionControl::step();
    }

    void set_velocity(double vx, double vy, double yaw_rate) override {
        ++velocity_calls;
        LocomotionControl::set_velocity(vx, vy, yaw_rate);
    }

    void disable_outputs() override {
        if (fsm) {
            stop_states.push_back(fsm->state_id);
        }
        LocomotionControl::disable_outputs();
    }

    RobotFSM* fsm = nullptr;
    int steps = 0;
    int velocity_calls = 0;
    std::vector<StateID> stop_states;
};

struct Fixture {
    SafetyMonitor monitor{{{"hip", JointLimits(-1.0, 1.0)}, {"knee", JointLimits(-1.0, 1.0)}}};
    ReadinessChecker readiness{monitor, 0.15, 0.0};
    RobotStatus status;
    RecordingControl controller;
    RobotFSM fsm{monitor, &status, &controller, &readiness};

    Fixture() {
        controller.fsm = &fsm;
        status.drives_ready = status.contact_sensors_ready = status.estimator_ready = true;
        status.contacts.fill(true);
        status.gait_stopped = true;
        feed();
    }

    void feed(double hip = 0.0, double knee = 0.0) {
        const double now = monotonic_seconds();
        status.joint_feedback = JointFeedback{{{"hip", hip}, {"knee", knee}}, now};
        status.drive_status_stamp_s = now;
        status.contact_stamp_s = now;
        status.estimator_stamp_s = now;
    }

    void tick() {
        feed();
        fsm.update();
    }

    void standing() {
        CHECK(fsm.request_state(StateID::STANDING));
        tick();
        CHECK(fsm.state_id == StateID::STANDUP);
        tick();
        tick();
        CHECK(fsm.state_id == StateID::STANDUP);
        tick();
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
    expect_invalid_argument([] { SafetyMonitor monitor({}); });
    SafetyMonitor monitor({{"joint", JointLimits(-1.0, 1.0)}});
    for (double value : {0.0, -1.0, nan, inf}) {
        expect_invalid_argument([&] { SafetyMonitor invalid(monitor.joint_limits, value); });
        expect_invalid_argument([&] { ReadinessChecker invalid(monitor, value); });
    }
    for (double value : {-1.0, nan, inf}) {
        expect_invalid_argument([&] { ReadinessChecker invalid(monitor, 0.15, value); });
    }
    ReadinessChecker immediate(monitor, 0.15, 0.0);
    RobotFSM defaults(monitor);
    CHECK(defaults.state_id == StateID::IDLE);
    CHECK(defaults.readiness_checker.stable_time_s == 0.20);
    defaults.update();
    CHECK(!monitor.fault());
    CHECK(!defaults.readiness_checker.last_errors.empty());
}

void safety_feedback() {
    // Имена намеренно заданы не в алфавитном порядке.
    SafetyMonitor monitor({{"z_joint", JointLimits(-1.0, 1.0)},
                           {"a_joint", JointLimits(-1.0, 1.0)}});
    CHECK(monitor.evaluate(std::nullopt)->code == "JOINT_FEEDBACK_MISSING");
    CHECK(!monitor.evaluate(std::nullopt, false));
    CHECK(!monitor.fault()); // evaluate() не фиксирует ошибку.

    std::optional<JointFeedback> feedback = JointFeedback{
        {{"z_joint", 0.0}, {"a_joint", 0.0}}, monotonic_seconds(),
    };
    CHECK(!monitor.evaluate(feedback));
    for (double stamp : {std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity(), monotonic_seconds() + 1.0}) {
        feedback->stamp_monotonic_s = stamp;
        CHECK(monitor.evaluate(feedback)->code == "JOINT_FEEDBACK_INVALID");
    }
    feedback->stamp_monotonic_s = monotonic_seconds() - 1.0;
    CHECK(monitor.evaluate(feedback)->code == "JOINT_FEEDBACK_STALE");
    feedback->stamp_monotonic_s = monotonic_seconds() + 0.0005;
    CHECK(!monitor.evaluate(feedback)); // Допуск будущей отметки: 1 мс.

    feedback->stamp_monotonic_s = monotonic_seconds();
    feedback->positions_rad.clear();
    CHECK(monitor.evaluate(feedback)->description == "Missing encoder position for z_joint");
    feedback->positions_rad = {{"z_joint", 0.0}, {"a_joint", 0.0}};
    for (double angle : {std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()}) {
        feedback->positions_rad["z_joint"] = angle;
        CHECK(monitor.evaluate(feedback)->code == "JOINT_INVALID");
        CHECK(monitor.evaluate(feedback, true, StateID::IDLE)->code == "JOINT_INVALID");
    }
    for (double angle : {-1.0, 1.0}) {
        feedback->positions_rad["z_joint"] = angle;
        CHECK(!monitor.evaluate(feedback)); // Границы диапазона включены.
    }
    feedback->positions_rad = {{"z_joint", 1.01}, {"a_joint", -1.01}};
    const auto fault = monitor.evaluate(feedback);
    CHECK(fault->code == "JOINT_LIMIT");
    CHECK(fault->description == "z_joint: q=1.0100 rad outside [-1.0000, 1.0000] rad");
    CHECK(!monitor.evaluate(feedback, true, StateID::IDLE));
    CHECK(!monitor.evaluate(feedback, true, StateID::STANDUP));
    for (StateID state : {StateID::STANDING, StateID::LOCOMOTION, StateID::LAYDOWN,
                          StateID::ACTION, StateID::EXTERNAL, StateID::FAILURE}) {
        CHECK(monitor.evaluate(feedback, true, state)->code == "JOINT_LIMIT");
    }
}

void safety_latching() {
    SafetyMonitor monitor({{"joint", JointLimits(-1.0, 1.0)}});
    CHECK(monitor.check(std::nullopt)->code == "JOINT_FEEDBACK_MISSING");
    const double fault_stamp = monitor.fault()->timestamp_monotonic_s;
    std::optional<JointFeedback> feedback = JointFeedback{{{"joint", 2.0}}, monotonic_seconds()};
    CHECK(monitor.check(feedback)->code == "JOINT_FEEDBACK_MISSING");
    CHECK(monitor.check(std::nullopt, false)->timestamp_monotonic_s == fault_stamp);
    CHECK(!monitor.is_encoder_feedback_valid(feedback));
    CHECK(!monitor.clear_fault(feedback, false, true));
    CHECK(!monitor.clear_fault(feedback, true, false));
    CHECK(!monitor.clear_fault(std::nullopt, true, true));
    CHECK(monitor.clear_fault(feedback, true, true)); // В IDLE угол не ограничивается.
    CHECK(!monitor.fault());
    CHECK(monitor.is_encoder_feedback_valid(feedback));
    CHECK(monitor.check(feedback)->code == "JOINT_LIMIT");
}

void readiness_diagnostics() {
    Fixture fixture;
    RobotStatus empty;
    CHECK(!fixture.readiness.is_ready(empty));
    CHECK(fixture.readiness.last_errors == std::vector<std::string>{
        "Приводы не готовы", "Нет актуального статуса приводов",
        "Нет корректных актуальных данных энкодеров", "Датчики контакта не готовы",
        "Нет актуальных данных датчиков контакта", "Оценщик состояния не готов",
        "Нет актуальных данных оценщика состояния",
    });
    CHECK(!fixture.monitor.fault());
    fixture.status.contacts.fill(false); // Готовность не требует касания всех стоп.
    CHECK(fixture.readiness.is_ready(fixture.status));
    for (double stamp : {monotonic_seconds() - 1.0, monotonic_seconds() + 1.0,
                         std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()}) {
        fixture.feed();
        fixture.status.drive_status_stamp_s = stamp;
        fixture.status.contact_stamp_s = stamp;
        fixture.status.estimator_stamp_s = stamp;
        CHECK(!fixture.readiness.is_ready(fixture.status));
        CHECK(fixture.readiness.last_errors.size() == 3);
    }
    for (int component = 0; component < 3; ++component) {
        fixture.status.vx = fixture.status.vy = fixture.status.yaw_rate = 0.0;
        std::array<double*, 3> velocities{
            &fixture.status.vx, &fixture.status.vy, &fixture.status.yaw_rate,
        };
        *velocities[component] = std::numeric_limits<double>::quiet_NaN();
        fixture.feed();
        CHECK(!fixture.readiness.is_ready(fixture.status));
        CHECK(fixture.readiness.last_errors.back() == "Некорректные скорости оценщика состояния");
    }
    fixture.status.yaw_rate = 0.0;
    fixture.feed(2.0);
    CHECK(fixture.readiness.is_ready(fixture.status));
    fixture.monitor.check(std::nullopt);
    CHECK(!fixture.readiness.is_ready(fixture.status));
}

void readiness_stability() {
    Fixture fixture;
    fixture.readiness.stable_time_s = 0.02;
    CHECK(!fixture.readiness.is_ready(fixture.status));
    std::this_thread::sleep_for(25ms);
    fixture.feed();
    CHECK(fixture.readiness.is_ready(fixture.status));
    fixture.status.drives_ready = false;
    CHECK(!fixture.readiness.is_ready(fixture.status));
    CHECK(!fixture.readiness.stable_since);
    fixture.status.drives_ready = true;
    CHECK(!fixture.readiness.is_ready(fixture.status));
    std::this_thread::sleep_for(25ms);
    fixture.feed();
    CHECK(fixture.readiness.is_ready(fixture.status));
    fixture.readiness.reset();
    CHECK(!fixture.readiness.stable_since && fixture.readiness.last_errors.empty());
    CHECK(!fixture.readiness.is_ready(fixture.status));
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
    CHECK(fixture.fsm.find_path(StateID::IDLE, StateID::IDLE) == std::vector{StateID::IDLE});
    CHECK(!fixture.fsm.target_state);
    for (StateID target : {StateID::STANDUP, StateID::LAYDOWN, StateID::FAILURE,
                           static_cast<StateID>(99)}) {
        expect_invalid_argument([&] { fixture.fsm.request_state(target); });
    }
    CHECK(!fixture.fsm.handle_event(Event::START_ACTION));
    CHECK(fixture.fsm.request_state(StateID::IDLE));
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::IDLE);
    fixture.status.estop_active = true;
    CHECK(!fixture.fsm.request_state(StateID::STANDING));
    CHECK(!fixture.fsm.handle_event(Event::START_STANDUP));
}

void transitions() {
    Fixture fixture;
    fixture.status.joint_feedback.reset();
    CHECK(fixture.fsm.request_state(StateID::STANDING));
    {
        CaptureOutput output;
        fixture.fsm.update();
        fixture.fsm.update();
        const std::string log = output.stream.str();
        const auto first = log.find("Ожидание условий");
        CHECK(first != std::string::npos);
        CHECK(log.find("Ожидание условий", first + 1) == std::string::npos);
    }
    CHECK(fixture.fsm.state_id == StateID::IDLE && !fixture.monitor.fault());
    CHECK(!fixture.readiness.last_errors.empty());
    fixture.controller.enable = true;
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::IDLE);
    CHECK(fixture.fsm.enable_actuator_commands());
    fixture.feed(2.0);
    fixture.fsm.update();
    CHECK(fixture.fsm.state_id == StateID::STANDUP);
    CHECK(fixture.fsm.request_state(StateID::IDLE)); // Новая цель не прерывает вставание.
    for (int i = 0; i < 2; ++i) {
        fixture.feed(2.0);
        fixture.fsm.update();
        CHECK(fixture.fsm.state_id == StateID::STANDUP);
    }
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::STANDING);
    CHECK(!fixture.fsm.enable_actuator_commands());
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::LAYDOWN);
    fixture.tick();
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::LAYDOWN);
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::IDLE);
    fixture.standing(); // Повторный вход сбрасывает счётчик STANDUP.
    for (StateID target : {StateID::ACTION, StateID::EXTERNAL}) {
        CHECK(fixture.fsm.request_state(target));
        fixture.tick();
        CHECK(fixture.fsm.state_id == target);
        CHECK(fixture.fsm.request_state(StateID::STANDING));
        fixture.tick();
        CHECK(fixture.fsm.state_id == StateID::STANDING);
    }
    CHECK(fixture.fsm.request_state(StateID::IDLE));
    fixture.tick();
    fixture.tick();
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::LAYDOWN); // Счётчик LAYDOWN тоже сброшен.
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::IDLE);
}

void emergency_stop_and_reset() {
    constexpr std::array states{
        StateID::IDLE, StateID::STANDUP, StateID::STANDING, StateID::LOCOMOTION,
        StateID::LAYDOWN, StateID::ACTION, StateID::EXTERNAL, StateID::FAILURE,
    };
    for (StateID state : states) {
        Fixture fixture;
        if (state == StateID::STANDUP) {
            CHECK(fixture.fsm.request_state(StateID::STANDING));
            fixture.tick();
        } else if (state != StateID::IDLE && state != StateID::FAILURE) {
            fixture.standing();
            if (state != StateID::STANDING) {
                CHECK(fixture.fsm.request_state(state == StateID::LAYDOWN ? StateID::IDLE : state));
                fixture.tick();
            }
        } else if (state == StateID::FAILURE) {
            CHECK(fixture.fsm.handle_event(Event::ESTOP));
        }
        CHECK(fixture.fsm.state_id == state);
        CaptureOutput output;
        CHECK(fixture.fsm.handle_event(Event::ESTOP));
        CHECK(fixture.fsm.state_id == StateID::FAILURE);
        CHECK(fixture.controller.enable && !fixture.fsm.target_state);
        CHECK(fixture.controller.velocity_command == std::array{0.0, 0.0, 0.0});
        if (state != StateID::FAILURE) {
            CHECK(fixture.controller.stop_states == std::vector{state, StateID::FAILURE});
            const std::string log = output.stream.str();
            CHECK(log.find("outputs inhibited") < log.find("Abort "));
            CHECK(log.find("Abort ") < log.find("Enter FailureState"));
            CHECK(log.find("Exit  ") == std::string::npos);
        }
        CHECK(!fixture.fsm.request_state(StateID::STANDING));
        CHECK(!fixture.fsm.handle_event(Event::START_STANDING));
        fixture.tick();
        CHECK(fixture.fsm.state_id == StateID::FAILURE);
    }

    Fixture fixture;
    fixture.locomotion();
    fixture.status.joint_feedback.reset();
    const int steps = fixture.controller.steps;
    fixture.fsm.update();
    CHECK(fixture.controller.steps == steps); // Безопасность проверяется до контроллера.
    CHECK(fixture.monitor.fault()->code == "JOINT_FEEDBACK_MISSING");
    CHECK(!fixture.fsm.handle_event(Event::CLEAR_FAULT));
    CHECK(!fixture.fsm.try_clear_fault(true)); // Нет актуальной телеметрии.
    fixture.feed(2.0);
    CHECK(!fixture.fsm.try_clear_fault(false));
    fixture.status.estop_active = true;
    CHECK(!fixture.fsm.try_clear_fault(true));
    fixture.status.estop_active = false;
    CHECK(fixture.fsm.try_clear_fault(true));
    CHECK(fixture.fsm.state_id == StateID::IDLE && !fixture.monitor.fault());
    CHECK(fixture.controller.enable && !fixture.fsm.target_state);
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::IDLE); // Цель автоматически не восстанавливается.
    CHECK(fixture.fsm.request_state(StateID::STANDING));
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::IDLE);
    CHECK(fixture.fsm.enable_actuator_commands());
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::STANDUP);
    fixture.status.joint_feedback->stamp_monotonic_s -= 1.0;
    fixture.fsm.update();
    CHECK(fixture.fsm.state_id == StateID::FAILURE);
    CHECK(fixture.monitor.fault()->code == "JOINT_FEEDBACK_STALE");

    Fixture external;
    external.status.estop_active = true;
    external.fsm.update();
    CHECK(external.fsm.state_id == StateID::FAILURE && !external.monitor.fault());
    external.status.estop_active = false;
    // Прямое CLEAR_FAULT имеет те же условия, что и в исходнике.
    CHECK(external.fsm.handle_event(Event::CLEAR_FAULT));
    CHECK(external.controller.enable);
}

void locomotion_stop() {
    Fixture fixture;
    fixture.locomotion();
    fixture.controller.set_velocity(0.2, 0.0, 0.0);
    fixture.tick();
    CHECK(fixture.controller.velocity_command[0] == 0.2);
    CHECK(fixture.fsm.request_state(StateID::ACTION));
    // Даже прямое событие не обходит запрос остановки в on_update().
    CHECK(!fixture.fsm.handle_event(Event::START_STANDING));

    const auto blocked = [&] {
        const int steps = fixture.controller.steps;
        fixture.tick();
        CHECK(fixture.fsm.state_id == StateID::LOCOMOTION);
        CHECK(fixture.controller.velocity_command == std::array{0.0, 0.0, 0.0});
        CHECK(fixture.controller.steps == steps + 1);
    };
    fixture.status.vx = 0.05;
    blocked();
    fixture.status.vx = 0.04;
    fixture.status.vy = 0.04; // Проверяется модуль скорости, а не отдельные компоненты.
    blocked();
    fixture.status.vx = fixture.status.vy = 0.0;
    fixture.status.yaw_rate = -0.10;
    blocked();
    fixture.status.yaw_rate = std::numeric_limits<double>::quiet_NaN();
    blocked();
    fixture.status.yaw_rate = 0.0;
    fixture.status.vx = std::numeric_limits<double>::infinity();
    blocked();
    fixture.status.vx = 0.0;
    fixture.status.contacts[0] = false;
    blocked();
    fixture.status.contacts.fill(true);
    fixture.status.contact_sensors_ready = false;
    blocked();
    fixture.status.contact_sensors_ready = true;
    fixture.status.gait_stopped = false;
    blocked();
    fixture.status.gait_stopped = true;
    blocked(); // Начало накопления 0.30 с устойчивости.
    std::this_thread::sleep_for(160ms);
    CHECK(fixture.fsm.request_state(StateID::LOCOMOTION));
    fixture.tick(); // Отмена остановки сбрасывает таймер.
    fixture.controller.set_velocity(0.2, 0.0, 0.0);
    fixture.tick();
    CHECK(fixture.controller.velocity_command[0] == 0.2);
    CHECK(fixture.fsm.request_state(StateID::ACTION));
    blocked();
    std::this_thread::sleep_for(160ms);
    blocked(); // Время предыдущего запроса не засчитывается.
    fixture.status.contacts[1] = false;
    blocked(); // Потеря контакта тоже сбрасывает таймер.
    fixture.status.contacts[1] = true;
    blocked();
    std::this_thread::sleep_for(160ms);
    blocked();
    std::this_thread::sleep_for(160ms);
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::STANDING);
    fixture.tick();
    CHECK(fixture.fsm.state_id == StateID::ACTION);
}

void controller_stub() {
    LocomotionControl controller;
    CHECK(!controller.input_mode);
    controller.set_data_input_mode("CONTROLLER");
    CHECK(controller.input_mode == "CONTROLLER");
    controller.set_data_input_mode("arbitrary mode"); // Заглушка не валидирует строки.
    CHECK(controller.input_mode == "arbitrary mode");
    controller.set_robot_cmd(RobotCommand{});
    controller.set_wbc_cmd(WbcCommand{});
    controller.set_joints_cmd(JointsCommand{});
    controller.set_locomotion_mode("WALK");
    controller.start_thread();
    controller.stop_thread();
    controller.set_velocity(1.0, 2.0, 3.0);
    CHECK(controller.velocity_command == std::array{1.0, 2.0, 3.0});
    {
        CaptureOutput output;
        controller.disable_outputs();
        const auto first_log = output.stream.str();
        controller.disable_outputs();
        CHECK(output.stream.str() == first_log);
    }
    controller.set_velocity(1.0, 2.0, 3.0);
    controller.step();
    CHECK(controller.velocity_command == std::array{0.0, 0.0, 0.0});
    controller.enable_outputs();
    controller.set_velocity(1.0, 2.0, 3.0);
    CHECK(controller.velocity_command == std::array{1.0, 2.0, 3.0});
}

} // namespace

int main() {
    struct Test {
        std::string_view name;
        void (*run)();
    };
    const std::array tests{
        Test{"configuration", configuration},
        Test{"safety_feedback", safety_feedback},
        Test{"safety_latching", safety_latching},
        Test{"readiness_diagnostics", readiness_diagnostics},
        Test{"readiness_stability", readiness_stability},
        Test{"routes_and_targets", routes_and_targets},
        Test{"transitions", transitions},
        Test{"emergency_stop_and_reset", emergency_stop_and_reset},
        Test{"locomotion_stop", locomotion_stop},
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
