#include "RobotFSM.hpp"

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

void require(bool condition, std::string_view message) {
    if (!condition) {
        throw std::runtime_error(std::string(message));
    }
}

} // namespace

int main() try {
    using namespace mors::fsm;

    SafetyMonitor monitor({{"demo_hip", JointLimits(-1.0, 1.0)},
                           {"demo_thigh", JointLimits(-1.0, 1.0)},
                           {"demo_knee", JointLimits(-1.0, 1.0)}}, 0.05);
    ReadinessChecker readiness;
    RobotStatus status;
    RobotFSM fsm(monitor, &status, nullptr, &readiness);

    std::optional<RobotStateSnapshot> observation;
    const LocomotionSetpoints setpoints;
    double now = 0.0;
    const auto tick = [&](bool receive = true) {
        // Сценарий использует модельное время для свежести и готовности.
        now += 0.01;
        if (observation && receive) {
            observation->received_at_s = now;
        }
        status.safety = monitor.evaluate(observation, now);
        status.ready_to_move = readiness.is_ready(status, now);
        fsm.update(setpoints);
    };

    require(fsm.request_state(StateID::STANDING), "Запрос STANDING отклонён");
    tick();
    require(fsm.state_id == StateID::IDLE && !monitor.fault(), "Ожидался IDLE без аварии");

    observation.emplace();
    observation->readiness.fill(true);
    observation->joints.position[2] = 1.2;
    for (int i = 0; i < 25 && fsm.state_id == StateID::IDLE; ++i) {
        tick();
    }
    require(fsm.state_id == StateID::STANDUP, "Ожидался STANDUP");
    tick();
    tick();
    observation->joints.position[2] = 0.0;
    tick();
    require(fsm.state_id == StateID::STANDING, "Вставание не завершено");

    observation->joints.position[2] = 1.2;
    tick();
    require(fsm.state_id == StateID::FAILURE, "Не выполнен ESTOP по углу");
    require(monitor.fault() && monitor.fault()->code == "JOINT_LIMIT", "Ожидалась JOINT_LIMIT");
    require(!fsm.controller.outputs_enabled(), "Выдача команд не заблокирована");
    require(fsm.try_clear_fault().has_value(), "Сброс при нарушенном лимите");

    observation->joints.position[2] = 0.0;
    for (int i = 0; i < 25; ++i) {
        tick();
    }
    require(!fsm.try_clear_fault(), "Ручной сброс отклонён");
    require(fsm.state_id == StateID::IDLE && fsm.controller.outputs_enabled() && !fsm.target_state,
            "После сброса ожидался IDLE с разрешённой выдачей без цели");

    require(fsm.request_state(StateID::STANDING), "Повторный запрос STANDING отклонён");
    for (int i = 0; i < 25 && fsm.state_id == StateID::IDLE; ++i) {
        tick();
    }
    require(fsm.state_id == StateID::STANDUP, "Ожидался повторный STANDUP");
    for (int i = 0; i < 6; ++i) {
        tick(false);
    }
    require(fsm.state_id == StateID::FAILURE, "Не выполнен ESTOP по потере данных");
    require(monitor.fault() && monitor.fault()->code == "ROBOT_STATE_STALE",
            "Ожидалась ROBOT_STATE_STALE");
    std::cout << "Все демонстрационные проверки пройдены.\n";
    return 0;
} catch (const std::exception& error) {
    std::cerr << "Демонстрационная проверка не пройдена: " << error.what() << '\n';
    return 1;
}
