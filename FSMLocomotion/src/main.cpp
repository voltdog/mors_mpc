#include "RobotFSM.hpp"

#include <chrono>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>

namespace {

// В отличие от assert(), проверки демонстрации работают и в Release.
void require(bool condition, std::string_view message) {
    if (!condition) {
        throw std::runtime_error(std::string(message));
    }
}

} // namespace

int main() try {
    using namespace mors::fsm;
    using namespace std::chrono_literals;

    // Только демонстрация. Для MORS нужны ограничения всех 12 суставов.
    SafetyMonitor monitor({
        {"demo_hip", JointLimits(-1.0, 1.0)},
        {"demo_thigh", JointLimits(-1.0, 1.0)},
        {"demo_knee", JointLimits(-1.0, 1.0)},
    }, 0.15);
    ReadinessChecker readiness(monitor, 0.15, 0.10);
    RobotStatus status;
    RobotFSM fsm(monitor, &status, nullptr, &readiness);

    const auto feed = [&status](double knee = 0.0) {
        // Реальные источники обновляют эти поля только при приёме новых пакетов.
        const double now = monotonic_seconds();
        status.joint_feedback = JointFeedback{
            {{"demo_hip", 0.0}, {"demo_thigh", 0.0}, {"demo_knee", knee}}, now,
        };
        status.drive_status_stamp_s = now;
        status.contact_stamp_s = now;
        status.estimator_stamp_s = now;
    };

    std::cout << "\n1. Отсутствие данных не вызывает ESTOP, но блокирует выход из IDLE\n";
    require(fsm.request_state(StateID::STANDING), "Запрос STANDING отклонён");
    fsm.update();
    require(fsm.state_id == StateID::IDLE && !monitor.fault(), "Ожидался IDLE без аварии");
    std::cout << "Причины: (";
    for (std::size_t i = 0; i < readiness.last_errors.size(); ++i) {
        std::cout << (i == 0 ? "" : ", ") << '\'' << readiness.last_errors[i] << '\'';
    }
    std::cout << ")\n";

    std::cout << "\n2. После диагностики: IDLE -> STANDUP -> STANDING\n";
    status.drives_ready = status.contact_sensors_ready = status.estimator_ready = true;
    // В IDLE и STANDUP положение за пределами углов разрешено.
    for (int i = 0; i < 6; ++i) {
        feed(1.2);
        fsm.update();
        std::this_thread::sleep_for(25ms);
    }
    require(fsm.state_id == StateID::STANDUP && !monitor.fault(), "Ожидался STANDUP без аварии");
    for (int i = 0; i < 2; ++i) {
        feed(1.2);
        fsm.update();
    }
    feed();
    fsm.update();
    require(fsm.state_id == StateID::STANDING, "Вставание не завершено");

    std::cout << "\n3. Превышение угла в STANDING -> ESTOP\n";
    feed(1.2);
    fsm.update();
    require(fsm.state_id == StateID::FAILURE, "Не выполнен ESTOP по углу");
    require(monitor.fault() && monitor.fault()->code == "JOINT_LIMIT", "Ожидалась JOINT_LIMIT");
    require(fsm.controller.outputs_inhibited, "Выдача команд не заблокирована");

    std::cout << "\n4. Ручной сброс и разрешение выдачи команд\n";
    feed();
    require(!fsm.try_clear_fault(false), "Сброс без подтверждения оператора");
    require(fsm.try_clear_fault(true), "Ручной сброс отклонён");
    require(fsm.state_id == StateID::IDLE && fsm.controller.outputs_inhibited,
            "После сброса ожидался IDLE с блокировкой команд");
    for (int i = 0; i < 6; ++i) {
        feed();
        fsm.update();
        std::this_thread::sleep_for(25ms);
    }
    require(fsm.enable_actuator_commands(), "Разрешение выдачи команд отклонено");

    std::cout << "\n5. Потеря актуальных данных в STANDUP -> ESTOP\n";
    require(fsm.request_state(StateID::STANDING), "Повторный запрос STANDING отклонён");
    feed();
    fsm.update();
    require(fsm.state_id == StateID::STANDUP, "Ожидался STANDUP");
    std::this_thread::sleep_for(160ms);
    fsm.update();
    require(fsm.state_id == StateID::FAILURE, "Не выполнен ESTOP по потере данных");
    require(monitor.fault() && monitor.fault()->code == "JOINT_FEEDBACK_STALE",
            "Ожидалась JOINT_FEEDBACK_STALE");
    std::cout << "\nВсе демонстрационные проверки пройдены.\n";
    return 0;
} catch (const std::exception& error) {
    std::cerr << "Демонстрационная проверка не пройдена: " << error.what() << '\n';
    return 1;
}
