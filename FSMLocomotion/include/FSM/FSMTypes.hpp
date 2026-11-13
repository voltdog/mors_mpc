#pragma once

#include <array>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mors::fsm {

enum class StateID {
    IDLE = 1,
    STANDUP,
    STANDING,
    LOCOMOTION,
    LAYDOWN,
    ACTION,
    EXTERNAL,
    FAILURE,
};

enum class Event {
    START_STANDUP = 1,
    START_STANDING,
    START_LOCOMOTION,
    START_LAYDOWN,
    START_IDLE,
    START_ACTION,
    START_EXTERNAL,
    ESTOP,
    CLEAR_FAULT,
};

std::string_view state_name(StateID state);

// Все отметки времени используют одни монотонные часы, единица измерения: с.
double monotonic_seconds();

struct JointLimits {
    JointLimits(double minimum_rad, double maximum_rad);

    double minimum_rad;
    double maximum_rad;
};

// Порядок конфигурации определяет, какая из одновременных ошибок будет первой.
// Как в Python-словаре, каждому имени сустава соответствует одна запись.
using JointLimitList = std::vector<std::pair<std::string, JointLimits>>;

struct JointFeedback {
    std::unordered_map<std::string, double> positions_rad;
    double stamp_monotonic_s;  // Время приёма пакета на управляющем компьютере.
};

struct SafetyFault {
    std::string code;
    std::string description;
    double timestamp_monotonic_s;
};

struct RobotStatus {
    // Готовность источников перед выходом из IDLE.
    bool drives_ready = false;
    bool contact_sensors_ready = false;
    bool estimator_ready = false;

    double vx = 0.0;        // м/с, в системе координат корпуса.
    double vy = 0.0;        // м/с.
    double yaw_rate = 0.0;  // рад/с.
    std::array<bool, 4> contacts{};
    bool gait_stopped = false;

    bool estop_active = false;    // Внешняя кнопка или контроллер безопасности.
    bool drives_enabled = false; // Фактическое состояние силовых выходов.
    std::optional<JointFeedback> joint_feedback;

    // Обновлять только при получении новых пакетов, не на каждом такте FSM.
    std::optional<double> drive_status_stamp_s;
    std::optional<double> contact_stamp_s;
    std::optional<double> estimator_stamp_s;
};

} // namespace mors::fsm
