#pragma once

#include <array>
#include <optional>
#include <string>
#include <string_view>
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
using JointLimitList = std::vector<std::pair<std::string, JointLimits>>;

struct SafetyFault {
    std::string code;
    std::string description;
    double timestamp_monotonic_s;
};

struct SafetyDiagnostics {
    std::optional<SafetyFault> telemetry_fault;
    std::optional<SafetyFault> joint_fault;
    bool contact_sensors_ready = false;
};

struct GaitStopConfig {
    double linear_mps = 0.05;
    double yaw_radps = 0.10;
    double hold_s = 0.30;
};

struct RobotStatus {
    SafetyDiagnostics safety;
    bool ready_to_move = false;

    double vx = 0.0;        // м/с, в системе координат корпуса.
    double vy = 0.0;        // м/с.
    double yaw_rate = 0.0;  // рад/с.
    std::array<bool, 4> contacts{};
    bool gait_stopped = false;
};

} // namespace mors::fsm
