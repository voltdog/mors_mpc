#pragma once

#include "structs.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace mors::fsm {

enum class LocomotionMode { STANDING, WALKING, RECOVERY };

struct BodyHeightCommand {
    double world_z_m;
    double velocity_mps;
};

struct RobotStateSnapshot {
    RobotData body;
    LegData legs;
    ServoStateData joints;
    std::array<bool, 19> readiness{};
    double received_at_s = 0.0;
};

struct LocomotionSetpoints {
    RobotData robot_cmd;
    std::vector<bool> active_legs{true, true, true, true};
    std::uint8_t adaptation_type = INCL_ADAPT;
    double t_st = 0.3;
    double t_sw = 0.2;
    std::vector<double> gait_type{0.0, 0.5, 0.5, 0.0};
    double stride_height = 0.06;
    std::optional<double> robot_cmd_received_at_s;
    LocomotionMode mode = LocomotionMode::WALKING;
};

} // namespace mors::fsm
