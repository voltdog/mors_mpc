#pragma once

#include "WBICThread.hpp"
#include <filesystem>


namespace mors::fsm {

struct VisionConfig {
    explicit VisionConfig(const std::filesystem::path& directory);

    RobotPhysicalParams robot{};
    WBICThreadConfig wbic;
    float kd_idle;
    double touchdown_phase, contact_velocity, transition_s;
    int contact_debounce;
    double filter_hz, zero_velocity, foot_radius;
    std::array<double, 4> interleave_x, interleave_y;
    double near_ground_velocity, rising_proportion, footstep_gain;
    double dt, mpc_dt, friction, f_min, f_max;
    double recovery_stance, recovery_swing, handoff_s;
    int horizon;
    Eigen::VectorXd Q, R;
    bool local_velocity;
};

} // namespace mors::fsm
