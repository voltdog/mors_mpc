#pragma once

#include "FSM/FSMTypes.hpp"
#include "LocomotionData.hpp"

namespace mors::fsm {

struct LocomotionTransitionConfig {
    double command_timeout_s = 0.50;
    double start_linear_mps = 0.05;
    double start_yaw_radps = 0.10;
    double stop_linear_mps = 0.025;
    double stop_yaw_radps = 0.05;
    double stop_hold_s = 0.30;

    struct RecoveryThresholds {
        double position_m;
        double velocity_mps;
    };
    RecoveryThresholds recovery_entry{0.10, 0.20};
    RecoveryThresholds recovery_exit_walking{0.07, 0.12};
    RecoveryThresholds recovery_exit_standing{0.04, 0.08};
};

class LocomotionTransitionMonitor {
public:
    explicit LocomotionTransitionMonitor(LocomotionTransitionConfig config = {});

    bool velocity_is_fresh(const LocomotionSetpoints& setpoints, double now_s) const;
    void on_state_entered(StateID state, const RobotData* estimate);
    // Вызывается в STANDING и LOCOMOTION. Скорость просроченной команды уже обнулена.
    LocomotionMode recommend(StateID state, bool walking_allowed, bool recovering,
                             const RobotData* estimate, const RobotData& command, double now_s);

private:
    void capture_standing_xy(const RobotData* estimate);

    const LocomotionTransitionConfig config_;
    std::optional<Eigen::Vector2d> standing_xy_;
    std::optional<double> below_stop_since_s_;
    bool stopping_ = false;
};

} // namespace mors::fsm
