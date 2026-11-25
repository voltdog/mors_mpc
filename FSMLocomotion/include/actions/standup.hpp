#pragma once

#include "LocomotionControlBase.hpp"

#include <variant>

#define STANDUP_ABAD_RAISED_RAD 1.0
#define STANDUP_HIP_RAISED_RAD 1.57
#define STANDUP_HIP_FOLDED_RAD 1.57
#define STANDUP_KNEE_RAISED_RAD 3.0
#define STANDUP_KNEE_FOLDED_RAD 2.9

namespace mors::fsm {

struct StandupConfig {
    double kp = 12.0;
    double kd = 0.4;
    double max_joint_speed_rad_s = 5.0;
};

class StandupAction {
public:
    enum class Progress { running, finished };
    using Result = std::variant<Progress, SafetyFault>;

    explicit StandupAction(LocomotionControlBase& controller, StandupConfig config = {});
    void reset();
    Result step(const LocomotionSetpoints& setpoints, const std::optional<SafetyFault>& joint_fault);

private:
    enum class Phase { start, gains, prepare, prepare_hold, contact, handoff, lift };

    std::optional<RobotStateSnapshot> supported_state(double contact_confirmed_at_s) const;

    LocomotionControlBase& controller_;
    StandupConfig config_;
    Phase phase_ = Phase::start;
    std::optional<RobotStateSnapshot> entry_state_;
    std::optional<double> last_state_at_s_;
    std::optional<double> stable_since_s_;
    std::optional<double> contact_confirmed_at_s_;
    std::optional<double> lost_support_at_s_;
    std::array<JointVector, 5> prep_poses_;
    std::size_t prep_segment_ = 0;
    std::array<double, NUM_LEGS> extension_s_{};
    double phase_start_s_ = 0.0;
    double start_z_ = 0.0;
    double target_z_ = 0.0;
    Eigen::Vector2d start_tilt_ = Eigen::Vector2d::Zero();
};

} // namespace mors::fsm
