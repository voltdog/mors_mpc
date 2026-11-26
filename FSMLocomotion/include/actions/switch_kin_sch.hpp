#pragma once

#include "LocomotionControlBase.hpp"

#include <variant>

namespace mors::fsm {

class SafetyMonitor;

enum class KneeScheme { inside, outside, back, front };

struct SwitchKinSchConfig {
    double kp = 12.0;
    double kd = 0.4;
    double max_joint_speed_rad_s = 1.0;
    double handoff_s = 0.4;
    double preparation_height_m = 0.27;
    double max_body_height_speed_mps = 0.05;
    double height_motion_min_s = 1.5;
};

struct SwitchKinSchPlan {
    JointVector initial;
    JointVector target;
    KneeScheme scheme = KneeScheme::inside;
};

std::variant<SwitchKinSchPlan, std::string> plan_switch_kin_sch(
    const JointVector& measured, KneeScheme scheme);

class SwitchKinSchAction {
public:
    enum class Progress { running, finished };
    using Result = std::variant<Progress, SafetyFault>;

    explicit SwitchKinSchAction(LocomotionControlBase& controller, SwitchKinSchConfig config = {});
    void reset(const SwitchKinSchPlan& plan);
    Result step(const LocomotionSetpoints& setpoints, const SafetyMonitor& safety);

private:
    enum class Phase { prepare_height, take_over, front, rear, return_control, handoff, restore_height, finished };
    JointCommand joint_command(const JointVector& position) const;
    void standing_command(const LocomotionSetpoints& setpoints);
    void start_height(double now, double from_z, double target_z);
    double return_height(const LocomotionSetpoints& setpoints) const;

    LocomotionControlBase& controller_;
    SwitchKinSchConfig config_;
    SwitchKinSchPlan plan_;
    Phase phase_ = Phase::finished;
    bool started_ = false;
    double phase_start_s_ = 0.0;
    std::optional<double> last_state_at_s_;
    std::optional<double> stable_since_s_, lost_support_since_s_;
    bool height_started_ = false;
    double initial_z_ = 0.0;
    double support_z_ = 0.0;
    double height_from_z_ = 0.0;
    double height_target_z_ = 0.0;
    double height_duration_s_ = 0.0;
    double handoff_z_ = 0.0;
};

} // namespace mors::fsm
