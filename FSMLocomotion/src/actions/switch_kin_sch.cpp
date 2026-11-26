#include "actions/switch_kin_sch.hpp"
#include "SafetyMonitor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mors::fsm {
namespace {

constexpr double height_tolerance_m = 0.005;
constexpr double height_speed_tolerance_mps = 0.02;
constexpr double height_hold_s = 0.1;
constexpr double height_settle_timeout_s = 2.0;
constexpr double contact_loss_s = 0.08;
constexpr std::array feet{&LegData::r1_pos, &LegData::l1_pos, &LegData::r2_pos, &LegData::l2_pos};

bool all_contacts(const RobotStateSnapshot& state) {
    return state.legs.contacts.size() == NUM_LEGS &&
        std::ranges::all_of(state.legs.contacts, [](bool contact) { return contact; });
}

} // namespace

std::variant<SwitchKinSchPlan, std::string> plan_switch_kin_sch(
    const JointVector& measured, KneeScheme scheme) {
    std::array<int, NUM_LEGS> signs;
    switch (scheme) {
    case KneeScheme::inside:  signs = { 1, -1, -1,  1}; break;
    case KneeScheme::outside: signs = {-1,  1,  1, -1}; break;
    case KneeScheme::back:    signs = { 1, -1,  1, -1}; break;
    case KneeScheme::front:   signs = {-1,  1, -1,  1}; break;
    default: return "Invalid knee scheme";
    }
    if (!measured.allFinite()) return "Missing or invalid measured joint angles";
    SwitchKinSchPlan plan{measured, measured, scheme};
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        const int knee = 3 * leg + 2;
        if (measured(knee) == 0.0) return "Knee direction is undefined at zero angle";
        if ((measured(knee) > 0.0) != (signs[leg] > 0)) {
            plan.target(knee - 1) = -measured(knee - 1);
            plan.target(knee) = -measured(knee);
        }
    }
    return plan;
}

SwitchKinSchAction::SwitchKinSchAction(LocomotionControlBase& controller, SwitchKinSchConfig config)
    : controller_(controller), config_(config) {
    constexpr double max_gain = std::numeric_limits<float>::max();
    if (!(config_.kp > 0.0 && config_.kp <= max_gain) ||
        !(config_.kd >= 0.0 && config_.kd <= max_gain) ||
        !std::isfinite(config_.max_joint_speed_rad_s) || config_.max_joint_speed_rad_s <= 0.0 ||
        !std::isfinite(config_.handoff_s) || config_.handoff_s <= 0.0 ||
        !std::isfinite(config_.preparation_height_m) || config_.preparation_height_m <= 0.0 ||
        !std::isfinite(config_.max_body_height_speed_mps) || config_.max_body_height_speed_mps <= 0.0 ||
        !std::isfinite(config_.height_motion_min_s) || config_.height_motion_min_s <= 0.0) {
        throw std::invalid_argument("Invalid switch_kin_sch configuration");
    }
}

void SwitchKinSchAction::reset(const SwitchKinSchPlan& plan) {
    plan_ = plan;
    phase_ = (plan.target - plan.initial).isZero(0.0) ? Phase::finished : Phase::prepare_height;
    started_ = false;
    last_state_at_s_.reset();
    stable_since_s_.reset();
    lost_support_since_s_.reset();
    height_started_ = false;
}

void SwitchKinSchAction::start_height(double now, double from_z, double target_z) {
    height_from_z_ = from_z;
    height_target_z_ = target_z;
    const double delta = std::abs(target_z - from_z);
    height_duration_s_ = delta == 0.0 ? 0.0 : std::max(config_.height_motion_min_s,
        1.5 * (delta / config_.max_body_height_speed_mps));
    phase_start_s_ = now;
    stable_since_s_.reset();
    height_started_ = true;
}

double SwitchKinSchAction::return_height(const LocomotionSetpoints& setpoints) const {
    if (!setpoints.robot_cmd_received_at_s) return initial_z_;
    return setpoints.robot_cmd.pos.z() + (setpoints.adaptation_type == NOADAPT ? 0.0 : support_z_);
}

JointCommand SwitchKinSchAction::joint_command(const JointVector& position) const {
    JointCommand command;
    command.position = position;
    command.kp.setConstant(config_.kp);
    command.kd.setConstant(config_.kd);
    return command;
}

void SwitchKinSchAction::standing_command(const LocomotionSetpoints& setpoints) {
    controller_.set_adaptation_type(setpoints.adaptation_type);
    controller_.set_active_legs(setpoints.active_legs);
    if (setpoints.robot_cmd_received_at_s) {
        RobotData command = setpoints.robot_cmd;
        command.lin_vel.setZero();
        command.ang_vel.setZero();
        controller_.set_robot_cmd(command);
    } else {
        controller_.hold_measured_pose();
    }
}

SwitchKinSchAction::Result SwitchKinSchAction::step(const LocomotionSetpoints& setpoints, const SafetyMonitor& safety) {
    if (phase_ == Phase::finished) return Progress::finished;
    const auto state = controller_.robot_state();
    if (!state || !std::isfinite(state->received_at_s) || !state->joints.position.allFinite() ||
        !state->body.pos.allFinite() || !state->body.lin_vel.allFinite() || state->legs.contacts.size() != NUM_LEGS) {
        return SafetyFault{"SWITCH_KIN_SCH_STATE", "Missing or invalid joint observation", monotonic_seconds()};
    }
    if (setpoints.adaptation_type > NOADAPT)
        return SafetyFault{"SWITCH_KIN_SCH_HEIGHT", "Invalid height adaptation type", state->received_at_s};
    for (auto foot : feet) {
        if (!(state->legs.*foot).allFinite())
            return SafetyFault{"SWITCH_KIN_SCH_STATE", "Invalid support position", state->received_at_s};
    }
    const double now = state->received_at_s;
    if (last_state_at_s_ && now <= *last_state_at_s_) return Progress::running;
    last_state_at_s_ = now;

    if (phase_ == Phase::prepare_height || phase_ == Phase::restore_height) {
        if (!height_started_) {
            initial_z_ = state->body.pos.z();
            support_z_ = 0.0;
            for (auto foot : feet) {
                support_z_ += (state->legs.*foot).z() / NUM_LEGS;
            }
            const double target = config_.preparation_height_m +
                (setpoints.adaptation_type == NOADAPT ? 0.0 : support_z_);
            start_height(now, initial_z_, target);
        }
        if (!std::isfinite(height_duration_s_) || !std::isfinite(height_target_z_) || height_target_z_ <= support_z_) {
            return SafetyFault{"SWITCH_KIN_SCH_HEIGHT", "Invalid height trajectory", now};
        }
        if (all_contacts(*state)) {
            lost_support_since_s_.reset();
        } else {
            if (!lost_support_since_s_) lost_support_since_s_ = now;
            if (now - *lost_support_since_s_ >= contact_loss_s)
                return SafetyFault{"SWITCH_KIN_SCH_CONTACT_LOST", "Lost support during height motion", now};
        }
        const double elapsed = now - phase_start_s_;
        BodyHeightCommand command{height_target_z_, 0.0};
        if (elapsed < height_duration_s_) {
            const double u = std::clamp(elapsed / height_duration_s_, 0.0, 1.0);
            command.world_z_m = std::lerp(height_from_z_, height_target_z_, u * u * (3.0 - 2.0 * u));
            command.velocity_mps = 6.0 * u * (1.0 - u) * (height_target_z_ - height_from_z_) / height_duration_s_;
        }
        controller_.set_body_height_override(command);
        standing_command(setpoints);
        if (elapsed < height_duration_s_) return Progress::running;

        if (phase_ == Phase::restore_height) {
            const double next = return_height(setpoints);
            if (!std::isfinite(next) || next <= support_z_)
                return SafetyFault{"SWITCH_KIN_SCH_HEIGHT", "Invalid return height", now};
            if (next != height_target_z_) {
                start_height(now, height_target_z_, next);
                return Progress::running;
            }
        }
        if (all_contacts(*state) && std::abs(state->body.pos.z() - height_target_z_) <= height_tolerance_m &&
            std::abs(state->body.lin_vel.z()) <= height_speed_tolerance_mps) {
            if (!stable_since_s_) stable_since_s_ = now;
            if (now - *stable_since_s_ >= height_hold_s) {
                if (phase_ == Phase::prepare_height) {
                    const auto replanned = plan_switch_kin_sch(state->joints.position, plan_.scheme);
                    if (const auto* reason = std::get_if<std::string>(&replanned))
                        return SafetyFault{"SWITCH_KIN_SCH_POSE", *reason, now};
                    plan_ = std::get<SwitchKinSchPlan>(replanned);
                    auto target = *state;
                    target.joints.position = plan_.target;
                    if (const auto fault = safety.evaluate(target, now).joint_fault) return *fault;
                    if ((plan_.target - plan_.initial).isZero(0.0)) {
                        phase_ = Phase::restore_height;
                        start_height(now, height_target_z_, return_height(setpoints));
                    } else {
                        phase_ = Phase::take_over;
                    }
                } else {
                    if (!setpoints.robot_cmd_received_at_s)
                        controller_.set_locomotion_mode(LocomotionControlBase::standing_mode);
                    controller_.set_body_height_override(std::nullopt);
                    standing_command(setpoints);
                    phase_ = Phase::finished;
                    return Progress::finished;
                }
                return Progress::running;
            }
        } else {
            stable_since_s_.reset();
        }
        if (elapsed >= height_duration_s_ + height_settle_timeout_s)
            return SafetyFault{"SWITCH_KIN_SCH_HEIGHT_TIMEOUT", "Body did not settle at the requested height", now};
        return Progress::running;
    }

    if (phase_ == Phase::take_over) {
        if (!started_) {
            controller_.begin_input_handoff(LocomotionControlBase::joints_input,
                                            config_.handoff_s, config_.max_joint_speed_rad_s);
            controller_.set_joints_cmd(joint_command(plan_.initial));
            started_ = true;
        }
        if (controller_.joints_control_ready()) {
            phase_ = Phase::front;
            phase_start_s_ = now;
        }
        return Progress::running;
    }

    if (phase_ == Phase::front || phase_ == Phase::rear) {
        JointVector from = plan_.initial;
        JointVector target = plan_.target;
        if (phase_ == Phase::front) target.tail<6>() = plan_.initial.tail<6>();
        else from.head<6>() = plan_.target.head<6>();
        const JointVector delta = target - from;
        const double duration = 1.5 * (delta.cwiseAbs().maxCoeff() / config_.max_joint_speed_rad_s);
        if (!std::isfinite(duration)) {
            return SafetyFault{"SWITCH_KIN_SCH_TRAJECTORY", "Invalid trajectory duration", now};
        }
        const double elapsed = now - phase_start_s_;
        auto command = joint_command(target);
        if (elapsed < duration) {
            const double u = std::clamp(elapsed / duration, 0.0, 1.0);
            command.position = from + u * u * (3.0 - 2.0 * u) * delta;
            command.velocity = (6.0 * u * (1.0 - u) / duration) * delta;
        } else {
            phase_ = phase_ == Phase::front ? Phase::rear : Phase::return_control;
            phase_start_s_ = now;
        }
        controller_.set_joints_cmd(command);
        return Progress::running;
    }

    if (phase_ == Phase::return_control) {
        controller_.begin_input_handoff(LocomotionControlBase::robot_input,
                                        config_.handoff_s, config_.max_joint_speed_rad_s);
        controller_.set_locomotion_mode(LocomotionControlBase::standing_mode);
        handoff_z_ = state->body.pos.z();
        controller_.set_body_height_override(BodyHeightCommand{handoff_z_, 0.0});
        phase_ = Phase::handoff;
    }
    controller_.set_adaptation_type(setpoints.adaptation_type);
    controller_.set_active_legs(setpoints.active_legs);
    controller_.hold_measured_pose();
    if (!controller_.robot_control_ready()) return Progress::running;
    phase_ = Phase::restore_height;
    start_height(now, handoff_z_, return_height(setpoints));
    return Progress::running;
}

} // namespace mors::fsm
