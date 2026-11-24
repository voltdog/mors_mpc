#include "actions/standup.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mors::fsm {
namespace {

constexpr double fold_s = 0.5;
constexpr double contact_motion_s = 0.5;
constexpr double contact_timeout_s = 1.0;
constexpr double contact_hold_s = 0.03;
constexpr double contact_loss_s = 0.08;
constexpr double lift_s = 1.5;
constexpr double settle_timeout_s = 1.0;
constexpr double settle_hold_s = 0.10;
constexpr double height_tolerance_m = 0.02;
constexpr double vertical_speed_tolerance_mps = 0.05;
constexpr double default_height_m = 0.20;

constexpr std::array hip_sign{-1.0, 1.0, 1.0, -1.0}; // R1, L1, R2, L2
constexpr std::array feet{&LegData::r1_pos, &LegData::l1_pos, &LegData::r2_pos, &LegData::l2_pos};

bool all_contacts(const RobotStateSnapshot& state) {
    return state.legs.contacts.size() == NUM_LEGS &&
           std::ranges::all_of(state.legs.contacts, [](bool contact) { return contact; });
}

struct Ramp { double position, velocity; };

Ramp cubic(double elapsed, double duration) {
    const double s = std::clamp(elapsed / duration, 0.0, 1.0);
    return {s * s * (3.0 - 2.0 * s), 6.0 * s * (1.0 - s) / duration};
}

} // namespace

StandupAction::StandupAction(LocomotionControlBase& controller, StandupConfig config)
    : controller_(controller), config_(config) {
    if (!std::isfinite(config_.kp) || config_.kp <= 0.0 ||
        !std::isfinite(config_.kd) || config_.kd < 0.0) {
        throw std::invalid_argument("Invalid standup joint gains");
    }
}

void StandupAction::reset() {
    entry_state_ = controller_.robot_state();
    phase_ = Phase::start;
    last_state_at_s_.reset();
    stable_since_s_.reset();
    contact_confirmed_at_s_.reset();
    lost_support_at_s_.reset();
    extension_s_.fill(0.0);
}

std::optional<RobotStateSnapshot> StandupAction::supported_state(double contact_confirmed_at_s) const {
    auto state = controller_.robot_state();
    if (!entry_state_ || !state || state->received_at_s <= std::max(entry_state_->received_at_s, contact_confirmed_at_s) ||
        !all_contacts(*state)) {
        return std::nullopt;
    }
    return state;
}

StandupAction::Result StandupAction::step(const LocomotionSetpoints& setpoints,
                                         const std::optional<SafetyFault>& joint_fault) {
    const auto state = controller_.robot_state();
    if (!entry_state_ || !state) {
        return SafetyFault{"STANDUP_STATE", "Missing standup observation", monotonic_seconds()};
    }
    const double now = state->received_at_s;
    if (last_state_at_s_ && now <= *last_state_at_s_) return Progress::running;
    const double dt = now - last_state_at_s_.value_or(now);
    last_state_at_s_ = now;
    const double elapsed = now - phase_start_s_;

    if (!state->joints.position.allFinite() || !state->body.pos.allFinite() ||
        !state->body.orientation.allFinite() || !state->body.lin_vel.allFinite() ||
        state->legs.contacts.size() != NUM_LEGS) {
        return SafetyFault{"STANDUP_STATE", "Invalid standup observation", now};
    }

    if (phase_ == Phase::start || phase_ == Phase::fold || phase_ == Phase::contact) {
        JointCommand command;
        command.kp.setConstant(config_.kp);
        command.kd.setConstant(config_.kd);
        JointVector folded;
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            const double direction = hip_sign[leg];
            folded.segment<3>(3 * leg) << 0.0, direction * 1.57, -direction * 2.9;
        }

        if (phase_ == Phase::start) {
            const auto& q = entry_state_->joints.position;
            const auto& tilt = entry_state_->body.orientation;
            if (!q.allFinite() || !(std::cos(tilt.x()) * std::cos(tilt.y()) > 0.0)) {
                return SafetyFault{"STANDUP_POSE", "Standup requires an upright X pose", now};
            }
            for (int leg = 0; leg < NUM_LEGS; ++leg) {
                if (hip_sign[leg] * q(3 * leg + 1) <= 0.0 || hip_sign[leg] * q(3 * leg + 2) >= 0.0) {
                    return SafetyFault{"STANDUP_POSE", "Standup requires an upright X pose", now};
                }
            }
            controller_.set_data_input_mode(LocomotionControlBase::joints_input);
            command.position = q;
            controller_.set_joints_cmd(command);
            phase_ = Phase::fold;
            phase_start_s_ = now;
            return Progress::running;
        }

        if (phase_ == Phase::fold) {
            const Ramp ramp = cubic(elapsed, fold_s);
            const JointVector delta = folded - entry_state_->joints.position;
            command.position = entry_state_->joints.position + ramp.position * delta;
            command.velocity = ramp.velocity * delta;
            controller_.set_joints_cmd(command);
            if (elapsed >= fold_s) {
                phase_ = Phase::contact;
                phase_start_s_ = now;
            }
            return Progress::running;
        }

        if (elapsed >= contact_timeout_s) {
            return SafetyFault{"STANDUP_CONTACT_TIMEOUT", "Four support contacts were not confirmed", now};
        }
        command.position = folded;
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            const bool touching = state->legs.contacts[leg];
            if (!touching) extension_s_[leg] = std::min(extension_s_[leg] + dt, contact_motion_s);
            const Ramp ramp = cubic(extension_s_[leg], contact_motion_s);
            const Eigen::Vector2d delta = hip_sign[leg] * Eigen::Vector2d(-0.17, 0.10);
            command.position.segment<2>(3 * leg + 1) += ramp.position * delta;
            if (!touching) command.velocity.segment<2>(3 * leg + 1) = ramp.velocity * delta;
        }
        controller_.set_joints_cmd(command);

        if (!all_contacts(*state)) {
            stable_since_s_.reset();
            contact_confirmed_at_s_.reset();
            return Progress::running;
        }
        if (!stable_since_s_) stable_since_s_ = now;
        if (now - *stable_since_s_ < contact_hold_s) return Progress::running;
        if (!contact_confirmed_at_s_) contact_confirmed_at_s_ = now;
        const auto support = supported_state(*contact_confirmed_at_s_);
        if (!support) return Progress::running;
        if (joint_fault) return *joint_fault;

        double support_z = 0.0;
        for (auto foot : feet) {
            const auto& position = support->legs.*foot;
            if (!position.allFinite()) return SafetyFault{"STANDUP_STATE", "Invalid support position", now};
            support_z += position.z() / NUM_LEGS;
        }
        start_z_ = support->body.pos.z();
        start_tilt_ = support->body.orientation.head<2>();
        const bool has_height = setpoints.robot_cmd_received_at_s.has_value();
        const bool absolute = has_height && setpoints.adaptation_type == NOADAPT;
        target_z_ = absolute ? setpoints.robot_cmd.pos.z()
                             : support_z + (has_height ? setpoints.robot_cmd.pos.z() : default_height_m);
        if (!std::isfinite(start_z_) || !start_tilt_.allFinite() || !std::isfinite(target_z_) ||
            target_z_ <= support_z) {
            return SafetyFault{"STANDUP_HEIGHT", "Invalid standup target height or starting pose", now};
        }

        controller_.set_data_input_mode(LocomotionControlBase::robot_input);
        controller_.set_locomotion_mode(LocomotionControlBase::standing_mode);
        controller_.set_active_legs({true, true, true, true});
        // Траектория подъёма уже задана в мировой системе координат.
        controller_.set_adaptation_type(NOADAPT);
        controller_.hold_measured_pose();
        phase_ = Phase::handoff;
        stable_since_s_.reset();
        return Progress::running;
    }

    if (joint_fault) return *joint_fault;
    if (all_contacts(*state)) {
        lost_support_at_s_.reset();
    } else {
        if (!lost_support_at_s_) lost_support_at_s_ = now;
        if (now - *lost_support_at_s_ >= contact_loss_s) {
            return SafetyFault{"STANDUP_CONTACT_LOST", "Lost support during standup", now};
        }
    }

    if (phase_ == Phase::handoff) {
        if (!controller_.robot_control_ready()) return Progress::running;
        phase_ = Phase::lift;
        phase_start_s_ = now;
        return Progress::running;
    }

    const Ramp ramp = cubic(elapsed, lift_s);
    RobotData command;
    command.pos.z() = std::lerp(start_z_, target_z_, ramp.position);
    command.lin_vel.z() = ramp.velocity * (target_z_ - start_z_);
    command.orientation.head<2>() = (1.0 - ramp.position) * start_tilt_;
    controller_.set_robot_cmd(command);

    if (elapsed >= lift_s && all_contacts(*state) &&
        std::abs(state->body.pos.z() - target_z_) <= height_tolerance_m &&
        std::abs(state->body.lin_vel.z()) <= vertical_speed_tolerance_mps) {
        if (!stable_since_s_) stable_since_s_ = now;
        if (now - *stable_since_s_ >= settle_hold_s) return Progress::finished;
    } else {
        stable_since_s_.reset();
    }
    if (elapsed >= lift_s + settle_timeout_s) {
        return SafetyFault{"STANDUP_HEIGHT_TIMEOUT", "Body did not settle at the standup height", now};
    }
    return Progress::running;
}

} // namespace mors::fsm
