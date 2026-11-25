#include "actions/laydown.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <stdexcept>

namespace mors::fsm {
namespace {

constexpr double fold_s = 2.0;
constexpr double center_s = 0.5;
constexpr double damp_s = 0.5;
constexpr double folded_hip_rad = 1.57;
constexpr double folded_knee_rad = 2.9;
constexpr std::array<int, NUM_LEGS> x_hip_signs{-1, 1, 1, -1};
constexpr std::array<int, NUM_LEGS> m_hip_signs{-1, 1, -1, 1};

int sign(double angle) {
    return (angle > 0.0) - (angle < 0.0);
}

JointVector folded_pose(const JointVector& measured) {
    std::array<int, NUM_LEGS> hip_signs;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        hip_signs[leg] = sign(measured(3 * leg + 1));
    }

    const bool x_or_m = hip_signs == x_hip_signs || hip_signs == m_hip_signs;
    JointVector folded = measured;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        const int hip = 3 * leg + 1;
        const int knee = hip + 1;
        const int knee_sign = x_or_m ? -hip_signs[leg] : sign(measured(knee));
        folded(hip) = hip_signs[leg] * folded_hip_rad;
        folded(knee) = knee_sign * folded_knee_rad;
    }
    return folded;
}

} // namespace

LaydownAction::LaydownAction(LocomotionControlBase& controller, LaydownConfig config)
    : controller_(controller), config_(config) {
    constexpr double max_gain = std::numeric_limits<float>::max();
    if (!(config_.kp > 0.0 && config_.kp <= max_gain) ||
        !(config_.kd >= 0.0 && config_.kd <= max_gain / 2.0)) {
        throw std::invalid_argument("Invalid laydown joint gains");
    }
}

void LaydownAction::reset() {
    phase_ = Phase::fold;
    phase_started_at_s_.reset();
}

LaydownAction::Result LaydownAction::step(double now_s) {
    if (phase_ == Phase::released) {
        return Progress::finished;
    }

    if (!phase_started_at_s_) {
        if (phase_ == Phase::fold) {
            const auto observation = controller_.robot_state();
            if (!observation || !observation->joints.position.allFinite()) {
                return SafetyFault{"LAYDOWN_STATE", "Missing or invalid laydown joint position", now_s};
            }
            initial_ = observation->joints.position;
            folded_ = folded_pose(initial_);
            centered_ = folded_;
            for (int leg = 0; leg < NUM_LEGS; ++leg) {
                centered_(3 * leg) = 0.0;
            }
            controller_.set_data_input_mode(LocomotionControlBase::joints_input);
        }
        phase_started_at_s_ = now_s;
    }

    const double elapsed = now_s - *phase_started_at_s_;
    JointCommand command;
    if (phase_ == Phase::damp) {
        // Эталон сохраняет первую сложенную уставку при выключенном позиционном контуре.
        command.position = folded_;
        if (elapsed < damp_s) {
            command.kd.setConstant(2.0 * config_.kd);
        } else {
            phase_ = Phase::released;
        }
    } else {
        const bool folding = phase_ == Phase::fold;
        const double duration = folding ? fold_s : center_s;
        const auto& from = folding ? initial_ : folded_;
        const auto& to = folding ? folded_ : centered_;
        const double t = std::clamp(elapsed / duration, 0.0, 1.0);
        const double blend = t * t * (3.0 - 2.0 * t);
        command.position = from + blend * (to - from);
        command.kp.setConstant(config_.kp);
        command.kd.setConstant(config_.kd);
        if (elapsed >= duration) {
            phase_ = folding ? Phase::center : Phase::damp;
            phase_started_at_s_.reset();
        }
    }

    controller_.set_joints_cmd(command);
    return Progress::running;
}

} // namespace mors::fsm
