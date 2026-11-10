#include "SafetyMonitor.hpp"

#include <cmath>
#include <format>
#include <stdexcept>
#include <utility>

namespace mors::fsm {

SafetyMonitor::SafetyMonitor(JointLimitList joint_limits, double max_feedback_age_s)
    : joint_limits(std::move(joint_limits)), max_feedback_age_s(max_feedback_age_s) {
    if (this->joint_limits.empty()) {
        throw std::invalid_argument("Configure limits for every controlled joint");
    }
    if (!std::isfinite(max_feedback_age_s) || max_feedback_age_s <= 0.0) {
        throw std::invalid_argument("max_feedback_age_s must be positive");
    }
}

const std::optional<SafetyFault>& SafetyMonitor::fault() const {
    return latched_fault_;
}

std::optional<SafetyFault> SafetyMonitor::evaluate(
    const std::optional<JointFeedback>& feedback, bool feedback_required,
    std::optional<StateID> fsm_state) const {
    const double now = monotonic_seconds();
    const auto fault = [now](std::string code, std::string description) {
        return SafetyFault{std::move(code), std::move(description), now};
    };

    if (!feedback_required) {
        return std::nullopt;
    }
    if (!feedback) {
        return fault("JOINT_FEEDBACK_MISSING", "No joint feedback");
    }

    const double stamp = feedback->stamp_monotonic_s;
    if (!std::isfinite(stamp)) {
        return fault("JOINT_FEEDBACK_INVALID", "Non-finite feedback timestamp");
    }
    const double age = now - stamp;
    if (age < -0.001) {
        return fault("JOINT_FEEDBACK_INVALID", "Feedback timestamp is in the future");
    }
    if (age > max_feedback_age_s) {
        return fault("JOINT_FEEDBACK_STALE", std::format("Joint feedback age {:.3f} s", age));
    }

    // Даже без проверки углов нужны актуальные показания всех энкодеров.
    const bool check_angle_limits = fsm_state != StateID::IDLE && fsm_state != StateID::STANDUP;
    for (const auto& [joint, limits] : joint_limits) {
        const auto it = feedback->positions_rad.find(joint);
        if (it == feedback->positions_rad.end()) {
            return fault("JOINT_MISSING", "Missing encoder position for " + joint);
        }
        const double q = it->second;
        if (!std::isfinite(q)) {
            return fault("JOINT_INVALID", std::format("{}: non-finite angle: {}", joint, q));
        }
        if (check_angle_limits && !(limits.minimum_rad <= q && q <= limits.maximum_rad)) {
            return fault("JOINT_LIMIT", std::format(
                "{}: q={:.4f} rad outside [{:.4f}, {:.4f}] rad",
                joint, q, limits.minimum_rad, limits.maximum_rad));
        }
    }
    return std::nullopt;
}

const std::optional<SafetyFault>& SafetyMonitor::check(
    const std::optional<JointFeedback>& feedback, bool feedback_required,
    std::optional<StateID> fsm_state) {
    auto detected = evaluate(feedback, feedback_required, fsm_state);
    if (!latched_fault_ && detected) {
        latched_fault_ = std::move(detected);
    }
    return latched_fault_;
}

bool SafetyMonitor::is_encoder_feedback_valid(const std::optional<JointFeedback>& feedback) const {
    return !latched_fault_ && !evaluate(feedback, true, StateID::IDLE);
}

bool SafetyMonitor::clear_fault(const std::optional<JointFeedback>& feedback,
                                bool operator_acknowledged, bool external_estop_released) {
    if (!operator_acknowledged || !external_estop_released) {
        return false;
    }
    // Сброс ведёт в IDLE, где положение за пределами углов разрешено.
    if (evaluate(feedback, true, StateID::IDLE)) {
        return false;
    }
    latched_fault_.reset();
    return true;
}

} // namespace mors::fsm
