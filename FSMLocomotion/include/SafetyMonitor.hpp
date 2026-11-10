#pragma once

#include "FSMTypes.hpp"

namespace mors::fsm {

// Программный монитор фиксирует первую ошибку до явного сброса.
// Он не заменяет независимую аппаратную защиту и ограничения приводов.
class SafetyMonitor {
public:
    explicit SafetyMonitor(JointLimitList joint_limits, double max_feedback_age_s = 0.1);

    const std::optional<SafetyFault>& fault() const;

    // Проверка без изменения зафиксированной ошибки. В IDLE/STANDUP углы не ограничены.
    std::optional<SafetyFault> evaluate(
        const std::optional<JointFeedback>& feedback,
        bool feedback_required = true,
        std::optional<StateID> fsm_state = std::nullopt) const;

    const std::optional<SafetyFault>& check(
        const std::optional<JointFeedback>& feedback,
        bool feedback_required = true,
        std::optional<StateID> fsm_state = std::nullopt);

    bool is_encoder_feedback_valid(const std::optional<JointFeedback>& feedback) const;
    bool clear_fault(const std::optional<JointFeedback>& feedback,
                     bool operator_acknowledged, bool external_estop_released);

    JointLimitList joint_limits;
    double max_feedback_age_s;

private:
    std::optional<SafetyFault> latched_fault_;
};

} // namespace mors::fsm
