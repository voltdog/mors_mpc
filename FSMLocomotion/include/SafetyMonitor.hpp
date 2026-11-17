#pragma once

#include "FSM/FSMTypes.hpp"
#include "LocomotionData.hpp"

namespace mors::fsm {

// Программный монитор фиксирует первую ошибку до явного сброса.
// Он не заменяет независимую аппаратную защиту и ограничения приводов.
class SafetyMonitor {
public:
    explicit SafetyMonitor(JointLimitList joint_limits, double robot_state_timeout_s);

    SafetyDiagnostics evaluate(const std::optional<RobotStateSnapshot>& observation,
                               double now) const;
    const std::optional<SafetyFault>& fault() const;
    void latch_fault(const SafetyFault& fault);
    void clear_fault();

private:
    JointLimitList joint_limits_;
    double robot_state_timeout_s_;
    std::optional<SafetyFault> latched_fault_;
};

} // namespace mors::fsm
