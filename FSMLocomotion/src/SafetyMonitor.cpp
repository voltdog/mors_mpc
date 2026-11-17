#include "SafetyMonitor.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <stdexcept>
#include <utility>

namespace mors::fsm {
namespace {

constexpr std::size_t first_contact_bit = 14;
constexpr std::size_t mandatory_bits = 18; // Бит 18 (T265) необязателен.

} // namespace

SafetyMonitor::SafetyMonitor(JointLimitList joint_limits, double robot_state_timeout_s)
    : joint_limits_(std::move(joint_limits)), robot_state_timeout_s_(robot_state_timeout_s) {
    if (joint_limits_.empty() || joint_limits_.size() > mors::wbic::kJointDofs) {
        throw std::invalid_argument("Invalid number of joint limits");
    }
    if (!std::isfinite(robot_state_timeout_s_) || robot_state_timeout_s_ <= 0.0) {
        throw std::invalid_argument("robot_state_timeout_ms must be positive");
    }
}

SafetyDiagnostics SafetyMonitor::evaluate(
    const std::optional<RobotStateSnapshot>& observation, double now) const 
{
    SafetyDiagnostics result;
    if (!observation) {
        result.telemetry_fault = SafetyFault{"ROBOT_STATE_MISSING", "No ROBOT_STATE received", now};
        return result;
    }

    const double age = now - observation->received_at_s;
    if (!std::isfinite(age) || age < 0.0 || age > robot_state_timeout_s_) {
        result.telemetry_fault = SafetyFault{
            "ROBOT_STATE_STALE", std::format("ROBOT_STATE age {:.3f} s", age), now,
        };
        return result;
    }

    const auto& readiness = observation->readiness;
    const auto mandatory_end = readiness.begin() + mandatory_bits;
    const auto missing = std::find(readiness.begin(), mandatory_end, false);
    if (missing != mandatory_end) {
        result.telemetry_fault = SafetyFault{"ROBOT_STATE_NOT_READY",
            std::format("Required readiness bit {} is false", missing - readiness.begin()), now};
    }
    result.contact_sensors_ready = std::all_of(readiness.begin() + first_contact_bit, mandatory_end, [](bool ready) { return ready; });

    // Результат нужен и на границе STANDUP -> STANDING, без повторной проверки пакета.
    for (std::size_t i = 0; i < joint_limits_.size(); ++i) {
        const auto& [joint, limits] = joint_limits_[i];
        const double q = observation->joints.position(i);
        if (!(limits.minimum_rad <= q && q <= limits.maximum_rad)) {
            result.joint_fault = SafetyFault{"JOINT_LIMIT",
                std::format("{}: q={:.4f} rad outside [{:.4f}, {:.4f}] rad",
                            joint, q, limits.minimum_rad, limits.maximum_rad), now};
            break;
        }
    }
    return result;
}

const std::optional<SafetyFault>& SafetyMonitor::fault() const {
    return latched_fault_;
}

void SafetyMonitor::latch_fault(const SafetyFault& fault) {
    if (!latched_fault_) {
        latched_fault_ = fault;
    }
}

void SafetyMonitor::clear_fault() {
    latched_fault_.reset();
}

} // namespace mors::fsm
