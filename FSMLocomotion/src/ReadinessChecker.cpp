#include "ReadinessChecker.hpp"

#include <cmath>
#include <stdexcept>

namespace mors::fsm {

ReadinessChecker::ReadinessChecker(double stable_time_s)
    : stable_time_s_(stable_time_s) {
    if (!std::isfinite(stable_time_s_) || stable_time_s_ < 0.0) {
        throw std::invalid_argument("stable_time_s must be non-negative");
    }
}

void ReadinessChecker::reset() {
    stable_since_.reset();
}

bool ReadinessChecker::is_ready(const RobotStatus& status, double now) {
    if (status.safety.telemetry_fault || !status.safety.contact_sensors_ready) {
        reset();
        return false;
    }
    if (!stable_since_) {
        stable_since_ = now;
    }
    return now - *stable_since_ >= stable_time_s_;
}

} // namespace mors::fsm
