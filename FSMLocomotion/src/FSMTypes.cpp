#include "FSMTypes.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

namespace mors::fsm {

std::string_view state_name(StateID state) {
    switch (state) {
    case StateID::IDLE: return "IDLE";
    case StateID::STANDUP: return "STANDUP";
    case StateID::STANDING: return "STANDING";
    case StateID::LOCOMOTION: return "LOCOMOTION";
    case StateID::LAYDOWN: return "LAYDOWN";
    case StateID::ACTION: return "ACTION";
    case StateID::EXTERNAL: return "EXTERNAL";
    case StateID::FAILURE: return "FAILURE";
    }
    return "UNKNOWN";
}

double monotonic_seconds() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

JointLimits::JointLimits(double minimum_rad, double maximum_rad)
    : minimum_rad(minimum_rad), maximum_rad(maximum_rad) {
    if (!std::isfinite(minimum_rad) || !std::isfinite(maximum_rad)
        || minimum_rad >= maximum_rad) {
        throw std::invalid_argument("Invalid joint limits");
    }
}

} // namespace mors::fsm
