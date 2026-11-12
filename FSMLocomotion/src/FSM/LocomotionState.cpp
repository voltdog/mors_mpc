#include "FSM/LocomotionState.hpp"

#include "RobotFSM.hpp"

#include <algorithm>
#include <cmath>

namespace mors::fsm {

LocomotionState::LocomotionState(RobotFSM& fsm) : State(fsm, "LocomotionState") {}

void LocomotionState::on_enter() {
    State::on_enter();
    stable_since_.reset();
    stop_requested_ = false;
}

std::optional<Event> LocomotionState::on_update() {
    // Перед переходом в другую устойчивую цель нужно остановиться в STANDING.
    stop_requested_ = fsm.target_state && *fsm.target_state != StateID::LOCOMOTION;
    if (stop_requested_) {
        fsm.controller.set_velocity(0.0, 0.0, 0.0);
    } else {
        // Отмена остановки прерывает накопление времени устойчивости.
        stable_since_.reset();
    }
    // Контроллер продолжает работу до завершения остановки.
    fsm.controller.step();
    return std::nullopt;
}

bool LocomotionState::can_exit(StateID next_state) {
    if (next_state != StateID::STANDING || !stop_requested_) {
        stable_since_.reset();
        return false;
    }

    const auto& s = fsm.status;
    const double linear_speed = std::hypot(s.vx, s.vy);
    const bool velocity_ok = std::isfinite(linear_speed) && std::isfinite(s.yaw_rate)
        && linear_speed < LINEAR_VEL_THRESHOLD && std::abs(s.yaw_rate) < ANGULAR_VEL_THRESHOLD;
    const bool contacts_ok = s.contact_sensors_ready
        && std::ranges::all_of(s.contacts, [](bool contact) { return contact; })
        && s.gait_stopped;
    if (!(velocity_ok && contacts_ok)) {
        stable_since_.reset();
        return false;
    }

    const double now = monotonic_seconds();
    if (!stable_since_) {
        stable_since_ = now;
    }
    return now - *stable_since_ >= STABLE_TIME;
}

void LocomotionState::on_exit() {
    stable_since_.reset();
    stop_requested_ = false;
    State::on_exit();
}

} // namespace mors::fsm
