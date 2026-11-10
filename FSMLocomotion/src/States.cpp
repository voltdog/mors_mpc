#include "States.hpp"

#include "RobotFSM.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace mors::fsm {

State::State(RobotFSM& fsm, std::string_view name) : fsm(fsm), name_(name) {}

void State::on_enter() {
    std::cout << "Enter " << name_ << '\n';
}

std::optional<Event> State::on_update() {
    return std::nullopt;
}

bool State::can_exit(StateID) {
    return true;
}

void State::on_exit() {
    std::cout << "Exit  " << name_ << '\n';
}

void State::on_abort() {
    // Аварийный выход не отправляет команды управления.
    std::cout << "Abort " << name_ << '\n';
}

IdleState::IdleState(RobotFSM& fsm) : State(fsm, "IdleState") {}

void IdleState::on_enter() {
    State::on_enter();
    fsm.readiness_checker.reset();
}

std::optional<Event> IdleState::on_update() {
    // Фоновая диагностика работает и без запроса смены состояния.
    fsm.readiness_checker.is_ready(fsm.status);
    return std::nullopt;
}

bool IdleState::can_exit(StateID next_state) {
    return next_state == StateID::STANDUP && !fsm.controller.outputs_inhibited
        && fsm.readiness_checker.is_ready(fsm.status);
}

StandupState::StandupState(RobotFSM& fsm) : State(fsm, "StandupState") {}

void StandupState::on_enter() {
    State::on_enter();
    ticks_ = 0;
}

std::optional<Event> StandupState::on_update() {
    // Заглушка завершения вставания после трёх обновлений.
    ++ticks_;
    if (ticks_ >= 3) {
        return Event::START_STANDING;
    }
    return std::nullopt;
}

StandingState::StandingState(RobotFSM& fsm) : State(fsm, "StandingState") {}

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

LaydownState::LaydownState(RobotFSM& fsm) : State(fsm, "LaydownState") {}

void LaydownState::on_enter() {
    State::on_enter();
    ticks_ = 0;
}

std::optional<Event> LaydownState::on_update() {
    // Заглушка завершения укладывания после трёх обновлений.
    ++ticks_;
    if (ticks_ >= 3) {
        return Event::START_IDLE;
    }
    return std::nullopt;
}

ActionState::ActionState(RobotFSM& fsm) : State(fsm, "ActionState") {}

ExternalState::ExternalState(RobotFSM& fsm) : State(fsm, "ExternalState") {}

FailureState::FailureState(RobotFSM& fsm) : State(fsm, "FailureState") {}

void FailureState::on_enter() {
    State::on_enter();
    // Повторная блокировка идемпотентна, как и в Python-прототипе.
    fsm.controller.emergency_stop();
    std::cout << "FAILURE: движение запрещено\n";
}

} // namespace mors::fsm
