#include "FSM/State.hpp"

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

} // namespace mors::fsm
