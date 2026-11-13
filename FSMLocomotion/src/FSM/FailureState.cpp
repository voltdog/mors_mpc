#include "FSM/FailureState.hpp"

#include "RobotFSM.hpp"

#include <iostream>

namespace mors::fsm {

FailureState::FailureState(RobotFSM& fsm) : State(fsm, "FailureState") {}

void FailureState::on_enter() {
    State::on_enter();
    // Повторная блокировка идемпотентна, как и в Python-прототипе.
    fsm.controller.disable_outputs();
    std::cout << "FAILURE: движение запрещено\n";
}

} // namespace mors::fsm
