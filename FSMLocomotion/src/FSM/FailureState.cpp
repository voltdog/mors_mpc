#include "FSM/FailureState.hpp"

#include "RobotFSM.hpp"

#include <iostream>

namespace mors::fsm {

FailureState::FailureState(RobotFSM& fsm) : State(fsm, "FailureState") {}

void FailureState::on_enter() {
    State::on_enter();
    fsm.status.ready_to_move = false;
    fsm.status.gait_stopped = false;
    fsm.readiness_checker.reset();
    std::cout << "FAILURE: движение запрещено\n";
}

} // namespace mors::fsm
