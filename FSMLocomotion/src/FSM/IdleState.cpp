#include "FSM/IdleState.hpp"

#include "RobotFSM.hpp"

namespace mors::fsm {

IdleState::IdleState(RobotFSM& fsm) : State(fsm, "IdleState") {}

void IdleState::on_enter() {
    State::on_enter();
    fsm.controller.disable_outputs();
    fsm.readiness_checker.reset();
}

bool IdleState::can_exit(StateID next_state) {
    return next_state == StateID::STANDUP && fsm.status.ready_to_move;
}

} // namespace mors::fsm
