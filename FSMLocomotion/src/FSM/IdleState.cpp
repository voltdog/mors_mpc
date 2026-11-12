#include "FSM/IdleState.hpp"

#include "RobotFSM.hpp"

namespace mors::fsm {

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
    return next_state == StateID::STANDUP && !fsm.controller.enable
        && fsm.readiness_checker.is_ready(fsm.status);
}

} // namespace mors::fsm
