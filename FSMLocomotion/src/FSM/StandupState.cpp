#include "FSM/StandupState.hpp"
#include "RobotFSM.hpp"

namespace mors::fsm {

StandupState::StandupState(RobotFSM& fsm, StandupConfig config)
    : State(fsm, "StandupState"), action_(fsm.controller, config) {}

void StandupState::on_enter() {
    State::on_enter();
    action_.reset();
    finished_ = false;
}

std::optional<Event> StandupState::on_update(const LocomotionSetpoints& setpoints) {
    const auto result = action_.step(setpoints, fsm.status.safety.joint_fault);
    if (const auto* fault = std::get_if<SafetyFault>(&result)) {
        fsm.enter_failure(*fault);
        return std::nullopt;
    }
    finished_ = std::get<StandupAction::Progress>(result) == StandupAction::Progress::finished;
    return finished_ ? std::optional{Event::START_STANDING} : std::nullopt;
}

bool StandupState::can_exit(StateID next_state) {
    return next_state == StateID::STANDING && finished_;
}

} // namespace mors::fsm
