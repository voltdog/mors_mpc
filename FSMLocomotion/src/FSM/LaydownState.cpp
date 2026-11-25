#include "FSM/LaydownState.hpp"
#include "RobotFSM.hpp"

namespace mors::fsm {

LaydownState::LaydownState(RobotFSM& fsm, LaydownConfig config)
    : State(fsm, "LaydownState"), action_(fsm.controller, config) {}

void LaydownState::on_enter() {
    State::on_enter();
    action_.reset();
    finished_ = false;
}

std::optional<Event> LaydownState::on_update(const LocomotionSetpoints&) {
    const auto result = action_.step(monotonic_seconds());
    if (const auto* fault = std::get_if<SafetyFault>(&result)) {
        fsm.enter_failure(*fault);
        return std::nullopt;
    }
    finished_ = std::get<LaydownAction::Progress>(result) == LaydownAction::Progress::finished;
    return finished_ ? std::optional{Event::START_IDLE} : std::nullopt;
}

bool LaydownState::can_exit(StateID next_state) {
    return next_state == StateID::IDLE && finished_;
}

} // namespace mors::fsm
