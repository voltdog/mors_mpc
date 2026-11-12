#include "FSM/StandupState.hpp"

namespace mors::fsm {

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

} // namespace mors::fsm
