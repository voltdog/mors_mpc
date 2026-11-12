#include "FSM/LaydownState.hpp"

namespace mors::fsm {

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

} // namespace mors::fsm
