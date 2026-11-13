#pragma once

#include "State.hpp"

namespace mors::fsm {

class IdleState : public State {
public:
    explicit IdleState(RobotFSM& fsm);
    void on_enter() override;
    std::optional<Event> on_update() override;
    bool can_exit(StateID next_state) override;
};

} // namespace mors::fsm
