#pragma once

#include "State.hpp"

namespace mors::fsm {

class IdleState : public State {
public:
    explicit IdleState(RobotFSM& fsm);
    void on_enter() override;
    bool can_exit(StateID next_state) override;
};

} // namespace mors::fsm
