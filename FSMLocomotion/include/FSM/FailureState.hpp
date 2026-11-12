#pragma once

#include "State.hpp"

namespace mors::fsm {

class FailureState : public State {
public:
    explicit FailureState(RobotFSM& fsm);
    void on_enter() override;
};

} // namespace mors::fsm
