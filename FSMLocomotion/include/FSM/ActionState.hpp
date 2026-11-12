#pragma once

#include "State.hpp"

namespace mors::fsm {

class ActionState : public State {
public:
    explicit ActionState(RobotFSM& fsm);
};

} // namespace mors::fsm
