#pragma once

#include "State.hpp"

namespace mors::fsm {

class StandingState : public State {
public:
    explicit StandingState(RobotFSM& fsm);
};

} // namespace mors::fsm
