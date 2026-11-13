#pragma once

#include "State.hpp"

namespace mors::fsm {

class ExternalState : public State {
public:
    explicit ExternalState(RobotFSM& fsm);
};

} // namespace mors::fsm
