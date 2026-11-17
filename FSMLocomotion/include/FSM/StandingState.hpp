#pragma once

#include "State.hpp"

namespace mors::fsm {

class StandingState : public State {
public:
    explicit StandingState(RobotFSM& fsm);
    void on_enter() override;
    std::optional<Event> on_update(const LocomotionSetpoints& setpoints) override;
};

} // namespace mors::fsm
