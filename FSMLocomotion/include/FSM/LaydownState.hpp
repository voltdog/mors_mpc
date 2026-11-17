#pragma once

#include "State.hpp"

namespace mors::fsm {

class LaydownState : public State {
public:
    explicit LaydownState(RobotFSM& fsm);
    void on_enter() override;
    std::optional<Event> on_update(const LocomotionSetpoints& setpoints) override;

private:
    int ticks_ = 0;
};

} // namespace mors::fsm
