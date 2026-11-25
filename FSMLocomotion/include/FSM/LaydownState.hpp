#pragma once

#include "State.hpp"
#include "actions/laydown.hpp"

namespace mors::fsm {

class LaydownState : public State {
public:
    explicit LaydownState(RobotFSM& fsm, LaydownConfig config = {});
    void on_enter() override;
    std::optional<Event> on_update(const LocomotionSetpoints& setpoints) override;
    bool can_exit(StateID next_state) override;

private:
    LaydownAction action_;
    bool finished_ = false;
};

} // namespace mors::fsm
