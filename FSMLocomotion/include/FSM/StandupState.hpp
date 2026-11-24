#pragma once

#include "State.hpp"
#include "actions/standup.hpp"

namespace mors::fsm {

class StandupState : public State {
public:
    explicit StandupState(RobotFSM& fsm, StandupConfig config = {});
    void on_enter() override;
    std::optional<Event> on_update(const LocomotionSetpoints& setpoints) override;
    bool can_exit(StateID next_state) override;

private:
    StandupAction action_;
    bool finished_ = false;
};

} // namespace mors::fsm
