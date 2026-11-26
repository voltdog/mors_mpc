#pragma once

#include "State.hpp"
#include "actions/switch_kin_sch.hpp"

#include <utility>

namespace mors::fsm {

class ActionState : public State {
public:
    explicit ActionState(RobotFSM& fsm, SwitchKinSchConfig config = {});
    std::optional<std::string> prepare(KneeScheme scheme);
    bool take_prepared() { return std::exchange(prepared_, false); }
    void on_enter() override;
    std::optional<Event> on_update(const LocomotionSetpoints& setpoints) override;
    bool can_exit(StateID next_state) override;

private:
    SwitchKinSchAction action_;
    bool prepared_ = false;
    bool finished_ = false;
};

} // namespace mors::fsm
