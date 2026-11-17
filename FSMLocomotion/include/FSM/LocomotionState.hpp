#pragma once

#include "State.hpp"

#include <cstdint>

namespace mors::fsm {

class LocomotionState : public State {
public:
    explicit LocomotionState(RobotFSM& fsm, GaitStopConfig gait_stop = {});
    void on_enter() override;
    std::optional<Event> on_update(const LocomotionSetpoints& setpoints) override;
    bool can_exit(StateID next_state) override;
    void on_exit() override;

private:
    const GaitStopConfig gait_stop_;
    std::optional<std::uint8_t> applied_mode_;
    std::optional<double> stable_since_;
    bool stop_requested_ = false;
};

} // namespace mors::fsm
