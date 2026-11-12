#pragma once

#include "State.hpp"

namespace mors::fsm {

class LocomotionState : public State {
public:
    static constexpr double LINEAR_VEL_THRESHOLD = 0.05;  // м/с.
    static constexpr double ANGULAR_VEL_THRESHOLD = 0.10; // рад/с.
    static constexpr double STABLE_TIME = 0.30;          // с.

    explicit LocomotionState(RobotFSM& fsm);
    void on_enter() override;
    std::optional<Event> on_update() override;
    bool can_exit(StateID next_state) override;
    void on_exit() override;

private:
    std::optional<double> stable_since_;
    bool stop_requested_ = false;
};

} // namespace mors::fsm
