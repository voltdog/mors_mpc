#pragma once

#include "FSMTypes.hpp"

namespace mors::fsm {

class RobotFSM;

class State {
public:
    State(RobotFSM& fsm, std::string_view name);
    virtual ~State() = default;

    virtual void on_enter();
    virtual std::optional<Event> on_update();
    virtual bool can_exit(StateID next_state);
    virtual void on_exit();
    virtual void on_abort();

protected:
    RobotFSM& fsm;

private:
    std::string_view name_;
};

class IdleState : public State {
public:
    explicit IdleState(RobotFSM& fsm);
    void on_enter() override;
    std::optional<Event> on_update() override;
    bool can_exit(StateID next_state) override;
};

class StandupState : public State {
public:
    explicit StandupState(RobotFSM& fsm);
    void on_enter() override;
    std::optional<Event> on_update() override;

private:
    int ticks_ = 0;
};

class StandingState : public State {
public:
    explicit StandingState(RobotFSM& fsm);
};

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

class LaydownState : public State {
public:
    explicit LaydownState(RobotFSM& fsm);
    void on_enter() override;
    std::optional<Event> on_update() override;

private:
    int ticks_ = 0;
};

class ActionState : public State {
public:
    explicit ActionState(RobotFSM& fsm);
};

class ExternalState : public State {
public:
    explicit ExternalState(RobotFSM& fsm);
};

class FailureState : public State {
public:
    explicit FailureState(RobotFSM& fsm);
    void on_enter() override;
};

} // namespace mors::fsm
