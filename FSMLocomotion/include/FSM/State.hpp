#pragma once

#include "FSMTypes.hpp"

#include <optional>
#include <string_view>

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

} // namespace mors::fsm
