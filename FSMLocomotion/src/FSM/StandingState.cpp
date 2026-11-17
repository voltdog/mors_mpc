#include "FSM/StandingState.hpp"

#include "RobotFSM.hpp"

namespace mors::fsm {

StandingState::StandingState(RobotFSM& fsm) : State(fsm, "StandingState") {}

void StandingState::on_enter() {
    State::on_enter();
    fsm.controller.set_locomotion_mode(LocomotionControlBase::standing_mode);
}

std::optional<Event> StandingState::on_update(const LocomotionSetpoints& setpoints) {
    RobotData command = setpoints.robot_cmd;
    command.lin_vel.setZero();
    command.ang_vel.setZero();
    fsm.controller.set_robot_cmd(command);
    return std::nullopt;
}

} // namespace mors::fsm
