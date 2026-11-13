#include "FSM/ActionState.hpp"

namespace mors::fsm {

ActionState::ActionState(RobotFSM& fsm) : State(fsm, "ActionState") {}

} // namespace mors::fsm
