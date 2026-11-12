#include "FSM/ExternalState.hpp"

namespace mors::fsm {

ExternalState::ExternalState(RobotFSM& fsm) : State(fsm, "ExternalState") {}

} // namespace mors::fsm
