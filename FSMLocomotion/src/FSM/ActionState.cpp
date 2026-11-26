#include "FSM/ActionState.hpp"
#include "RobotFSM.hpp"

namespace mors::fsm {

ActionState::ActionState(RobotFSM& fsm, SwitchKinSchConfig config)
    : State(fsm, "ActionState"), action_(fsm.controller, config) {}

std::optional<std::string> ActionState::prepare(KneeScheme scheme) {
    prepared_ = false;
    const auto state = fsm.controller.robot_state();
    if (!state) return "Missing measured joint angles";
    auto result = plan_switch_kin_sch(state->joints.position, scheme);
    if (const auto* reason = std::get_if<std::string>(&result)) return *reason;
    const auto& plan = std::get<SwitchKinSchPlan>(result);
    const auto diagnostics = fsm.safety_monitor.evaluate(state, monotonic_seconds());
    if (diagnostics.telemetry_fault) return diagnostics.telemetry_fault->description;
    if (diagnostics.joint_fault) return diagnostics.joint_fault->description;
    auto target = *state;
    target.joints.position = plan.target;
    if (const auto fault = fsm.safety_monitor.evaluate(target, monotonic_seconds()).joint_fault) {
        return "Target joint limit: " + fault->description;
    }
    action_.reset(plan);
    prepared_ = true;
    return std::nullopt;
}

void ActionState::on_enter() {
    State::on_enter();
    prepared_ = false;
    finished_ = false;
}

std::optional<Event> ActionState::on_update(const LocomotionSetpoints& setpoints) {
    const auto result = action_.step(setpoints, fsm.safety_monitor);
    if (const auto* fault = std::get_if<SafetyFault>(&result)) {
        fsm.enter_failure(*fault);
        return std::nullopt;
    }
    finished_ = std::get<SwitchKinSchAction::Progress>(result) == SwitchKinSchAction::Progress::finished;
    return finished_ ? std::optional{Event::START_STANDING} : std::nullopt;
}

bool ActionState::can_exit(StateID next_state) {
    return next_state == StateID::STANDING && finished_;
}

} // namespace mors::fsm
