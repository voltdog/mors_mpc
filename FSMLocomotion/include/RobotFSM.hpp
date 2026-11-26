#pragma once

#include "FSM/FSMTypes.hpp"
#include "FSM/State.hpp"
#include "LocomotionControlBase.hpp"
#include "ReadinessChecker.hpp"
#include "SafetyMonitor.hpp"
#include "actions/laydown.hpp"
#include "actions/standup.hpp"
#include "actions/switch_kin_sch.hpp"

#include <map>
#include <memory>

namespace mors::fsm {

// Цель -> BFS -> следующий переход -> can_exit -> on_exit/on_enter.
// update() вызывается из внешнего главного цикла; собственных потоков нет.
class RobotFSM {
private:
    // Объекты по умолчанию создаются только при отсутствии внешних.
    std::unique_ptr<RobotStatus> owned_status_;
    std::unique_ptr<LocomotionControlBase> owned_controller_;
    std::unique_ptr<ReadinessChecker> owned_readiness_checker_;

public:
    // Внешние объекты не принадлежат FSM и должны существовать дольше неё.
    explicit RobotFSM(SafetyMonitor& safety_monitor, RobotStatus* status = nullptr,
                      LocomotionControlBase* controller = nullptr,
                      ReadinessChecker* readiness_checker = nullptr, GaitStopConfig gait_stop = {},
                      StandupConfig standup = {}, LaydownConfig laydown = {},
                      LocomotionAccelerationConfig acceleration = {}, SwitchKinSchConfig switch_kin_sch = {});
    RobotFSM(const RobotFSM&) = delete;
    RobotFSM& operator=(const RobotFSM&) = delete;
    RobotFSM(RobotFSM&&) = delete;
    RobotFSM& operator=(RobotFSM&&) = delete;

    std::optional<std::vector<StateID>> find_path(StateID start, StateID goal) const;
    bool request_state(StateID target);
    std::optional<std::string> start_switch_kinematics(KneeScheme scheme);
    bool handle_event(Event event);
    void enforce_safety();
    void update(const LocomotionSetpoints& setpoints);
    // Возвращает причину отказа или пустое значение после успешного сброса.
    std::optional<std::string> try_clear_fault();

    RobotStatus& status;
    LocomotionControlBase& controller;
    SafetyMonitor& safety_monitor;
    ReadinessChecker& readiness_checker;
    std::optional<StateID> target_state;
    StateID state_id = StateID::IDLE;

private:
    friend class LaydownState;
    friend class StandupState;
    friend class ActionState;
    void enter_failure(const SafetyFault& fault);
    bool transition_to(StateID next_state);

    // По одному экземпляру каждого состояния, включая переходные состояния.
    std::map<StateID, std::unique_ptr<State>> states_;
    std::optional<std::pair<StateID, StateID>> last_blocked_;
};

} // namespace mors::fsm
