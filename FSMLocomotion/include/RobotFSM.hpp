#pragma once

#include "FSMTypes.hpp"
#include "LocomotionControl.hpp"
#include "ReadinessChecker.hpp"
#include "SafetyMonitor.hpp"
#include "States.hpp"

#include <map>
#include <memory>

namespace mors::fsm {

// Цель -> BFS -> следующий переход -> can_exit -> on_exit/on_enter.
// update() вызывается из внешнего главного цикла; собственных потоков нет.
class RobotFSM {
private:
    // Объекты по умолчанию создаются только при отсутствии внешних.
    std::unique_ptr<RobotStatus> owned_status_;
    std::unique_ptr<LocomotionControl> owned_controller_;
    std::unique_ptr<ReadinessChecker> owned_readiness_checker_;

public:
    // Внешние объекты не принадлежат FSM и должны существовать дольше неё.
    explicit RobotFSM(SafetyMonitor& safety_monitor, RobotStatus* status = nullptr,
                      LocomotionControl* controller = nullptr,
                      ReadinessChecker* readiness_checker = nullptr);
    RobotFSM(const RobotFSM&) = delete;
    RobotFSM& operator=(const RobotFSM&) = delete;
    RobotFSM(RobotFSM&&) = delete;
    RobotFSM& operator=(RobotFSM&&) = delete;

    std::optional<std::vector<StateID>> find_path(StateID start, StateID goal) const;
    bool request_state(StateID target);
    bool handle_event(Event event);
    void update();
    bool try_clear_fault(bool operator_acknowledged);
    bool enable_actuator_commands();

    RobotStatus& status;
    LocomotionControl& controller;
    SafetyMonitor& safety_monitor;
    ReadinessChecker& readiness_checker;
    std::optional<StateID> target_state;
    StateID state_id = StateID::IDLE;

private:
    bool transition_to(StateID next_state);

    // По одному экземпляру каждого состояния, включая переходные состояния.
    std::map<StateID, std::unique_ptr<State>> states_;
    std::optional<std::pair<StateID, StateID>> last_blocked_;
};

} // namespace mors::fsm
