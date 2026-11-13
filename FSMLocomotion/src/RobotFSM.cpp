#include "RobotFSM.hpp"

#include "FSM/ActionState.hpp"
#include "FSM/ExternalState.hpp"
#include "FSM/FailureState.hpp"
#include "FSM/IdleState.hpp"
#include "FSM/LaydownState.hpp"
#include "FSM/LocomotionState.hpp"
#include "FSM/StandingState.hpp"
#include "FSM/StandupState.hpp"

#include <algorithm>
#include <array>
#include <iostream>
#include <queue>
#include <set>
#include <stdexcept>
#include <utility>

namespace mors::fsm {
namespace {

struct Edge {
    StateID from;
    StateID to;
    Event event;
};

// Порядок рёбер совпадает с порядком вставки в Python-словарь EDGES.
constexpr std::array edges{
    Edge{StateID::IDLE, StateID::STANDUP, Event::START_STANDUP},
    Edge{StateID::STANDUP, StateID::STANDING, Event::START_STANDING},
    Edge{StateID::STANDING, StateID::LAYDOWN, Event::START_LAYDOWN},
    Edge{StateID::LAYDOWN, StateID::IDLE, Event::START_IDLE},
    Edge{StateID::STANDING, StateID::LOCOMOTION, Event::START_LOCOMOTION},
    Edge{StateID::LOCOMOTION, StateID::STANDING, Event::START_STANDING},
    Edge{StateID::STANDING, StateID::ACTION, Event::START_ACTION},
    Edge{StateID::ACTION, StateID::STANDING, Event::START_STANDING},
    Edge{StateID::STANDING, StateID::EXTERNAL, Event::START_EXTERNAL},
    Edge{StateID::EXTERNAL, StateID::STANDING, Event::START_STANDING},
};

constexpr std::array transient{StateID::STANDUP, StateID::LAYDOWN};
constexpr std::array targets{
    StateID::IDLE, StateID::STANDING, StateID::LOCOMOTION, StateID::ACTION, StateID::EXTERNAL,
};

} // namespace

RobotFSM::RobotFSM(SafetyMonitor& safety_monitor, RobotStatus* status,
                   LocomotionControl* controller, ReadinessChecker* readiness_checker)
    : owned_status_(status ? nullptr : std::make_unique<RobotStatus>()),
      owned_controller_(controller ? nullptr : std::make_unique<LocomotionControl>()),
      owned_readiness_checker_(readiness_checker ? nullptr
                                               : std::make_unique<ReadinessChecker>(safety_monitor)),
      status(status ? *status : *owned_status_),
      controller(controller ? *controller : *owned_controller_),
      safety_monitor(safety_monitor),
      readiness_checker(readiness_checker ? *readiness_checker : *owned_readiness_checker_) {
    states_.emplace(StateID::IDLE, std::make_unique<IdleState>(*this));
    states_.emplace(StateID::STANDUP, std::make_unique<StandupState>(*this));
    states_.emplace(StateID::STANDING, std::make_unique<StandingState>(*this));
    states_.emplace(StateID::LOCOMOTION, std::make_unique<LocomotionState>(*this));
    states_.emplace(StateID::LAYDOWN, std::make_unique<LaydownState>(*this));
    states_.emplace(StateID::ACTION, std::make_unique<ActionState>(*this));
    states_.emplace(StateID::EXTERNAL, std::make_unique<ExternalState>(*this));
    states_.emplace(StateID::FAILURE, std::make_unique<FailureState>(*this));
    states_.at(state_id)->on_enter();
}

std::optional<std::vector<StateID>> RobotFSM::find_path(StateID start, StateID goal) const {
    using Path = std::vector<StateID>;
    std::queue<std::pair<StateID, Path>> queue;
    queue.emplace(start, Path{start});
    std::set<StateID> visited{start};

    while (!queue.empty()) {
        auto [current, path] = std::move(queue.front());
        queue.pop();
        if (current == goal) {
            return path;
        }
        for (const auto& edge : edges) {
            if (edge.from == current && !visited.contains(edge.to)) {
                visited.insert(edge.to);
                auto next_path = path;
                next_path.push_back(edge.to);
                queue.emplace(edge.to, std::move(next_path));
            }
        }
    }
    return std::nullopt;
}

bool RobotFSM::request_state(StateID target) {
    if (std::ranges::find(targets, target) == targets.end()) {
        throw std::invalid_argument("Недопустимое целевое состояние: "
                                    + std::string(state_name(target)));
    }
    if (state_id == StateID::FAILURE || status.estop_active || safety_monitor.fault()) {
        return false;
    }
    const auto path = find_path(state_id, target);
    if (!path) {
        return false;
    }

    target_state = target;
    std::cout << "Маршрут: ";
    for (std::size_t i = 0; i < path->size(); ++i) {
        std::cout << (i == 0 ? "" : " -> ") << state_name((*path)[i]);
    }
    std::cout << '\n';
    return true;
}

bool RobotFSM::transition_to(StateID next_state) {
    if (next_state == state_id) {
        return true;
    }

    // ESTOP обходит обычные проверки готовности и устойчивости.
    if (next_state != StateID::FAILURE && !states_.at(state_id)->can_exit(next_state)) {
        const auto edge = std::pair{state_id, next_state};
        if (last_blocked_ != edge) {
            std::cout << "Ожидание условий: " << state_name(edge.first)
                      << " -> " << state_name(edge.second) << '\n';
        }
        last_blocked_ = edge;
        return false;
    }

    last_blocked_.reset();
    if (next_state == StateID::FAILURE) {
        // Обычный on_exit() в аварии может отправить команды управления.
        states_.at(state_id)->on_abort();
    } else {
        states_.at(state_id)->on_exit();
    }
    state_id = next_state;
    states_.at(state_id)->on_enter();
    return true;
}

bool RobotFSM::handle_event(Event event) {
    if (event == Event::ESTOP) {
        target_state.reset(); // Без автоматического возобновления движения.
        controller.disable_outputs(); // Блокировка ДО обработчиков состояний.
        return transition_to(StateID::FAILURE);
    }
    if (event == Event::CLEAR_FAULT) {
        if (state_id == StateID::FAILURE && !status.estop_active && !safety_monitor.fault()) {
            target_state.reset();
            return transition_to(StateID::IDLE);
        }
        return false;
    }
    if (status.estop_active || state_id == StateID::FAILURE || safety_monitor.fault()) {
        return false;
    }
    for (const auto& edge : edges) {
        if (edge.from == state_id && edge.event == event) {
            return transition_to(edge.to);
        }
    }
    return false;
}

void RobotFSM::update() {
    // Безопасность проверяется до on_update() и выдачи команд.
    // В IDLE отсутствие телеметрии блокирует запуск, но не вызывает ESTOP.
    const bool feedback_required = state_id != StateID::IDLE && state_id != StateID::FAILURE;
    const auto& fault = safety_monitor.check(status.joint_feedback, feedback_required, state_id);
    if ((status.estop_active || fault) && state_id != StateID::FAILURE) {
        if (fault) {
            std::cout << "SAFETY: " << fault->code << ": " << fault->description << '\n';
        }
        handle_event(Event::ESTOP);
        return;
    }

    if (const auto event = states_.at(state_id)->on_update()) {
        handle_event(*event);
        return;
    }
    // Переходные состояния завершаются собственными on_update().
    if (std::ranges::find(transient, state_id) != transient.end() || !target_state
        || state_id == *target_state) {
        return;
    }
    const auto path = find_path(state_id, *target_state);
    if (path && path->size() > 1) {
        const StateID next_state = (*path)[1];
        for (const auto& edge : edges) {
            if (edge.from == state_id && edge.to == next_state) {
                handle_event(edge.event);
                break;
            }
        }
    }
}

bool RobotFSM::try_clear_fault(bool operator_acknowledged) {
    if (state_id != StateID::FAILURE) {
        return false;
    }
    if (!safety_monitor.clear_fault(status.joint_feedback, operator_acknowledged,
                                     !status.estop_active)) {
        return false;
    }
    return handle_event(Event::CLEAR_FAULT);
}

bool RobotFSM::enable_actuator_commands() {
    if (state_id != StateID::IDLE || status.estop_active || !readiness_checker.is_ready(status)) {
        return false;
    }
    controller.enable_outputs();
    return true;
}

} // namespace mors::fsm
