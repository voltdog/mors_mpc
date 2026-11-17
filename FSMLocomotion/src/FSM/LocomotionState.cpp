#include "FSM/LocomotionState.hpp"

#include "LocomotionData.hpp"
#include "RobotFSM.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mors::fsm {

LocomotionState::LocomotionState(RobotFSM& fsm, GaitStopConfig gait_stop)
    : State(fsm, "LocomotionState"), gait_stop_(gait_stop) {
    for (double value : {gait_stop.linear_mps, gait_stop.yaw_radps, gait_stop.hold_s}) {
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument("fsm_locomotion.gait_stop values must be finite and positive");
        }
    }
}

void LocomotionState::on_enter() {
    State::on_enter();
    applied_mode_.reset();
    stable_since_.reset();
    stop_requested_ = false;
}

std::optional<Event> LocomotionState::on_update(const LocomotionSetpoints& setpoints) {
    const bool recovering = setpoints.mode == LocomotionMode::RECOVERY;
    const auto mode = recovering ? LocomotionControlBase::recovery_mode : LocomotionControlBase::walking_mode;
    if (applied_mode_ != mode) {
        // Контроллер должен обработать уставку в уже выбранном режиме.
        fsm.controller.set_locomotion_mode(mode);
        applied_mode_ = mode;
    }

    // Перед переходом в другую устойчивую цель нужно остановиться в STANDING.
    stop_requested_ = fsm.target_state && *fsm.target_state != StateID::LOCOMOTION;

    if (stop_requested_) {
        // Во время остановки входящий ROBOT_CMD не может вернуть ненулевую скорость.
        fsm.controller.request_gait_stop();
    } else {
        // Отмена остановки прерывает накопление времени устойчивости.
        stable_since_.reset();
        auto command = setpoints.robot_cmd;
        if (recovering) {
            command.lin_vel.setZero();
            command.ang_vel.setZero();
        } else {
            fsm.controller.set_gait_params(setpoints.t_st, setpoints.t_sw, setpoints.gait_type,
                                           setpoints.stride_height);
        }
        fsm.controller.set_active_legs(setpoints.active_legs);
        if (!recovering) {
            fsm.controller.set_adaptation_type(setpoints.adaptation_type);
        }
        fsm.controller.set_robot_cmd(command);
    }

    fsm.status.gait_stopped = fsm.controller.gait_stopped();
    return std::nullopt;
}

bool LocomotionState::can_exit(StateID next_state) {
    if (next_state != StateID::STANDING || !stop_requested_) {
        stable_since_.reset();
        return false;
    }

    const auto& s = fsm.status;
    const double linear_speed = std::hypot(s.vx, s.vy);
    const bool velocity_ok = std::isfinite(linear_speed) && std::isfinite(s.yaw_rate)
        && linear_speed < gait_stop_.linear_mps && std::abs(s.yaw_rate) < gait_stop_.yaw_radps;
    const bool contacts_ok = s.safety.contact_sensors_ready
        && std::ranges::all_of(s.contacts, [](bool contact) { return contact; })
        && s.gait_stopped;
    if (!(velocity_ok && contacts_ok)) {
        stable_since_.reset();
        return false;
    }

    const double now = monotonic_seconds();
    if (!stable_since_) {
        stable_since_ = now;
    }
    return now - *stable_since_ >= gait_stop_.hold_s;
}

void LocomotionState::on_exit() {
    stable_since_.reset();
    stop_requested_ = false;
    fsm.status.gait_stopped = false;
    State::on_exit();
}

} // namespace mors::fsm
