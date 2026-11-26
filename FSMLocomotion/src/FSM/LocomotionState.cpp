#include "FSM/LocomotionState.hpp"

#include "LocomotionData.hpp"
#include "RobotFSM.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mors::fsm {

LocomotionState::LocomotionState(RobotFSM& fsm, GaitStopConfig gait_stop,
                                 LocomotionAccelerationConfig acceleration)
    : State(fsm, "LocomotionState"), gait_stop_(gait_stop), acceleration_(acceleration) {
    for (double value : {gait_stop.linear_mps, gait_stop.yaw_radps, gait_stop.hold_s,
                         gait_stop.linear_decel_mps2, gait_stop.yaw_decel_radps2}) {
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument("fsm_locomotion.gait_stop values must be finite and positive");
        }
    }
    for (double value : {acceleration.linear_mps2, acceleration.yaw_radps2}) {
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument("fsm_locomotion.acceleration_limits values must be finite and positive");
        }
    }
}

void LocomotionState::on_enter() {
    State::on_enter();
    applied_mode_.reset();
    stable_since_.reset();
    last_command_.reset();
    last_update_s_ = monotonic_seconds();
    speed_average_.reset();
    stop_requested_ = false;
}

void LocomotionState::update_speed_average() {
    const auto& s = fsm.status;
    const double speed = std::hypot(s.vx, s.vy);
    const auto received = s.robot_state_received_at_s;
    if (s.safety.telemetry_fault || !std::isfinite(s.yaw_rate)) {
        speed_average_.reset(received);
        stable_since_.reset();
        return;
    }
    speed_average_.update(speed, received);
    if (!speed_average_.value()) stable_since_.reset();
}

bool LocomotionState::velocity_is_low() const {
    const auto& s = fsm.status;
    const double speed = std::hypot(s.vx, s.vy);
    const auto average = speed_average_.value();
    return average && *average < gait_stop_.linear_mps
        && std::isfinite(speed) && speed < gait_stop_.linear_mps
        && std::isfinite(s.yaw_rate) && std::abs(s.yaw_rate) < gait_stop_.yaw_radps;
}

std::optional<Event> LocomotionState::on_update(const LocomotionSetpoints& setpoints) {
    const double now = monotonic_seconds();
    const double dt = std::max(0.0, now - last_update_s_);
    last_update_s_ = now;
    update_speed_average();
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
        if (last_command_) {
            auto& command = *last_command_;
            if (recovering) {
                command.lin_vel.setZero();
                command.ang_vel.setZero();
            }
            const double speed = command.lin_vel.head<2>().norm();
            if (speed > 0.0) {
                const double next_speed = std::max(0.0, speed - gait_stop_.linear_decel_mps2 * dt);
                command.lin_vel.head<2>() *= next_speed / speed;
            }
            const double yaw = command.ang_vel.z();
            const double next_yaw = std::max(0.0, std::abs(yaw) - gait_stop_.yaw_decel_radps2 * dt);
            command.ang_vel.z() = std::copysign(next_yaw, yaw);
            command.lin_vel.z() = 0.0;
            command.ang_vel.head<2>().setZero();
        }
        const bool command_stopped = !last_command_
            || (last_command_->lin_vel.head<2>().isZero(0.0) && last_command_->ang_vel.z() == 0.0);
        if (command_stopped && velocity_is_low()) {
            fsm.controller.request_gait_stop();
        } else {
            // Пока корпус движется, походка должна позволять тормозящие шаги.
            stable_since_.reset();
            if (last_command_)
                fsm.controller.set_robot_cmd(*last_command_);
            else
                fsm.controller.hold_measured_pose();
        }
    } else {
        // Отмена остановки прерывает накопление времени устойчивости.
        stable_since_.reset();
        fsm.controller.set_gait_params(setpoints.t_st, setpoints.t_sw, setpoints.gait_type,
                                       setpoints.stride_height);
        fsm.controller.set_active_legs(setpoints.active_legs);
        fsm.controller.set_adaptation_type(setpoints.adaptation_type);
        if (setpoints.robot_cmd_received_at_s) {
            auto command = setpoints.robot_cmd;
            if (recovering) {
                command.lin_vel.setZero();
                command.ang_vel.setZero();
            } else {
                // Ограничиваем изменение вектора XY, включая смену направления.
                const auto previous = last_command_.value_or(RobotData{});
                const Eigen::Vector2d delta = command.lin_vel.head<2>() - previous.lin_vel.head<2>();
                const double distance = delta.norm();
                const double max_delta = acceleration_.linear_mps2 * dt;
                if (distance > max_delta) {
                    command.lin_vel.head<2>() = previous.lin_vel.head<2>() + delta * (max_delta / distance);
                }
                const double max_yaw_delta = acceleration_.yaw_radps2 * dt;
                command.ang_vel.z() = previous.ang_vel.z()
                    + std::clamp(command.ang_vel.z() - previous.ang_vel.z(), -max_yaw_delta, max_yaw_delta);
            }
            fsm.controller.set_robot_cmd(command);
            last_command_ = command;
        } else {
            fsm.controller.hold_measured_pose();
            last_command_.reset();
        }
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
    const bool contacts_ok = s.safety.contact_sensors_ready
        && std::ranges::all_of(s.contacts, [](bool contact) { return contact; })
        && s.gait_stopped;
    if (!(velocity_is_low() && contacts_ok)) {
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
    last_command_.reset();
    stop_requested_ = false;
    fsm.status.gait_stopped = false;
    State::on_exit();
}

} // namespace mors::fsm
