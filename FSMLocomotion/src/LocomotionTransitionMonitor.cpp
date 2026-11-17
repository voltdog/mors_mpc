#include "LocomotionTransitionMonitor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace mors::fsm {
namespace {

bool valid_planar_estimate(const RobotData* estimate) {
    return estimate && estimate->pos.head<2>().allFinite() && estimate->lin_vel.head<2>().allFinite();
}

} // namespace

LocomotionTransitionMonitor::LocomotionTransitionMonitor(LocomotionTransitionConfig config)
    : config_(config) {
    const std::array values{
        config.command_timeout_s, config.start_linear_mps, config.start_yaw_radps,
        config.stop_linear_mps, config.stop_yaw_radps, config.stop_hold_s,
        config.recovery_entry.position_m, config.recovery_entry.velocity_mps,
        config.recovery_exit_walking.position_m, config.recovery_exit_walking.velocity_mps,
        config.recovery_exit_standing.position_m, config.recovery_exit_standing.velocity_mps,
    };
    if (!std::ranges::all_of(values, [](double value) { return std::isfinite(value) && value > 0.0; })) {
        throw std::invalid_argument("fsm_locomotion thresholds and durations must be finite and positive");
    }
    if (config.stop_linear_mps >= config.start_linear_mps || config.stop_yaw_radps >= config.start_yaw_radps) {
        throw std::invalid_argument("fsm_locomotion stop thresholds must be below start thresholds");
    }
    for (const auto& exit : {config.recovery_exit_standing, config.recovery_exit_walking}) {
        if (exit.position_m >= config.recovery_entry.position_m
            || exit.velocity_mps >= config.recovery_entry.velocity_mps) {
            throw std::invalid_argument("fsm_locomotion recovery exit thresholds must be below entry thresholds");
        }
    }
}

bool LocomotionTransitionMonitor::velocity_is_fresh(const LocomotionSetpoints& setpoints, double now_s) const {
    if (!setpoints.robot_cmd_received_at_s) {
        return false;
    }
    const double age_s = now_s - *setpoints.robot_cmd_received_at_s;
    return age_s >= 0.0 && age_s <= config_.command_timeout_s
        && setpoints.robot_cmd.lin_vel.allFinite() && setpoints.robot_cmd.ang_vel.allFinite();
}

void LocomotionTransitionMonitor::capture_standing_xy(const RobotData* estimate) {
    if (!standing_xy_ && valid_planar_estimate(estimate)) {
        // XY уставки корпуса не гарантирует абсолютную координату в мире.
        standing_xy_ = estimate->pos.head<2>();
    }
}

void LocomotionTransitionMonitor::on_state_entered(StateID state, const RobotData* estimate) {
    below_stop_since_s_.reset();
    stopping_ = false;
    if (state != StateID::LOCOMOTION) {
        standing_xy_.reset();
    }
    if (state == StateID::STANDING) {
        capture_standing_xy(estimate);
    }
}

LocomotionMode LocomotionTransitionMonitor::recommend(StateID state, bool walking_allowed, bool recovering,
                                                       const RobotData* estimate, const RobotData& command,
                                                       double now_s) {
    using enum LocomotionMode;
    const double linear_mps = command.lin_vel.head<2>().norm();
    const double yaw_radps = std::abs(command.ang_vel.z());
    const bool active = walking_allowed
        && (linear_mps > config_.start_linear_mps || yaw_radps > config_.start_yaw_radps);

    if (state == StateID::STANDING) {
        capture_standing_xy(estimate);
    }
    if ((state == StateID::STANDING || recovering) && standing_xy_ && valid_planar_estimate(estimate)) {
        const double position_m = (estimate->pos.head<2>() - *standing_xy_).norm();
        const double velocity_mps = estimate->lin_vel.head<2>().norm(); // Опорная скорость в мире равна нулю.
        if (recovering) {
            // Выход требует устойчивости по обеим ошибкам для запрошенного режима.
            const auto& exit = active ? config_.recovery_exit_walking : config_.recovery_exit_standing;
            if (position_m < exit.position_m && velocity_mps < exit.velocity_mps) {
                return active ? WALKING : STANDING;
            }
        } else if (position_m > config_.recovery_entry.position_m
                   && velocity_mps > config_.recovery_entry.velocity_mps) {
            // Отклонение или скорость по отдельности недостаточны для входа.
            return RECOVERY;
        }
    }
    if (recovering) {
        return RECOVERY;
    }
    if (state == StateID::STANDING) {
        return active ? WALKING : STANDING;
    }

    const bool below_stop = linear_mps < config_.stop_linear_mps && yaw_radps < config_.stop_yaw_radps;
    if (!walking_allowed) {
        stopping_ = true;
    } else if (active || (!stopping_ && !below_stop)) {
        // После запроса остановки для её отмены нужна полноценная команда старта.
        below_stop_since_s_.reset();
        stopping_ = false;
    } else if (!stopping_) {
        if (!below_stop_since_s_) {
            below_stop_since_s_ = now_s;
        }
        stopping_ = now_s - *below_stop_since_s_ >= config_.stop_hold_s;
    }
    return stopping_ ? STANDING : WALKING;
}

} // namespace mors::fsm
