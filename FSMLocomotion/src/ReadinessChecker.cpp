#include "ReadinessChecker.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace mors::fsm {

ReadinessChecker::ReadinessChecker(SafetyMonitor& safety_monitor,
                                   double max_status_age_s, double stable_time_s)
    : safety_monitor(safety_monitor), max_status_age_s(max_status_age_s),
      stable_time_s(stable_time_s) {
    if (!std::isfinite(max_status_age_s) || max_status_age_s <= 0.0) {
        throw std::invalid_argument("max_status_age_s must be positive");
    }
    if (!std::isfinite(stable_time_s) || stable_time_s < 0.0) {
        throw std::invalid_argument("stable_time_s must be non-negative");
    }
}

void ReadinessChecker::reset() {
    stable_since.reset();
    last_errors.clear();
}

bool ReadinessChecker::fresh(const std::optional<double>& stamp, double now) const {
    if (!stamp || !std::isfinite(*stamp)) {
        return false;
    }
    const double age = now - *stamp;
    return -0.001 <= age && age <= max_status_age_s;
}

bool ReadinessChecker::is_ready(const RobotStatus& status) {
    const double now = monotonic_seconds();
    std::vector<std::string> errors;

    if (!status.drives_ready) {
        errors.emplace_back("Приводы не готовы");
    }
    if (!fresh(status.drive_status_stamp_s, now)) {
        errors.emplace_back("Нет актуального статуса приводов");
    }
    if (!safety_monitor.is_encoder_feedback_valid(status.joint_feedback)) {
        errors.emplace_back("Нет корректных актуальных данных энкодеров");
    }
    if (!status.contact_sensors_ready) {
        errors.emplace_back("Датчики контакта не готовы");
    }
    if (!fresh(status.contact_stamp_s, now)) {
        errors.emplace_back("Нет актуальных данных датчиков контакта");
    }
    // Размер и тип contacts гарантированы std::array<bool, 4>.
    if (!status.estimator_ready) {
        errors.emplace_back("Оценщик состояния не готов");
    }
    if (!fresh(status.estimator_stamp_s, now)) {
        errors.emplace_back("Нет актуальных данных оценщика состояния");
    }
    if (!std::isfinite(status.vx) || !std::isfinite(status.vy) || !std::isfinite(status.yaw_rate)) {
        errors.emplace_back("Некорректные скорости оценщика состояния");
    }

    last_errors = std::move(errors);
    if (!last_errors.empty()) {
        stable_since.reset();
        return false;
    }
    if (!stable_since) {
        stable_since = now;
    }
    return now - *stable_since >= stable_time_s;
}

} // namespace mors::fsm
