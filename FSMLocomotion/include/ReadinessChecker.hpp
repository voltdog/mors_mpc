#pragma once

#include "FSMTypes.hpp"
#include "SafetyMonitor.hpp"

namespace mors::fsm {

// Диагностика готовности перед выходом из IDLE; сама не вызывает ESTOP.
class ReadinessChecker {
public:
    explicit ReadinessChecker(SafetyMonitor& safety_monitor,
                              double max_status_age_s = 0.15, double stable_time_s = 0.20);

    void reset();
    bool is_ready(const RobotStatus& status);

    SafetyMonitor& safety_monitor;
    double max_status_age_s;
    double stable_time_s;
    std::optional<double> stable_since;
    std::vector<std::string> last_errors;

private:
    bool fresh(const std::optional<double>& stamp, double now) const;
};

} // namespace mors::fsm
