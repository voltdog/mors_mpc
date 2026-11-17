#pragma once

#include "FSM/FSMTypes.hpp"

namespace mors::fsm {

// Диагностика готовности перед выходом из IDLE; сама не вызывает ESTOP.
class ReadinessChecker {
public:
    explicit ReadinessChecker(double stable_time_s = 0.20);

    void reset();
    bool is_ready(const RobotStatus& status, double now);

private:
    double stable_time_s_;
    std::optional<double> stable_since_;
};

} // namespace mors::fsm
