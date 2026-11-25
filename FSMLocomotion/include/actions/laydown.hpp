#pragma once

#include "LocomotionControlBase.hpp"

#include <variant>

namespace mors::fsm {

struct LaydownConfig {
    double kp = 12.0;
    double kd = 0.4;
};

class LaydownAction {
public:
    enum class Progress { running, finished };
    using Result = std::variant<Progress, SafetyFault>;

    explicit LaydownAction(LocomotionControlBase& controller, LaydownConfig config = {});
    void reset();
    Result step(double now_s);

private:
    enum class Phase { fold, center, damp, released };

    LocomotionControlBase& controller_;
    LaydownConfig config_;
    Phase phase_ = Phase::fold;
    std::optional<double> phase_started_at_s_;
    JointVector initial_;
    JointVector folded_;
    JointVector centered_;
};

} // namespace mors::fsm
