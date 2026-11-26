#pragma once

#include "structs.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mors::fsm {

// Cubic correction to a live destination. Its position/velocity vanish together.
class JointCommandHandoff {
public:
    JointCommandHandoff(const JointCommand& from, const JointCommand& to,
                        double minimum_duration_s, double max_speed)
        : from_(from), offset_(from.position - to.position),
          velocity_offset_(from.velocity - to.velocity), duration_(minimum_duration_s) {
        if (!std::isfinite(duration_) || duration_ <= 0.0 ||
            !std::isfinite(max_speed) || max_speed <= 0.0 ||
            !offset_.allFinite() || !velocity_offset_.allFinite()) {
            throw std::invalid_argument("Invalid input handoff");
        }
        // Bound the analytic velocity of the initial destination extrapolation.
        // Subsequent live WBIC commands are checked again before publication.
        for (int attempt = 0; attempt < 40; ++attempt) {
            if (!std::isfinite(duration_)) throw std::invalid_argument("Invalid input handoff duration");
            double peak = 0.0;
            for (int i = 0; i < offset_.size(); ++i) {
                const double a = 6.0 * offset_(i) / duration_ + 3.0 * velocity_offset_(i);
                const double b = -6.0 * offset_(i) / duration_ - 4.0 * velocity_offset_(i);
                const double c = from.velocity(i);
                peak = std::max({peak, std::abs(c), std::abs(a + b + c)});
                if (a != 0.0) {
                    const double u = -b / (2.0 * a);
                    if (u > 0.0 && u < 1.0) peak = std::max(peak, std::abs((a * u + b) * u + c));
                }
            }
            if (peak <= max_speed * (1.0 + 1e-12)) return;
            duration_ *= 2.0;
        }
        throw std::invalid_argument("Input handoff cannot meet joint speed limit");
    }

    JointCommand sample(const JointCommand& to, double elapsed_s) const {
        if (elapsed_s >= duration_) return to;
        const double u = std::clamp(elapsed_s / duration_, 0.0, 1.0);
        const double s = u * u * (3.0 - 2.0 * u);
        const double ds = 6.0 * u * (1.0 - u) / duration_;
        const double h = u * (1.0 - u) * (1.0 - u);
        const double dh = 1.0 - 4.0 * u + 3.0 * u * u;
        JointCommand command;
        command.position = to.position + (1.0 - s) * offset_ + duration_ * h * velocity_offset_;
        command.velocity = to.velocity - ds * offset_ + dh * velocity_offset_;
        command.kp = (1.0 - s) * from_.kp + s * to.kp;
        command.kd = (1.0 - s) * from_.kd + s * to.kd;
        command.torque = (1.0 - s) * from_.torque + s * to.torque;
        return command;
    }

    double duration() const { return duration_; }

private:
    JointCommand from_;
    JointVector12d offset_, velocity_offset_;
    double duration_;
};

} // namespace mors::fsm
