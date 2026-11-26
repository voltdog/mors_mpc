#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

namespace mors::fsm {

class SpeedAverage {
public:
    void reset(std::optional<double> received_at_s = std::nullopt) {
        index_ = count_ = 0;
        sum_ = 0.0;
        last_sample_s_ = received_at_s && std::isfinite(*received_at_s) ? received_at_s : std::nullopt;
    }

    void update(double speed, std::optional<double> received_at_s) {
        if (!received_at_s || !std::isfinite(*received_at_s) || !std::isfinite(speed)) {
            reset(received_at_s);
            return;
        }
        if (last_sample_s_ && *received_at_s <= *last_sample_s_) {
            return;
        }
        last_sample_s_ = received_at_s;
        if (count_ == samples_.size()) {
            sum_ -= samples_[index_];
        } else {
            ++count_;
        }
        samples_[index_] = speed;
        sum_ += speed;
        index_ = (index_ + 1) % samples_.size();
    }

    std::optional<double> value() const {
        if (count_ != samples_.size()) return std::nullopt;
        return sum_ / samples_.size();
    }

private:
    std::array<double, 50> samples_{};
    std::size_t index_ = 0;
    std::size_t count_ = 0;
    double sum_ = 0.0;
    std::optional<double> last_sample_s_;
};

} // namespace mors::fsm
