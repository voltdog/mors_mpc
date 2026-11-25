#pragma once

#include "State.hpp"
#include "LocomotionData.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace mors::fsm {

class LocomotionState : public State {
public:
    explicit LocomotionState(RobotFSM& fsm, GaitStopConfig gait_stop = {},
                             LocomotionAccelerationConfig acceleration = {});
    void on_enter() override;
    std::optional<Event> on_update(const LocomotionSetpoints& setpoints) override;
    bool can_exit(StateID next_state) override;
    void on_exit() override;

private:
    void update_speed_average();
    bool velocity_is_low() const;

    const GaitStopConfig gait_stop_;
    const LocomotionAccelerationConfig acceleration_;
    std::optional<std::uint8_t> applied_mode_;
    std::optional<double> stable_since_;
    std::optional<RobotData> last_command_;
    double last_update_s_ = 0.0;
    std::array<double, 50> speed_samples_{};
    std::size_t speed_index_ = 0;
    std::size_t speed_count_ = 0;
    double speed_sum_ = 0.0;
    std::optional<double> last_sample_s_;
    bool stop_requested_ = false;
};

} // namespace mors::fsm
