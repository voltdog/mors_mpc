#pragma once

#include "LocomotionControlBase.hpp"

#include <filesystem>
#include <memory>

namespace mors::fsm {

class LCMExchanger;

class LocomotionControllerMPC final : public LocomotionControlBase {
public:
    LocomotionControllerMPC(const std::filesystem::path& config, LCMExchanger& exchange);
    ~LocomotionControllerMPC() override;

    void set_data_input_mode(const uint8_t& mode) override;
    void begin_input_handoff(std::uint8_t mode, double duration_s, double max_joint_speed_rad_s) override;
    bool joints_control_ready() const override;
    void set_locomotion_mode(const uint8_t& mode) override;
    void set_robot_cmd(const RobotData& cmd) override;
    void set_body_height_override(std::optional<BodyHeightCommand> height) override;
    void hold_measured_pose() override;
    bool robot_control_ready() const override;
    void set_wbc_cmd(const WbcDesiredCommand& cmd) override;
    void set_joints_cmd(const JointCommand& cmd) override;
    void set_standing_xy(const Eigen::Vector2d& xy) override;
    void set_gait_params(const double& stance, const double& swing,
                         const std::vector<double>& offsets, const double& height) override;
    void set_adaptation_type(const uint8_t& adaptation) override;
    void set_active_legs(const std::vector<bool>& legs) override;
    void start_thread() override;
    void stop_thread() override;
    void disable_outputs() override;
    void enable_outputs() override;
    void request_gait_stop() override;
    bool gait_stopped() const override;
    std::optional<SafetyFault> controller_fault() override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    bool accept_input(uint8_t mode);
    void commit_robot_command(const RobotData* cmd);
    void check_deadline();
    void wbic_tick();
    void run(std::stop_token stop) override;
};

} // namespace mors::fsm
