#pragma once

#include "LcmDataExchange.hpp"
#include "LocomotionControllerMPC/LocomotionControllerMPC.hpp"
#include "LocomotionTransitionMonitor.hpp"
#include "RobotFSM.hpp"

#include <chrono>
#include <filesystem>

namespace YAML {
class Node;
}

namespace mors::fsm {

class LocomotionNode {
public:
    LocomotionNode();
    ~LocomotionNode();

    void step();
    std::chrono::steady_clock::duration period() const;

private:
    explicit LocomotionNode(const std::filesystem::path& config_path);
    LocomotionNode(const std::filesystem::path& config_path, const YAML::Node& robot_config);

    void update_status(const std::optional<RobotStateSnapshot>& observation, double now_s);
    void process_mode_commands(const std::vector<mors_msgs::robot_mode_cmd_msg>& commands);
    void request_mode(std::int8_t target, bool fault_cleared);
    void finish_mode_request();
    void select_effective_mode(LocomotionSetpoints& setpoints, const RobotData* estimate, double now_s);

    const std::chrono::steady_clock::duration period_;
    RobotStatus status_;
    SafetyMonitor safety_monitor_;
    ReadinessChecker readiness_checker_;
    LCMExchanger exchanger_;
    LocomotionControllerMPC controller_;
    LocomotionTransitionMonitor transition_monitor_;
    RobotFSM fsm_;
    std::optional<StateID> operator_request_;
    std::optional<StateID> pending_mode_request_;
    double last_reset_s_ = 0.0;
    LocomotionMode locomotion_mode_ = LocomotionMode::WALKING;
};

} // namespace mors::fsm
