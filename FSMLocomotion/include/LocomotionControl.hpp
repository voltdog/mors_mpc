#pragma once

#include <array>
#include <optional>
#include <string>
#include <string_view>

namespace mors::fsm {

// Содержимое команд будет определено при подключении реального контроллера.
struct RobotCommand {};
struct WbcCommand {};
struct JointsCommand {};

// Демонстрационная заглушка. Методы не управляют аппаратурой и не создают потоки.
class LocomotionControl {
public:
    virtual ~LocomotionControl() = default;

    virtual void set_data_input_mode(std::string_view input_mode);
    virtual void set_robot_cmd(const RobotCommand& robot_cmd);
    virtual void set_wbc_cmd(const WbcCommand& wbc_cmd);
    virtual void set_joints_cmd(const JointsCommand& joints_cmd);
    virtual void set_locomotion_mode(std::string_view mode);
    virtual void start_thread();
    virtual void stop_thread();

    // Временные методы исходного прототипа.
    virtual void set_velocity(double vx, double vy, double yaw_rate);
    virtual void step();
    virtual void emergency_stop();
    virtual void enable_outputs();

    std::array<double, 3> velocity_command{};
    bool outputs_inhibited = false;
    std::optional<std::string> input_mode;
};

} // namespace mors::fsm
