#include "LocomotionControl.hpp"

#include <iostream>

namespace mors::fsm {

void LocomotionControl::set_data_input_mode(std::string_view input_mode) {
    // Будущее переключение источника команд: CONTROLLER, WBC или JOINTS.
    this->input_mode = input_mode;
}

void LocomotionControl::set_robot_cmd(const RobotCommand&) {
    // Заглушка передачи команды контроллеру движения.
}

void LocomotionControl::set_wbc_cmd(const WbcCommand&) {
    // Заглушка передачи команды WBC-контроллеру.
}

void LocomotionControl::set_joints_cmd(const JointsCommand&) {
    // Заглушка передачи команды приводам.
}

void LocomotionControl::set_locomotion_mode(std::string_view) {
    // Будущее переключение STAND/WALK при input_mode == "CONTROLLER".
}

void LocomotionControl::start_thread() {
    // Заглушка запуска потока управления движением.
}

void LocomotionControl::stop_thread() {
    // Заглушка остановки потока управления движением.
}

void LocomotionControl::set_velocity(double vx, double vy, double yaw_rate) {
    if (outputs_inhibited) {
        return;
    }
    velocity_command = {vx, vy, yaw_rate};
}

void LocomotionControl::step() {
    if (outputs_inhibited) {
        return;
    }
    // Здесь будет алгоритм управления. RobotStatus обновляют источники телеметрии.
}

void LocomotionControl::emergency_stop() {
    const bool already_inhibited = outputs_inhibited;
    outputs_inhibited = true;
    velocity_command = {0.0, 0.0, 0.0};
    // На роботе здесь нужны блокировка выходов и аварийное действие в приводах.
    if (!already_inhibited) {
        std::cout << "Controller: outputs inhibited, emergency stop requested\n";
    }
}

void LocomotionControl::enable_outputs() {
    // Аппаратное включение приводов потребует отдельного подтверждения.
    outputs_inhibited = false;
}

} // namespace mors::fsm
