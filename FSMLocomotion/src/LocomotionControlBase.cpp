#include "LocomotionControlBase.hpp"

#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

namespace mors::fsm {

void LocomotionControlBase::set_data_input_mode(const u_int8_t& input_mode) 
{
    this->input_mode = input_mode;
}

void LocomotionControlBase::set_locomotion_mode(const u_int8_t& mode) {
    assert(input_mode == robot_input);
    assert(mode == standing_mode || mode == walking_mode || mode == recovery_mode);
    // Демонстрационный run() ещё не выполняет управление в выбранном режиме.
    locomotion_mode_ = mode;
    cout << "Set locomotion mode: " << int(locomotion_mode_) << endl;
}

void LocomotionControlBase::set_robot_cmd(const RobotData& robot_cmd)
{
    assert(input_mode == robot_input);
    if (outputs_enabled()) {
        robot_cmd_ = robot_cmd;
        gait_stopped_ = false;
    }
}

void LocomotionControlBase::set_wbc_cmd(const WbcDesiredCommand&) 
{
    assert(input_mode == 2);
    // Заглушка передачи команды WBC-контроллеру.
}

void LocomotionControlBase::set_joints_cmd(const JointVector&) 
{
    assert(input_mode == 3);
    // Заглушка передачи команды приводам.
}

void LocomotionControlBase::set_gait_params(const double&, const double&,
                                       const std::vector<double>&, const double&) 
                                       {
    // Заглушка установки параметров походки.
}

void LocomotionControlBase::set_adaptation_type(const u_int8_t&) 
{
    // Заглушка выбора типа адаптации.
}

void LocomotionControlBase::set_active_legs(const std::vector<bool>&) 
{
    // Заглушка выбора активных ног.
}

void LocomotionControlBase::set_robot_state(const RobotData&, const LegData&) 
{
    // Заглушка передачи состояния робота.
}

void LocomotionControlBase::start_thread()
{
    const std::lock_guard lock(worker_mutex);
    if (!worker.joinable()) {
        worker = std::jthread([this](std::stop_token stop_token) { run(stop_token); });
    }
}

void LocomotionControlBase::stop_thread()
{
    const std::lock_guard lock(worker_mutex);
    if (worker.joinable()) {
        worker.request_stop();
        worker.join();
    }
}

void LocomotionControlBase::disable_outputs() 
{
    const bool was_enabled = enable.exchange(false);
    // На роботе здесь нужны блокировка выходов и аварийное действие в приводах.
    robot_cmd_ = RobotData{};
    gait_stopped_ = false;
    if (was_enabled) {
        std::cout << "Controller: outputs inhibited\n";
    }
}

void LocomotionControlBase::enable_outputs() 
{
    // Аппаратное включение приводов потребует отдельного подтверждения.
    enable = true;
}

bool LocomotionControlBase::outputs_enabled() const
{
    return enable.load();
}

void LocomotionControlBase::request_gait_stop()
{
    robot_cmd_.lin_vel.setZero();
    robot_cmd_.ang_vel.setZero();
    gait_stopped_ = true; // Заглушка останавливает походку за один вызов.
}

bool LocomotionControlBase::gait_stopped() const
{
    return gait_stopped_;
}

void LocomotionControlBase::run(std::stop_token stop_token)
{
    // инициализировать параметры
    // ...
    std::cout << "Locomotion Control: started\n";
    while (!stop_token.stop_requested())
    {
        if (enable.load())
        {
            // выполнять шаги управления локомоцией
            // ...
        } 
        else 
        {
            // выдать нулевые команды на приводы
            // ...
        }

        // std::cout << "Locomotion Control: running step\n";
        std::this_thread::sleep_for(std::chrono::seconds{1});
    }
}

} // namespace mors::fsm
