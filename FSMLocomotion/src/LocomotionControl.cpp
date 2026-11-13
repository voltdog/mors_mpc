#include "LocomotionControl.hpp"

#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

namespace mors::fsm {

void LocomotionControl::set_data_input_mode(const u_int8_t& input_mode) 
{
    this->input_mode = input_mode;
}

void LocomotionControl::set_locomotion_mode(const u_int8_t&) 
{
    // Заглушка переключения режима локомоции.
}

void LocomotionControl::set_robot_cmd(const RobotData&) 
{
    assert(input_mode == 1);
    // Заглушка передачи команды контроллеру движения.
}

void LocomotionControl::set_wbc_cmd(const WbcDesiredCommand&) 
{
    assert(input_mode == 2);
    // Заглушка передачи команды WBC-контроллеру.
}

void LocomotionControl::set_joints_cmd(const JointVector&) 
{
    assert(input_mode == 3);
    // Заглушка передачи команды приводам.
}

void LocomotionControl::set_gait_params(const double&, const double&,
                                       const std::vector<double>&, const double&) 
                                       {
    // Заглушка установки параметров походки.
}

void LocomotionControl::set_adaptation_type(const u_int8_t&) 
{
    // Заглушка выбора типа адаптации.
}

void LocomotionControl::set_active_legs(const std::vector<bool>&) 
{
    // Заглушка выбора активных ног.
}

void LocomotionControl::set_robot_state(const RobotData&, const LegData&) 
{
    // Заглушка передачи состояния робота.
}

void LocomotionControl::start_thread()
{
    const std::lock_guard lock(worker_mutex);
    if (!worker.joinable()) {
        worker = std::jthread([this](std::stop_token stop_token) { run(stop_token); });
    }
}

void LocomotionControl::stop_thread()
{
    const std::lock_guard lock(worker_mutex);
    if (worker.joinable()) {
        worker.request_stop();
        worker.join();
    }
}

void LocomotionControl::disable_outputs() 
{
    const bool was_enabled = enable.exchange(false);
    // На роботе здесь нужны блокировка выходов и аварийное действие в приводах.
    if (was_enabled) {
        std::cout << "Controller: outputs inhibited, emergency stop requested\n";
    }
}

void LocomotionControl::enable_outputs() 
{
    // Аппаратное включение приводов потребует отдельного подтверждения.
    enable = true;
}

void LocomotionControl::run(std::stop_token stop_token)
{
    // инициализировать параметры
    // ...

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

        std::cout << "Locomotion Control: running step\n";
        std::this_thread::sleep_for(std::chrono::seconds{1});
    }
}

} // namespace mors::fsm
