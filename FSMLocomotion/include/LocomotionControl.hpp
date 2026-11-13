#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <string_view>
#include <thread>
#include "structs.hpp"

namespace mors::fsm {

using JointVector = mors::wbic::Vector12d;

// Демонстрационная заглушка контура управления локомоцией.
class LocomotionControl {
public:
    virtual ~LocomotionControl() = default;

    // в зависимости от выбора режима, будут использоваться разные функции: 
    // 1 - set_robot_cmd, 
    // 2 - set_wbc_cmd, 
    // 3 - set_joints_cmd
    virtual void set_data_input_mode(const u_int8_t& input_mode); 

    // В зависимости от значения mode будут работать разные режимы:
    // 1 - standing mode
    // 2 - walking mode 
    // 3 - мб recovery mode
    // работает только в режиме input_mode = 1
    virtual void set_locomotion_mode(const u_int8_t& mode);

    // установка заданных команд. 
    // В зависимости от выбранного режима будут использоваться разные функции. 
    // Если режим не соответствует вызываемой функции, то произойдет assertion failed.
    virtual void set_robot_cmd(const RobotData& robot_cmd);
    virtual void set_wbc_cmd(const WbcDesiredCommand& wbc_cmd);
    virtual void set_joints_cmd(const JointVector& joints_cmd);

    // works only in walking mode
    virtual void set_gait_params(const double& t_st,
                                 const double& t_sw,
                                 const vector<double>& gait_type,
                                 const double& stride_height);
    
    // works only in walking mode
    // adaptation types declared in structs.hpp: INCL_ADAPT, HEIGHT_ADAPT, NOADAPT
    virtual void set_adaptation_type(const u_int8_t& adaptation_type);

    // works in the all modes
    virtual void set_active_legs(const vector<bool>& active_legs);
    virtual void set_robot_state(const RobotData& robot_state,
                                 const LegData& leg_state);

    // Запуск потока управления локомоцией.
    virtual void start_thread();
    // Запрос остановки и ожидание завершения потока
    virtual void stop_thread();

    // Прекращение выдачи команд на приводы
    virtual void disable_outputs();
    // Возобновление выдачи команд на приводы после вызова emergency_stop()
    virtual void enable_outputs();

    
    std::optional<u_int8_t> input_mode;

private:

    // Контур управления должен завершаться по запросу stop_token.
    // Класс, переопределяющий run(), останавливает поток в своём деструкторе.
    // в этой функции будет реализован контур управления локомоцией, как в LocomotionController/main.cpp
    virtual void run(std::stop_token stop_token);

    std::atomic<bool> enable{false};
    std::mutex worker_mutex;
    std::jthread worker;
};

} // namespace mors::fsm
