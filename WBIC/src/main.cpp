#include <iostream>
#include <array>
#include <chrono>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include "data_types.hpp"
#include "lcm_data_exchange_wbic.hpp"
#include "wbic_control.hpp"
#include "Robot.hpp"

using namespace std;
using namespace Eigen;
using namespace YAML;
using namespace std::chrono;
namespace wbic_types = mors::wbic;

// current time
auto now() 
{
  return std::chrono::steady_clock::now(); 
}

int main() {
    cout << "[WBIC]: Starting..." << endl;

    // load configs
    string config_address = mors_sys::GetEnv("CONFIGPATH");

    // read robot parameters
    string robot_config_address = config_address + "/robot_config.yaml";
    YAML::Node robot_config = YAML::LoadFile(robot_config_address);
    RobotPhysicalParams robot;
    robot.kt = robot_config["kt"].as<double>(); 
    robot.gear_ratio = robot_config["gear_ratio"].as<double>(); 
    for (int i = 0; i < 3; i++)
        robot.tau_max_array[i] = robot_config["tau_max"][i].as<double>(); 

    // read module dt
    string dt_config_address = config_address + "/timesteps.yaml";
    YAML::Node dt_config = YAML::LoadFile(dt_config_address);
    double module_dt = dt_config["wbic_controller_dt"].as<double>(); 

    // read WBIC parameters
    string wbic_config_address = config_address + "/wbic.yaml";
    YAML::Node wbic_config = YAML::LoadFile(wbic_config_address);
    double Qf_entry = wbic_config["Qf_entry"].as<double>(); 
    double Qa_entry = wbic_config["Qa_entry"].as<double>(); 
    double body_ori_task_kp = wbic_config["body_ori_task_kp"].as<double>(); 
    double body_ori_task_kd = wbic_config["body_ori_task_kd"].as<double>(); 
    double body_pos_task_kp = wbic_config["body_pos_task_kp"].as<double>(); 
    double body_pos_task_kd = wbic_config["body_pos_task_kd"].as<double>(); 
    double tip_pos_task_kp = wbic_config["tip_pos_task_kp"].as<double>(); 
    double tip_pos_task_kd = wbic_config["tip_pos_task_kd"].as<double>(); 
    double joint_kp_stance = wbic_config["joint_kp_stance"].as<double>(); 
    double joint_kd_stance = wbic_config["joint_kd_stance"].as<double>(); 
    double joint_kp_swing = wbic_config["joint_kp_swing"].as<double>(); 
    double joint_kd_swing = wbic_config["joint_kd_swing"].as<double>(); 


    // init some essential params
    auto dt = std::chrono::duration<double>(module_dt);//1ms;
    wbic_types::Vector12d tau_max;
    wbic_types::Vector12d tau_min;
    tau_max <<  robot.tau_max_array[0], robot.tau_max_array[1], robot.tau_max_array[2],
                robot.tau_max_array[0], robot.tau_max_array[1], robot.tau_max_array[2],
                robot.tau_max_array[0], robot.tau_max_array[1], robot.tau_max_array[2],
                robot.tau_max_array[0], robot.tau_max_array[1], robot.tau_max_array[2];
    tau_min <<  -robot.tau_max_array[0], -robot.tau_max_array[1], -robot.tau_max_array[2],
                -robot.tau_max_array[0], -robot.tau_max_array[1], -robot.tau_max_array[2],
                -robot.tau_max_array[0], -robot.tau_max_array[1], -robot.tau_max_array[2],
                -robot.tau_max_array[0], -robot.tau_max_array[1], -robot.tau_max_array[2];
    
    Robot dyn_model;
    dyn_model.BuildPinocchioModel();

    LegData leg_cmd, leg_state;
    RobotData body_cmd, body_state;
    wbic_types::Vector12d ref_joint_pos_our;
    wbic_types::Vector12d ref_joint_vel_our;
    wbic_types::Vector12d ref_joint_torque_our;
    wbic_types::Vector12d cur_joint_pos;
    wbic_types::Vector12d cur_joint_vel;
    wbic_types::Vector12d cur_joint_torque;
    wbic_types::Vector12d ref_joint_pos_pin;
    wbic_types::Vector12d ref_joint_vel_pin;
    wbic_types::Vector12d ref_joint_torque_pin;
    wbic_types::Vector12d fr_result;
    wbic_types::Vector12d motor_kp;
    wbic_types::Vector12d motor_kd;
    wbic_types::Vector4i phase_signal;
    wbic_types::Vector4i current_support_state;
    wbic_types::Vector4i support_state_pino;

    wbic_types::Vector19d ref_x;
    wbic_types::Vector18d ref_dx;
    wbic_types::Vector18d ref_ddx;
    wbic_types::Vector12d fr_mpc;
    ref_joint_pos_our.setZero();
    ref_joint_vel_our.setZero();
    ref_joint_torque_our.setZero();
    cur_joint_pos.setZero();
    cur_joint_vel.setZero();
    cur_joint_torque.setZero();
    ref_joint_pos_pin.setZero();
    ref_joint_vel_pin.setZero();
    ref_joint_torque_pin.setZero();
    fr_result.setZero();
    motor_kp.setZero();
    motor_kd.setZero();
    phase_signal.setZero();
    current_support_state.setZero();
    support_state_pino.setZero();
    ref_x.setZero();
    ref_dx.setZero();
    ref_ddx.setZero();
    fr_mpc.setZero();


    WBIC_Control wbic;
    wbic.set_q_entries(Qa_entry, Qf_entry);
    wbic.set_task_gains(body_ori_task_kp, body_ori_task_kd,
                        body_pos_task_kp, body_pos_task_kd,
                        tip_pos_task_kp, tip_pos_task_kd);


    LCMExchanger lcmExch;
    if (!lcmExch.initialized) return -1;
    lcmExch.start_exchanger();
    
    bool enable = false;
    bool reset = false;
    bool first = true;
    static constexpr size_t timing_history_capacity = 1024;
    std::array<std::chrono::steady_clock::time_point, timing_history_capacity> elapsed_timestamps{};
    std::array<double, timing_history_capacity> elapsed_values_ms{};
    size_t elapsed_head = 0;
    size_t elapsed_size = 0;
    double elapsed_sum_ms = 0.0;
    const auto average_window = std::chrono::milliseconds(500);
    auto last_average_print = now();

    cout << "[WBIC]: Started" << endl;

    while(true)
    {
        // Calculating current time
        const auto start{ now() };

        // Put your code here
        // -----------------------------------------------
        lcmExch.getEnableData(enable, reset);
        lcmExch.getWbcCmdData(body_cmd, leg_cmd);
        body_state = lcmExch.getBodyState();
        leg_state = lcmExch.getLegState();
        lcmExch.getServoStateData(cur_joint_pos, cur_joint_vel, cur_joint_torque);
        phase_signal = lcmExch.getGaitPhaseSignal();

        for (int i = 0; i < 4; i++)
        {
            if (phase_signal[i] == STANCE || phase_signal[i] == EARLY_CONTACT)
                current_support_state[i] = 1; // stance
            else 
                current_support_state[i] = 0; // swing
        }


        // enable = true;
        if (enable == true)
        {
            first = true;

            dyn_model.update(body_state, cur_joint_pos, cur_joint_vel);

            support_state_pino[PIN_L1] = current_support_state[L1];
            support_state_pino[PIN_L2] = current_support_state[L2];
            support_state_pino[PIN_R1] = current_support_state[R1];
            support_state_pino[PIN_R2] = current_support_state[R2];
            dyn_model.update_support_states(support_state_pino);

            // construct desired states: ref_x, ref_dx, ref_ddx, fr_mpc
            ref_x.setZero();
            ref_dx.setZero();
            ref_ddx.setZero();
            fr_mpc.setZero();

            ref_x.segment(0, 3) = body_cmd.pos;
            Eigen::Vector4d ref_body_quat = body_cmd.orientation_quaternion;
            const double ref_quat_norm = ref_body_quat.norm();
            if (ref_quat_norm < 1e-6)
            {
                ref_body_quat = body_cmd.orientation_quaternion;
            }
            else
            {
                ref_body_quat /= ref_quat_norm;
            }
            ref_x.segment(3, 4) = ref_body_quat;
            ref_x.segment(PIN_START_IDX+PIN_R1*3, 3) = leg_cmd.r1_pos;
            ref_x.segment(PIN_START_IDX+PIN_L1*3, 3) = leg_cmd.l1_pos;
            ref_x.segment(PIN_START_IDX+PIN_R2*3, 3) = leg_cmd.r2_pos;
            ref_x.segment(PIN_START_IDX+PIN_L2*3, 3) = leg_cmd.l2_pos;
            
            ref_dx.segment(0, 3) = body_cmd.lin_vel;
            ref_dx.segment(3, 3) = body_cmd.ang_vel;
            ref_dx.segment(6+PIN_R1*3, 3) = leg_cmd.r1_vel;
            ref_dx.segment(6+PIN_L1*3, 3) = leg_cmd.l1_vel;
            ref_dx.segment(6+PIN_R2*3, 3) = leg_cmd.r2_vel;
            ref_dx.segment(6+PIN_L2*3, 3) = leg_cmd.l2_vel;

            ref_ddx.segment(0, 6).setZero();
            ref_ddx.segment(6+PIN_R1*3, 3) = leg_cmd.r1_acc;
            ref_ddx.segment(6+PIN_L1*3, 3) = leg_cmd.l1_acc;
            ref_ddx.segment(6+PIN_R2*3, 3) = leg_cmd.r2_acc;
            ref_ddx.segment(6+PIN_L2*3, 3) = leg_cmd.l2_acc;

            fr_mpc.segment(PIN_R1*3, 3) = leg_cmd.r1_grf;
            fr_mpc.segment(PIN_L1*3, 3) = leg_cmd.l1_grf;
            fr_mpc.segment(PIN_R2*3, 3) = leg_cmd.r2_grf;
            fr_mpc.segment(PIN_L2*3, 3) = leg_cmd.l2_grf;

            // step wbic control
            wbic.update(dyn_model, ref_x, ref_dx, ref_ddx, fr_mpc, ref_joint_pos_pin, ref_joint_vel_pin, ref_joint_torque_pin);
            
            // reorder joints from pin to our
            ref_joint_pos_our.segment<3>(R1 * 3) = ref_joint_pos_pin.segment<3>(PIN_R1 * 3);
            ref_joint_pos_our.segment<3>(L1 * 3) = ref_joint_pos_pin.segment<3>(PIN_L1 * 3);
            ref_joint_pos_our.segment<3>(R2 * 3) = ref_joint_pos_pin.segment<3>(PIN_R2 * 3);
            ref_joint_pos_our.segment<3>(L2 * 3) = ref_joint_pos_pin.segment<3>(PIN_L2 * 3);

            ref_joint_vel_our.segment<3>(R1 * 3) = ref_joint_vel_pin.segment<3>(PIN_R1 * 3);
            ref_joint_vel_our.segment<3>(L1 * 3) = ref_joint_vel_pin.segment<3>(PIN_L1 * 3);
            ref_joint_vel_our.segment<3>(R2 * 3) = ref_joint_vel_pin.segment<3>(PIN_R2 * 3);
            ref_joint_vel_our.segment<3>(L2 * 3) = ref_joint_vel_pin.segment<3>(PIN_L2 * 3);

            ref_joint_torque_our.segment<3>(R1 * 3) = ref_joint_torque_pin.segment<3>(PIN_R1 * 3);
            ref_joint_torque_our.segment<3>(L1 * 3) = ref_joint_torque_pin.segment<3>(PIN_L1 * 3);
            ref_joint_torque_our.segment<3>(R2 * 3) = ref_joint_torque_pin.segment<3>(PIN_R2 * 3);
            ref_joint_torque_our.segment<3>(L2 * 3) = ref_joint_torque_pin.segment<3>(PIN_L2 * 3);


            // set motor kp and kd gains depending on current leg state (swing or stance)
            for (int i = 0; i < 4; i++)
            {
                if (current_support_state[i] == 1)
                {
                    motor_kp.segment(i*3, 3) << joint_kp_stance, joint_kp_stance, joint_kp_stance;
                    motor_kd.segment(i*3, 3) << joint_kd_stance, joint_kd_stance, joint_kd_stance;
                }
                else
                {
                    motor_kp.segment(i*3, 3) << joint_kp_swing, joint_kp_swing, joint_kp_swing;
                    motor_kd.segment(i*3, 3) << joint_kd_swing, joint_kd_swing, joint_kd_swing;
                }
            }

            // clip torque and then convert to actuator values
            ref_joint_torque_our =
                ref_joint_torque_our.cwiseMin(tau_max).cwiseMax(tau_min) * robot.gear_ratio / robot.kt;
            
            // send to actuators
            lcmExch.sendServoCmd(ref_joint_pos_our, ref_joint_vel_our, ref_joint_torque_our, motor_kp, motor_kd);
            
            // send wbc state
            fr_result = wbic.get_fr_result();
            lcmExch.sendWbcState(fr_result);

        }
        else
        {
            if (first == true)
            {
                // объявить все эти переменные
                ref_joint_torque_pin.setZero(); 
                fr_result.setZero();
                motor_kp.setZero();
                motor_kd.setZero();
                first = false;
                lcmExch.sendWbcState(fr_result);
                lcmExch.sendServoCmd(ref_joint_pos_pin, ref_joint_vel_pin, ref_joint_torque_pin, motor_kp, motor_kd);
            }
        }

        // // измерение среднего времени выполнения алгоритма
        // const auto sample_time = now();
        // std::chrono::duration<double, std::milli> elapsed{sample_time - start};
        // const size_t elapsed_tail = (elapsed_head + elapsed_size) % timing_history_capacity;
        // if (elapsed_size == timing_history_capacity)
        // {
        //     elapsed_sum_ms -= elapsed_values_ms[elapsed_head];
        //     elapsed_head = (elapsed_head + 1) % timing_history_capacity;
        //     elapsed_size--;
        // }

        // elapsed_timestamps[elapsed_tail] = sample_time;
        // elapsed_values_ms[elapsed_tail] = elapsed.count();
        // elapsed_sum_ms += elapsed.count();
        // elapsed_size++;

        // while (elapsed_size > 0 && sample_time - elapsed_timestamps[elapsed_head] > average_window)
        // {
        //     elapsed_sum_ms -= elapsed_values_ms[elapsed_head];
        //     elapsed_head = (elapsed_head + 1) % timing_history_capacity;
        //     elapsed_size--;
        // }

        // if (elapsed_size > 0 && sample_time - last_average_print >= average_window)
        // {
        //     cout << "Average waited for last 0.5 s: " << elapsed_sum_ms / static_cast<double>(elapsed_size) << " ms" << endl;
        //     last_average_print = sample_time;
        // }
        // -----------------------------------------------
        // std::chrono::duration<double, std::micro> elapsed_{now() - start};
        // cout << "Waited for : " << elapsed_.count() << " us" << endl;

        // Wait until spinning time
        while(true)
        {
            std::chrono::duration<double, std::milli> elapsed{now() - start};
            if (elapsed >= dt)
            {   
                if (elapsed.count() > 1000*module_dt+0.05)
                    cout << "[WBIC]: Waited for : " << elapsed.count() << " ms" << endl;
                break;
            }
        }
    }

    return 0;
}
