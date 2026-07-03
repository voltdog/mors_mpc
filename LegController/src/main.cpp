#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include "lcm_data_exchange_lc.hpp"
#include "leg_control.hpp"
#include "structs.hpp"
#include <vbmath.hpp>
#include <yaml-cpp/yaml.h>
#include "system_functions.hpp"

using namespace std;
using namespace Eigen;
using namespace YAML;
using namespace std::chrono;

// current time
auto now() 
{
  return std::chrono::steady_clock::now(); 
}

void set_motor_gains(Vector4i& phase_signal, 
                    Vector3d& servo_kp_swing, Vector3d& servo_kp_stance,
                    Vector3d& servo_kd_swing, Vector3d& servo_kd_stance, 
                    VectorXd& motor_kp, VectorXd& motor_kd)
{
    if (phase_signal[R1] == SWING || phase_signal[R1] == LATE_CONTACT)
    {
        motor_kp.segment(0, 3) = servo_kp_swing;
        motor_kd.segment(0, 3) = servo_kd_swing;
    }
    else
    {
        motor_kp.segment(0, 3) = servo_kp_stance;
        motor_kd.segment(0, 3) = servo_kd_stance;
    }

    if (phase_signal[L1] == SWING || phase_signal[L1] == LATE_CONTACT)
    {
        motor_kp.segment(3, 3) = servo_kp_swing;
        motor_kd.segment(3, 3) = servo_kd_swing;
    }
    else
    {
        motor_kp.segment(3, 3) = servo_kp_stance;
        motor_kd.segment(3, 3) = servo_kd_stance;
    }

    if (phase_signal[R2] == SWING || phase_signal[R2] == LATE_CONTACT)
    {
        motor_kp.segment(6, 3) = servo_kp_swing;
        motor_kd.segment(6, 3) = servo_kd_swing;
    }
    else
    {
        motor_kp.segment(6, 3) = servo_kp_stance;
        motor_kd.segment(6, 3) = servo_kd_stance;
    }

    if (phase_signal[L2] == SWING || phase_signal[L2] == LATE_CONTACT)
    {
        motor_kp.segment(9, 3) = servo_kp_swing;
        motor_kd.segment(9, 3) = servo_kd_swing;
    }
    else
    {
        motor_kp.segment(9, 3) = servo_kp_stance;
        motor_kd.segment(9, 3) = servo_kd_stance;
    }
}

int main() {
    cout << "[Leg Controller]: Starting..." << endl;
    // load configs
    string config_address = mors_sys::GetEnv("CONFIGPATH");
    string robot_config_address = config_address + "/robot_config.yaml";
    YAML::Node robot_config = YAML::LoadFile(robot_config_address);
    RobotPhysicalParams robot;
    robot.kt = robot_config["kt"].as<double>(); 
    robot.gear_ratio = robot_config["gear_ratio"].as<double>(); 
    for (int i = 0; i < 3; i++)
        robot.joint_tau_max_array[i] = robot_config["tau_max"][i].as<double>(); 

    string leg_config_address = config_address + "/imp_config.yaml";
    YAML::Node leg_config = YAML::LoadFile(leg_config_address);
    Eigen::Vector3d servo_kp_stance, servo_kd_stance;
    Eigen::Vector3d servo_kp_swing, servo_kd_swing;
    // servo_kp_stance.resize(12); servo_kd_stance.resize(12); servo_kp_swing.resize(12); servo_kd_swing.resize(12);

    for (int i = 0; i < 3; i++)
    {
        servo_kp_stance(i) = leg_config["Kp_stance"][i].as<double>(); 
        servo_kd_stance(i) = leg_config["Kd_stance"][i].as<double>(); 
        servo_kp_swing(i) = leg_config["Kp_swing"][i].as<double>(); 
        servo_kd_swing(i) = leg_config["Kd_swing"][i].as<double>(); 
    }

    // servo_kp_stance << servo_kp_stance.head(3), servo_kp_stance.head(3), servo_kp_stance.head(3), servo_kp_stance.head(3);
    // servo_kd_stance << servo_kd_stance.head(3), servo_kd_stance.head(3), servo_kd_stance.head(3), servo_kd_stance.head(3);
    // servo_kp_swing << servo_kp_swing.head(3), servo_kp_swing.head(3), servo_kp_swing.head(3), servo_kp_swing.head(3);
    // servo_kd_swing << servo_kd_swing.head(3), servo_kd_swing.head(3), servo_kd_swing.head(3), servo_kd_swing.head(3);


    string dt_config_address = config_address + "/timesteps.yaml";
    YAML::Node dt_config = YAML::LoadFile(dt_config_address);
    double module_dt = dt_config["leg_controller_dt"].as<double>(); 

    auto dt = std::chrono::duration<double>(module_dt);//1ms;
    VectorXd tau_max(12);
    VectorXd tau_min(12);
    tau_max <<  robot.joint_tau_max_array[0], robot.joint_tau_max_array[1], robot.joint_tau_max_array[2],
                robot.joint_tau_max_array[0], robot.joint_tau_max_array[1], robot.joint_tau_max_array[2],
                robot.joint_tau_max_array[0], robot.joint_tau_max_array[1], robot.joint_tau_max_array[2],
                robot.joint_tau_max_array[0], robot.joint_tau_max_array[1], robot.joint_tau_max_array[2];
    tau_min <<  -robot.joint_tau_max_array[0], -robot.joint_tau_max_array[1], -robot.joint_tau_max_array[2],
                -robot.joint_tau_max_array[0], -robot.joint_tau_max_array[1], -robot.joint_tau_max_array[2],
                -robot.joint_tau_max_array[0], -robot.joint_tau_max_array[1], -robot.joint_tau_max_array[2],
                -robot.joint_tau_max_array[0], -robot.joint_tau_max_array[1], -robot.joint_tau_max_array[2];

    LCMExchanger lcmExch;
    lcmExch.start_exchanger();
    bool enable = false;
    bool reset = true;

    LegData leg_cmd;

    MatrixXd ref_grf(3, 4);
    MatrixXd ref_pos(3, 4);
    MatrixXd ref_vel(3, 4);
    MatrixXd ref_acc(3, 4);

    ref_acc.setZero();

    VectorXd cur_euler(3);
    VectorXd cur_theta(12);
    VectorXd cur_omega(12);
    VectorXd cur_tau(12);
    VectorXd ref_theta(12);
    VectorXd ref_omega(12);
    VectorXd ref_tau(12);
    VectorXd motor_kp(12);
    VectorXd motor_kd(12);
    Vector4i phase_signal;

    ref_theta.setZero(12);
    ref_omega.setZero(12);

    motor_kp.setZero(12);
    motor_kd.setZero(12);

    std::this_thread::sleep_for(10ms);
    lcmExch.getServoStateData(cur_theta, cur_omega, cur_tau);

    LegControl leg_control;

    bool first = true;

    cout << "[Leg Controller]: Started" << endl;

    while(true)
    {
        // Calculating current time
        const auto start{ now() };

        // Put your code here
        // -----------------------------------------------
        
        lcmExch.getWbcCmdData(leg_cmd);
        lcmExch.getImuData(cur_euler);
        lcmExch.getServoStateData(cur_theta, cur_omega, cur_tau);
        lcmExch.getEnableData(enable, reset);
        phase_signal = lcmExch.getGaitPhaseSignal();
        
        if (enable == true)
        {
            first = true;

            leg_control.calculate(leg_cmd, cur_theta, cur_omega, ref_theta, ref_omega, ref_tau);
            ref_tau = vbmath::clip(ref_tau, tau_min, tau_max) * robot.gear_ratio / robot.kt;
            
            set_motor_gains(phase_signal, 
                    servo_kp_swing, servo_kp_stance,
                    servo_kd_swing, servo_kd_stance, 
                    motor_kp, motor_kd);
            // cout << ref_theta.head(3).transpose() << endl;
            
            lcmExch.sendServoCmd(ref_theta, ref_omega, ref_tau, motor_kp, motor_kd);

        }
        else
        {
            if (first == true)
            {
                ref_tau.setZero();
                motor_kp.setZero();
                motor_kd.setZero();
                first = false;
                lcmExch.sendServoCmd(ref_theta, ref_omega, ref_tau, motor_kp, motor_kd);
            }
        }
        // -----------------------------------------------

        // Wait until spinning time
        while(true)
        {
            std::chrono::duration<double, std::milli> elapsed{now() - start};
            if (elapsed >= dt)
            {
                // cout << "Waited for : " << elapsed.count() << " ms" << endl;
                break;
            }
        }
    }

    return 0;
}