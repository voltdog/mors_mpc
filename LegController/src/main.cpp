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

#define MOTOR_KP0 20.0
#define MOTOR_KP1 16.0
#define MOTOR_KP2 16.0

#define MOTOR_KD0 0.4
#define MOTOR_KD1 0.4
#define MOTOR_KD2 0.4

// current time
auto now() 
{
  return std::chrono::steady_clock::now(); 
}

int main() {
    cout << "[Leg Controller]: Starting..." << endl;
    // load config
    string config_address = mors_sys::GetEnv("CONFIGPATH");
    string robot_config_address = config_address + "/robot_config.yaml";
    cout << robot_config_address << endl;

    YAML::Node robot_config = YAML::LoadFile(robot_config_address);
    RobotPhysicalParams robot;
    robot.bx = robot_config["bx"].as<double>(); 
    robot.by = robot_config["by"].as<double>(); 
    robot.m1 = robot_config["m1"].as<double>(); 
    robot.m2 = robot_config["m2"].as<double>(); 
    robot.m3 = robot_config["m3"].as<double>(); 
    robot.l1 = robot_config["l1"].as<double>(); 
    robot.l2 = robot_config["l2"].as<double>(); 
    robot.l3 = robot_config["l3"].as<double>(); 
    robot.d1 = robot_config["d1"].as<double>(); 
    robot.d2 = robot_config["d2"].as<double>(); 
    robot.d3 = robot_config["d3"].as<double>(); 
    robot.l_cz_2 = robot_config["Pc2"][2].as<double>(); 
    robot.l_cx_3 = robot_config["Pc3"][0].as<double>(); 
    robot.g = robot_config["g"].as<double>(); 
    robot.kt = robot_config["kt"].as<double>(); 
    robot.gear_ratio = robot_config["gear_ratio"].as<double>(); 

    robot.tau_max_array[0] = robot_config["tau_max"][0].as<double>(); 
    robot.tau_max_array[1] = robot_config["tau_max"][1].as<double>(); 
    robot.tau_max_array[2] = robot_config["tau_max"][2].as<double>(); 

    string dt_config_address = config_address + "/timesteps.yaml";
    YAML::Node dt_config = YAML::LoadFile(dt_config_address);
    double module_dt = dt_config["leg_controller_dt"].as<double>(); 

    auto dt = std::chrono::duration<double>(module_dt);//1ms;
    VectorXd tau_max(12);
    VectorXd tau_min(12);
    tau_max <<  robot.tau_max_array[0], robot.tau_max_array[1], robot.tau_max_array[2],
                robot.tau_max_array[0], robot.tau_max_array[1], robot.tau_max_array[2],
                robot.tau_max_array[0], robot.tau_max_array[1], robot.tau_max_array[2],
                robot.tau_max_array[0], robot.tau_max_array[1], robot.tau_max_array[2];
    tau_min <<  -robot.tau_max_array[0], -robot.tau_max_array[1], -robot.tau_max_array[2],
                -robot.tau_max_array[0], -robot.tau_max_array[1], -robot.tau_max_array[2],
                -robot.tau_max_array[0], -robot.tau_max_array[1], -robot.tau_max_array[2],
                -robot.tau_max_array[0], -robot.tau_max_array[1], -robot.tau_max_array[2];

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
    VectorXd motor_kp_ref(3);
    VectorXd motor_kd_ref(3);

    MatrixXd Kp_r1(3,3);
    MatrixXd Kp_l1(3,3);
    MatrixXd Kp_r2(3,3);
    MatrixXd Kp_l2(3,3);
    MatrixXd Kd_r1(3,3);
    MatrixXd Kd_l1(3,3);
    MatrixXd Kd_r2(3,3);
    MatrixXd Kd_l2(3,3);

    Vector4i phase_signal;

    ref_theta.setZero(12);
    ref_omega.setZero(12);

    motor_kp.setZero(12);
    motor_kd.setZero(12);
    motor_kp_ref(0) = MOTOR_KP0;
    motor_kp_ref(1) = MOTOR_KP1;
    motor_kp_ref(2) = MOTOR_KP2;
    motor_kd_ref(0) = MOTOR_KD0;
    motor_kd_ref(1) = MOTOR_KD1;
    motor_kd_ref(2) = MOTOR_KD2;

    Kp_r1.setZero();
    Kd_r1.setZero();
    Kp_l1.setZero();
    Kd_l1.setZero();
    Kp_r2.setZero();
    Kd_r2.setZero();
    Kp_l2.setZero();
    Kd_l2.setZero();

    std::this_thread::sleep_for(10ms);
    lcmExch.getServoStateData(cur_theta, cur_omega, cur_tau);

    LegControl leg_control;
    leg_control.set_leg_params(robot, cur_theta);    

    bool first = true;


    cout << "[Leg Controller]: Started" << endl;

    while(true)
    {
        // Calculating current time
        const auto start{ now() };

        // Put your code here
        // -----------------------------------------------
        
        lcmExch.getLegCmdData(leg_cmd);
        lcmExch.getImuData(cur_euler);
        lcmExch.getServoStateData(cur_theta, cur_omega, cur_tau);
        lcmExch.getEnableData(enable, reset);
        phase_signal = lcmExch.getPhaseSignal();
        
        if (enable == true)
        {
            first = true;
            
            Kp_r1 = leg_cmd.r1_kp.array().matrix().asDiagonal();
            Kd_r1 = leg_cmd.r1_kd.array().matrix().asDiagonal();
            Kp_l1 = leg_cmd.l1_kp.array().matrix().asDiagonal();
            Kd_l1 = leg_cmd.l1_kd.array().matrix().asDiagonal();
            Kp_r2 = leg_cmd.r2_kp.array().matrix().asDiagonal();
            Kd_r2 = leg_cmd.r2_kd.array().matrix().asDiagonal();
            Kp_l2 = leg_cmd.l2_kp.array().matrix().asDiagonal();
            Kd_l2 = leg_cmd.l2_kd.array().matrix().asDiagonal();

            leg_control.set_feedback_params(Kp_r1, Kd_r1, Kp_l1, Kd_l1, Kp_r2, Kd_r2, Kp_l2, Kd_l2);            
            ref_tau = leg_control.calculate(leg_cmd, cur_theta, cur_omega, cur_euler, phase_signal, ref_theta, ref_omega);
            ref_tau = vbmath::clip(ref_tau, tau_min, tau_max) * robot.gear_ratio / robot.kt;

            for (int i = 0; i < 3; i++)
            {
                if (leg_cmd.r1_kp(i) >= 0.01)
                {
                    motor_kp(i) = motor_kp_ref(i);
                    motor_kd(i) = motor_kd_ref(i);
                }
                else
                {
                    motor_kp(i) = 0;
                    motor_kd(i) = 0;
                }

                if (leg_cmd.l1_kp(i) >= 0.01)
                {
                    motor_kp(i+3) = motor_kp_ref(i);
                    motor_kd(i+3) = motor_kd_ref(i);
                }
                else
                {
                    motor_kp(i+3) = 0;
                    motor_kd(i+3) = 0;
                }

                if (leg_cmd.r2_kp(i) >= 0.01)
                {
                    motor_kp(i+6) = motor_kp_ref(i);
                    motor_kd(i+6) = motor_kd_ref(i);
                }
                else
                {
                    motor_kp(i+6) = 0;
                    motor_kd(i+6) = 0;
                }

                if (leg_cmd.l2_kp(i) >= 0.01)
                {
                    motor_kp(i+9) = motor_kp_ref(i);
                    motor_kd(i+9) = motor_kd_ref(i);
                }
                else
                {
                    motor_kp(i+9) = 0;
                    motor_kd(i+9) = 0;
                }
            }

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