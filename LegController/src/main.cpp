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

// #define MOTOR_KP0 0.0
// #define MOTOR_KP1 0.0
// #define MOTOR_KP2 0.0

// #define MOTOR_KD0 0.0
// #define MOTOR_KD1 0.0
// #define MOTOR_KD2 0.0

// current time
auto now() 
{
  return std::chrono::steady_clock::now(); 
}

// std::string GetEnv( const std::string & var ) {
//     const char * val = std::getenv( var.c_str() );
//     if ( val == nullptr ) { // invalid to assign nullptr to std::string
//         return "";
//     }
//     else {
//         return val;
//     }
// }

int main() {
    cout << "Leg Controller starting..." << endl;
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
    // int control_type = 0;

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
    // motor_kp.setConstant(MOTOR_KP);
    // motor_kd.setConstant(MOTOR_KD);
    // for (int i = 0; i < 3; i++)
    // {
        // motor_kp(i) = MOTOR_KP;
        // motor_kd(i) = MOTOR_KD;
    // }
    motor_kp(0) = MOTOR_KP0;
    motor_kp(1) = MOTOR_KP1;
    motor_kp(2) = MOTOR_KP2;
    motor_kd(0) = MOTOR_KD0;
    motor_kd(1) = MOTOR_KD1;
    motor_kd(2) = MOTOR_KD2;

    Kp_r1.setZero();
    Kd_r1.setZero();
    Kp_l1.setZero();
    Kd_l1.setZero();
    Kp_r2.setZero();
    Kd_r2.setZero();
    Kp_l2.setZero();
    Kd_l2.setZero();

    LegControl leg_control;
    leg_control.set_leg_params(robot);
                                

    bool first = true;


    cout << "Leg Controller started" << endl;

    while(true)
    {
        // Calculating current time
        const auto start{ now() };

        // Put your code here
        // -----------------------------------------------
        
        lcmExch.getLegCmdData(leg_cmd);
        lcmExch.getImuData(cur_euler);
        // cur_euler << 0.0, 0.0, 0.0;
        lcmExch.getServoStateData(cur_theta, cur_omega, cur_tau);
        lcmExch.getEnableData(enable, reset);
        // control_type = lcmExch.getCtrlTypeData();
        phase_signal = lcmExch.getPhaseSignal();
        
        // cout << "control_type: " << control_type << endl;
        // enable = true;
        // control_type = LEG_CONTROL;
        if (enable == true)// && control_type == LEG_CONTROL)
        {
            // auto start_alg = high_resolution_clock::now();

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
            ref_tau = leg_control.calculate(leg_cmd, cur_theta, cur_omega, cur_euler, phase_signal, ref_theta);
            ref_tau = vbmath::clip(ref_tau, tau_min, tau_max) * robot.gear_ratio / robot.kt;

            if (abs(cur_theta[2]) - 0.3 < 0.0 || cur_theta[2] < 0.0)
            {
                ref_tau(2) = 20.5;
                // cout << "otstoi" << endl;
            }

            // ref_tau.setZero();
            // cout << cur_theta[2] << endl;
            // if (abs(cur_theta[5] - 0.08) < 0.0 || cur_theta[5] > 0.0)
            //     ref_tau(5) = -0.5;
            // if (abs(cur_theta[8] - 0.08) < 0.0 || cur_theta[8] > 0.0)
            //     ref_tau(8) = -0.5;
            // if (abs(cur_theta[11] - 0.08) < 0.0 || cur_theta[11] < 0.0)
            //     ref_tau(11) = 0.5;

            // cout << leg_cmd.r1_grf << endl;

            // auto stop = high_resolution_clock::now();
            // auto duration = duration_cast<microseconds>(stop - start_alg);
            // cout << duration.count() << endl;

            // for (int i = 0; i < 3; i++)
            // {
            //     motor_kp(i) = MOTOR_KP;
            //     motor_kd(i) = MOTOR_KD;
            // }

            if (leg_cmd.r1_kp(0) >= 0.1)
            {
                motor_kp(0) = MOTOR_KP0;
                motor_kp(1) = MOTOR_KP1;
                motor_kp(2) = MOTOR_KP2;
                motor_kd(0) = MOTOR_KD0;
                motor_kd(1) = MOTOR_KD1;
                motor_kd(2) = MOTOR_KD2;
            }
            else
            {
                motor_kp(0) = 0;
                motor_kp(1) = 0;
                motor_kp(2) = 0;
                motor_kd(0) = 0;
                motor_kd(1) = 0;
                motor_kd(2) = 0;
            }

            if (leg_cmd.l1_kp(0) >= 0.1)
            {
                motor_kp(3) = MOTOR_KP0;
                motor_kp(4) = MOTOR_KP1;
                motor_kp(5) = MOTOR_KP2;
                motor_kd(3) = MOTOR_KD0;
                motor_kd(4) = MOTOR_KD1;
                motor_kd(5) = MOTOR_KD2;
            }
            else
            {
                motor_kp(3) = 0;
                motor_kp(4) = 0;
                motor_kp(5) = 0;
                motor_kd(3) = 0;
                motor_kd(4) = 0;
                motor_kd(5) = 0;
            }

            if (leg_cmd.r2_kp(0) >= 0.1)
            {
                motor_kp(6) = MOTOR_KP0;
                motor_kp(7) = MOTOR_KP1;
                motor_kp(8) = MOTOR_KP2;
                motor_kd(6) = MOTOR_KD0;
                motor_kd(7) = MOTOR_KD1;
                motor_kd(8) = MOTOR_KD2;
            }
            else
            {
                motor_kp(6) = 0;
                motor_kp(7) = 0;
                motor_kp(8) = 0;
                motor_kd(6) = 0;
                motor_kd(7) = 0;
                motor_kd(8) = 0;
            }

            if (leg_cmd.l2_kp(0) >= 1.0)
            {
                motor_kp(9) = MOTOR_KP0;
                motor_kp(10) = MOTOR_KP1;
                motor_kp(11) = MOTOR_KP2;
                motor_kd(9) = MOTOR_KD0;
                motor_kd(10) = MOTOR_KD1;
                motor_kd(11) = MOTOR_KD2;
            }
            else
            {
                motor_kp(9) = 0;
                motor_kp(10) = 0;
                motor_kp(11) = 0;
                motor_kd(9) = 0;
                motor_kd(10) = 0;
                motor_kd(11) = 0;
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