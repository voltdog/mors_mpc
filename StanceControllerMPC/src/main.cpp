#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include "lcm_data_exchange_sc.hpp"
#include "convex_mpc.hpp"
#include "SimpleGaitScheduler.hpp"
#include "structs.hpp"
#include <vbmath.hpp>
#include <yaml-cpp/yaml.h>
#include "system_functions.hpp"

using namespace std;
using namespace Eigen;
using namespace YAML;
using namespace std::chrono;

// #define SWING  0
// #define STANCE 1
// #define LATE   2

#define X 0
#define Y 1
#define Z 2

// #define K_Q 100
// #define K_R 0.001

// current time
auto now() 
{
  return std::chrono::steady_clock::now(); 
}

int main() {
    cout << "Stance Controller ConvexMPC starting..." << endl;
    // load config
    string config_address = mors_sys::GetEnv("CONFIGPATH");
    string robot_config_address = config_address + "/robot_config.yaml";

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
    robot.M_b = robot_config["M"].as<double>(); 

    robot.I_b.resize(3,3);
    for (int i=0; i<3; i++)
    {
        for (int j=0; j<3; j++)
            robot.I_b(i,j) = robot_config["I_b"][i][j].as<double>(); 
    }

    string mpc_config_address = config_address + "/stance_controller_mpc.yaml";
    YAML::Node mpc_config = YAML::LoadFile(mpc_config_address);
    int horizon = mpc_config["horizon"].as<int>();
    double friction = mpc_config["friction"].as<double>();
    double f_min = mpc_config["f_min"].as<double>();
    double f_max = mpc_config["f_max"].as<double>();
    VectorXd Q_vec(13);
    VectorXd R_vec(12);
    for (int i=0; i<13; i++)
        Q_vec(i) = mpc_config["Q"][i].as<double>(); 
    for (int i=0; i<12; i++)
        R_vec(i) = mpc_config["R"][i].as<double>(); 
    double Q_gain = mpc_config["Q_gain"].as<double>();
    double R_gain = mpc_config["R_gain"].as<double>();
    Q_vec *= Q_gain;
    R_vec *= R_gain;
    cout << "Q_gain: " << Q_gain << " | R_gain: " << R_gain << endl;

    string dt_config_address = config_address + "/timesteps.yaml";
    YAML::Node dt_config = YAML::LoadFile(dt_config_address);
    double module_dt = dt_config["stance_controller_dt"].as<double>(); 

    auto dt = std::chrono::duration<double>(module_dt);//1ms;
    VectorXd tau_max(12);
    VectorXd tau_min(12);

    LCMExchanger lcmExch;
    lcmExch.start_exchanger();

    RobotData robot_state;
    RobotData robot_cmd;
    LegData leg_state;

    

    LegData leg_cmd;
    leg_cmd.r1_grf.resize(3);
    leg_cmd.l1_grf.resize(3);
    leg_cmd.r2_grf.resize(3);
    leg_cmd.l2_grf.resize(3);
    robot_cmd.ang_vel.resize(3);
    robot_cmd.lin_vel.resize(3);
    robot_cmd.orientation.resize(3);
    robot_cmd.pos.resize(3);
    robot_state.ang_vel.resize(3);
    robot_state.lin_vel.resize(3);
    robot_state.orientation.resize(3);
    robot_state.pos.resize(3);

    ConvexMPC mpc;
    mpc.set_physical_params(robot);
    mpc.set_mpc_params(module_dt, horizon, friction, f_min, f_max, Q_vec, R_vec);

    VectorXd x0(13);
    VectorXd x_ref(13);
    MatrixXd foot_positions(3, 4);
    MatrixXd R_body(3,3);
    VectorXd grf_cmd(12);

    // gait generator for future phases
    // bounding
    // double t_sw = 0.25;
    // double t_st = 0.25;
    // std::vector<double> phase_offsets = {0.0, 0.0, 0.5, 0.5};
    // jumping
    // double t_sw = 0.25;
    // double t_st = 0.25;
    // std::vector<double> phase_offsets = {0.0, 0.0, 0.0, 0.0};
    // trot walk
    // double t_sw = 0.3;
    // double t_st = 0.3;
    // std::vector<double> phase_offsets = {0.0, 0.5, 0.5, 0.0};
    // standing
    double t_sw = 0.0;
    double t_st = 0.3;
    std::vector<double> phase_offsets = {0.0, 0.0, 0.0, 0.0};
    bool standing = true;
    // walk
    // double t_sw = 0.3;
    // double t_st = 1.1;
    // std::vector<double> phase_offsets = {0.0, 0.25, 0.5, 0.75};

    std::vector<int> phase_init = {STANCE, STANCE, STANCE, STANCE};
    // double dt_mpc = 0.01;
    // int mpc_horizon = 12;
    SimpleGaitScheduler gait_scheduler;
    gait_scheduler.set_gait_params(t_st, t_sw, phase_offsets, phase_init);
    gait_scheduler.setMpcParams(module_dt, horizon);
    vector<int> gait_table(4 * horizon);
    vector<int> gait_phase(4);
    vector<double> gait_phi(4);
    // vector<int> contact_states = {STANCE, STANCE, STANCE, STANCE};
    // contact_states << STANCE, STANCE, STANCE, STANCE;
    double t_gait;

    cout << "Stance Controller ConvexMPC started" << endl;

    while(true)
    {
        // Calculating current time
        const auto start{ now() };

        // Put your code here
        // -----------------------------------------------
        // read data from lcm
        robot_cmd = lcmExch.getRobotCmd();
        robot_state = lcmExch.getBodyState();
        leg_state = lcmExch.getLegsState();
        gait_phase = lcmExch.getPhaseSignals();
        t_gait = lcmExch.getTGait();
        gait_phi = lcmExch.getPhiGait();
        lcmExch.get_gait_params(t_st, t_sw, phase_offsets, standing);
        gait_scheduler.reset();
        // robot_cmd.pos(Z) = 0.5;


        // form x0 vector
        x0 <<   robot_state.orientation(X),
                robot_state.orientation(Y),
                robot_state.orientation(Z),
                robot_state.pos(X),
                robot_state.pos(Y),
                robot_state.pos(Z),
                robot_state.ang_vel(X),
                robot_state.ang_vel(Y),
                robot_state.ang_vel(Z),
                robot_state.lin_vel(X),
                robot_state.lin_vel(Y),
                robot_state.lin_vel(Z),
                -robot.g;
        // form x_ref vector
        x_ref << robot_cmd.orientation(X),
                robot_cmd.orientation(Y),
                robot_cmd.orientation(Z),
                robot_cmd.pos(X),
                robot_cmd.pos(Y),
                robot_cmd.pos(Z),
                robot_cmd.ang_vel(X),
                robot_cmd.ang_vel(Y),
                robot_cmd.ang_vel(Z),
                robot_cmd.lin_vel(X),
                robot_cmd.lin_vel(Y),
                robot_cmd.lin_vel(Z),
                -robot.g;

        // form foot_positions vector
        R_body = mors_sys::euler2mat(robot_state.orientation(X), robot_state.orientation(Y), robot_state.orientation(Z));
        foot_positions.col(0) = R_body * leg_state.r1_pos;
        foot_positions.col(1) = R_body * leg_state.l1_pos;
        foot_positions.col(2) = R_body * leg_state.r2_pos;
        foot_positions.col(3) = R_body * leg_state.l2_pos;
        
        // define contact states
        // for (int i=0; i<4; i++)
        // {
        //     if (gait_phase(i) == LATE)
        //         contact_states(i) = SWING;
        //     else
        //         contact_states(i) = gait_phase(i);
        // }
        gait_scheduler.set_gait_params(t_st, t_sw, phase_offsets, phase_init);
        gait_table = gait_scheduler.getMpcTable(t_gait, standing, gait_phase, gait_phi);
        // contact_states << STANCE, SWING, STANCE, SWING;
        // contact_states << 1, 1, 1, 0;
        // cout << contact_states.transpose() << endl;
        // cout << "----" << endl;
        // cout << standing << endl;
        // if (t_gait > 4.03 && t_gait < 4.07)
        // {
        //     cout << "t: " << t_gait << endl;
        //     for (int i=0; i < 4; i++)
        //     {
        //         cout << gait_phase[i] << " ";
        //     }
        //     cout << endl;
        //     cout << "-" << endl;
        //     for (int i=0; i < static_cast<int>(gait_table.size()); i++)
        //     {
        //         cout << gait_table[i] << " ";
        //         if ((i+1) % 4 == 0)
        //             cout << "| ";
        //     }
        //     cout << endl;
        //     cout << "===" << endl;
        // }

        // solve mpc problem
        grf_cmd = mpc.get_contact_forces(x0, x_ref, foot_positions, gait_table);


        // convert grf_cmd to convinient format
        leg_cmd.r1_grf = grf_cmd.segment(0, 3);
        leg_cmd.l1_grf = grf_cmd.segment(3, 3);
        leg_cmd.r2_grf = grf_cmd.segment(6, 3);
        leg_cmd.l2_grf = grf_cmd.segment(9, 3);
        // cout << leg_cmd.r1_grf(Z) << endl;

        lcmExch.sendGrfCmd(leg_cmd);

        // -----------------------------------------------
        std::chrono::duration<double, std::milli> elapsed_{now() - start};
        cout << "[cMPC]: Waited for : " << elapsed_.count() << " ms" << endl;

        // Wait until spinning time
        while(true)
        {
            std::chrono::duration<double, std::milli> elapsed{now() - start};
            if (elapsed >= dt)
            {
                // cout << "[cMPC]: Waited for : " << elapsed.count() << " ms" << endl;
                break;
            }
        }
    }

    return 0;
}