#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include "lcm_data_exchange_gs.hpp"
#include "CommandShaper.hpp"
#include "ContactStateFSM.hpp"
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

// current time
auto now() 
{
  return std::chrono::steady_clock::now(); 
}

int main() {
    cout << "[GaitScheduler]: starting..." << endl;
    // load config
    string config_address = mors_sys::GetEnv("CONFIGPATH");
    string gait_scheduler_config_address = config_address + "/gait_scheduler.yaml";

    YAML::Node gait_sched_config = YAML::LoadFile(gait_scheduler_config_address);
    // double ampl = gait_sched_config["ampl"].as<double>(); 
    // double alpha = gait_sched_config["alpha"].as<double>(); 
    // double lamb = gait_sched_config["lamb"].as<double>(); 
    // double a = gait_sched_config["a"].as<double>(); 
    double start_td_detecting = gait_sched_config["start_td_detecting"].as<double>(); 

    string dt_config_address = config_address + "/timesteps.yaml";
    YAML::Node dt_config = YAML::LoadFile(dt_config_address);
    double module_dt = dt_config["gait_scheduler_dt"].as<double>(); 

    auto dt = std::chrono::duration<double>(module_dt);//5ms;

    // init lcm
    LCMExchanger lcmExch;
    lcmExch.start_exchanger();

    

    // define variables
    RobotData body_state;
    RobotData robot_cmd, robot_ref;
    LegData leg_state;
    double t_st, t_sw, stride_height;
    vector<double> ref_gait = {M_PI, 0.0, 0.0, M_PI};
    bool standing;//, pre_standing;
    
    VectorXd phi;
    vector<double> phi_cur;
    // vector<bool> leg_contacts;
    // leg_contacts.resize(4);
    vector<int> phase_signal;
    // phase.resize(4);
    // phase.setZero();
    phi.resize(4);
    phi.setZero();
    robot_cmd.ang_vel.resize(3);
    robot_cmd.lin_vel.resize(3);
    robot_cmd.orientation.resize(3);
    robot_cmd.pos.resize(3);
    robot_cmd.ang_vel.setZero();
    robot_cmd.lin_vel.setZero();
    robot_cmd.orientation.setZero();
    robot_cmd.pos.setZero();

    MatrixXd R_body(3,3);
    std::vector<Eigen::Vector3d> foot_pos_local(4);
    std::vector<Eigen::Vector3d> foot_pos_global(4);
    VectorXd x_ref(13);

    // init gait scheduler
    t_sw = 0.25;
    t_st = 0.4;
    std::vector<double> phase_offsets = {0.0, 0.5, 0.5, 0.0};
    std::vector<int> phase_init = {0,0,0,0};
    double dt_mpc = 0.01;
    int mpc_horizon = 16;
    SimpleGaitScheduler gait_scheduler;
    gait_scheduler.set_gait_params(t_sw, t_st, phase_offsets, phase_init);
    gait_scheduler.setMpcParams(dt_mpc, mpc_horizon);

    std::vector<int> desired_leg_state = {STANCE, STANCE, STANCE, STANCE};
    std::vector<double> leg_phase = {0,0,0,0};

    // init contact state fsm
    ContactStateFSM contact_fsm(start_td_detecting);
    // init command shaper
    CommandShaper cmd_shaper(module_dt, 1.0);

    double t = 0.0;

    cout << "[GaitScheduler]: started" << endl;

    while(true)
    {
        // Calculating current time
        const auto start{ now() };

        // Put your code here
        // -----------------------------------------------
        // read data from lcm
        lcmExch.get_gait_params(t_st, t_sw, ref_gait, standing, stride_height);
        body_state = lcmExch.getBodyState();
        leg_state = lcmExch.getLegState();
        robot_ref = lcmExch.getRobotRef();

        // control
        // Gait scheduling

        // gait_scheduler.set_gait_params(w_sw, w_st, ref_gait);

        
        // cout << leg_phase[0] << endl;
        // cout << "1" << endl;
        
        
        // else if (standing == false && pre_standing == true)
        // {
        //     gait_scheduler.init_integrator(module_dt);
        // }

        

        if (standing == true)
        {
            phase_signal = {STANCE, STANCE, STANCE, STANCE};
            leg_phase = {0,0,0,0};
        }
        else
        {
            gait_scheduler.step(t, desired_leg_state, leg_phase);
            phase_signal = contact_fsm.step(leg_state.contacts, leg_phase, desired_leg_state);
        }

        // Command shaping
        // calc foot pos local
        R_body = mors_sys::euler2mat(body_state.orientation(X), body_state.orientation(Y), body_state.orientation(Z));
        foot_pos_local[0] = R_body * leg_state.r1_pos;
        foot_pos_local[1] = R_body * leg_state.l1_pos;
        foot_pos_local[2] = R_body * leg_state.r2_pos;
        foot_pos_local[3] = R_body * leg_state.l2_pos;

        // calc foot pos global
        for (int i = 0; i < 4; i++)
            foot_pos_global[i] = foot_pos_local[i] + body_state.pos;

        // step command shaper
        
        x_ref = cmd_shaper.step(phase_signal, 
                            foot_pos_global, 
                            foot_pos_local, 
                            robot_ref.lin_vel, 
                            robot_ref.ang_vel[Z], 
                            robot_ref.pos[Z]);

        robot_cmd.orientation = x_ref.segment(0, 3);
        robot_cmd.pos = x_ref.segment(3, 3);
        robot_cmd.ang_vel = x_ref.segment(6, 3);
        robot_cmd.lin_vel = x_ref.segment(9, 3);

        // send data to lcm
        // cout << standing << " | " << leg_phase[0] << endl;
        lcmExch.sendRobotCmd(robot_cmd);
        lcmExch.sendPhaseSig(phase_signal, leg_phase);

        // pre_standing = standing;

        t += module_dt;

        // -----------------------------------------------

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