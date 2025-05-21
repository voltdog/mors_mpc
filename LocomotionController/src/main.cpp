#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include "CommandShaper.hpp"
#include "ContactStateFSM.hpp"
#include "ConvexMpcThread.hpp"
#include "SwingController.hpp"
#include "LcmDataExchange.hpp"
#include "SimpleGaitScheduler.hpp"
#include "GaitTransition.hpp"
#include "structs.hpp"
#include "system_functions.hpp"
#include <vbmath.hpp>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace Eigen;
using namespace YAML;
using namespace std::chrono;

// current time
auto now() 
{
  return std::chrono::steady_clock::now(); 
}


int main() {
    cout << "[LocomotionController]: Starting..." << endl;
    // load config
    string config_address = mors_sys::GetEnv("CONFIGPATH");
    string robot_config_address = config_address + "/robot_config.yaml";

    // robot physical params
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

    // mpc params
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

    // gait scheduler params
    string gait_scheduler_config_address = config_address + "/gait_scheduler.yaml";
    YAML::Node gait_sched_config = YAML::LoadFile(gait_scheduler_config_address);
    double start_td_detecting = gait_sched_config["start_td_detecting"].as<double>(); 

    // swing controller params
    string swing_controller_config_address = config_address + "/swing_controller.yaml";
    YAML::Node swing_controller_config = YAML::LoadFile(swing_controller_config_address);
    array<double, 4> interleave_x; 
    array<double, 4>interleave_y;
    for (int i=0; i<4; i++)
    {
        interleave_x[i] = swing_controller_config["interleave_x"][i].as<double>(); 
        interleave_y[i] = swing_controller_config["interleave_y"][i].as<double>(); 
    }
    double dz_near_ground = swing_controller_config["dz_near_ground"].as<double>(); 
    double k1_fsp = swing_controller_config["k1_fsp"].as<double>(); 
    double k2_fsp = swing_controller_config["k2_fsp"].as<double>(); 

    // leg control params
    string imp_config_address = config_address + "/imp_config.yaml";
    YAML::Node imp_config = YAML::LoadFile(imp_config_address);
    VectorXd leg_kp(3);
    VectorXd leg_kd(3);
    for (int i=0; i<3; i++)
    {
        leg_kp[i] = imp_config["Kp"][i].as<double>();
        leg_kd[i] = imp_config["Kd"][i].as<double>();
    }

    // timesteps duration
    string dt_config_address = config_address + "/timesteps.yaml";
    YAML::Node dt_config = YAML::LoadFile(dt_config_address);
    double module_dt = dt_config["locomotion_controller"].as<double>(); 
    double mpc_dt = dt_config["stance_controller_dt"].as<double>(); 
    // double module_freq = 1/module_dt;
    // double mpc_freq = 1/mpc_dt;

    auto dt = std::chrono::duration<double>(module_dt);//1ms;

    // init LCM
    LCMExchanger lcmExch;
    lcmExch.start_exchanger();

    // init data variables
    RobotData robot_state;
    RobotData robot_cmd, robot_ref;
    LegData leg_state;
    LegData leg_cmd;

    leg_cmd.r1_grf.resize(3);
    leg_cmd.l1_grf.resize(3);
    leg_cmd.r2_grf.resize(3);
    leg_cmd.l2_grf.resize(3);
    leg_cmd.r1_pos.resize(3);
    leg_cmd.l1_pos.resize(3);
    leg_cmd.r2_pos.resize(3);
    leg_cmd.l2_pos.resize(3);
    leg_cmd.r1_vel.resize(3);
    leg_cmd.l1_vel.resize(3);
    leg_cmd.r2_vel.resize(3);
    leg_cmd.l2_vel.resize(3);
    leg_cmd.r1_acc.resize(3);
    leg_cmd.l1_acc.resize(3);
    leg_cmd.r2_acc.resize(3);
    leg_cmd.l2_acc.resize(3);
    leg_cmd.r1_kp.resize(3);
    leg_cmd.l1_kp.resize(3);
    leg_cmd.r2_kp.resize(3);
    leg_cmd.l2_kp.resize(3);
    leg_cmd.r1_kd.resize(3);
    leg_cmd.l1_kd.resize(3);
    leg_cmd.r2_kd.resize(3);
    leg_cmd.l2_kd.resize(3);
    leg_state.r1_pos.resize(3);
    leg_state.l1_pos.resize(3);
    leg_state.r2_pos.resize(3);
    leg_state.l2_pos.resize(3);
    leg_state.r1_vel.resize(3);
    leg_state.l1_vel.resize(3);
    leg_state.r2_vel.resize(3);
    leg_state.l2_vel.resize(3);
    leg_state.r1_acc.resize(3);
    leg_state.l1_acc.resize(3);
    leg_state.r2_acc.resize(3);
    leg_state.l2_acc.resize(3);
    leg_state.r1_grf.resize(3);
    leg_state.l1_grf.resize(3);
    leg_state.r2_grf.resize(3);
    leg_state.l2_grf.resize(3);
    leg_state.r1_kp.resize(3);
    leg_state.l1_kp.resize(3);
    leg_state.r2_kp.resize(3);
    leg_state.l2_kp.resize(3);
    leg_state.r1_kd.resize(3);
    leg_state.l1_kd.resize(3);
    leg_state.r2_kd.resize(3);
    leg_state.l2_kd.resize(3);
    robot_cmd.ang_vel.resize(3);
    robot_cmd.lin_vel.resize(3);
    robot_cmd.orientation.resize(3);
    robot_cmd.pos.resize(3);
    robot_state.ang_vel.resize(3);
    robot_state.lin_vel.resize(3);
    robot_state.orientation.resize(3);
    robot_state.pos.resize(3);
    

    // gait generator params
    double t_sw = 0.2;
    double t_st = 0.3;
    std::vector<double> phase_offsets = {0.0, 0.0, 0.0, 0.0};
    bool standing = true;
    std::vector<int> phase_init = {STANCE, STANCE, STANCE, STANCE};
    double stride_height = 0.06;

    SimpleGaitScheduler gait_scheduler(module_dt);
    gait_scheduler.set_gait_params(t_st, t_sw, phase_offsets, phase_init);
    gait_scheduler.setMpcParams(mpc_dt, horizon);
    gait_scheduler.reset();
    gait_scheduler.reset_mpc_table();
    vector<int> gait_table(4 * horizon);
    vector<int> gait_phase(4);
    vector<double> gait_phi(4);
    // double t_gait;

    VectorXd phi;
    vector<double> phi_cur;
    vector<int> phase_signal;
    phi.resize(4);
    phi.setZero();
    std::vector<Eigen::Vector3d> foot_pos_local(4);
    std::vector<Eigen::Vector3d> foot_pos_global(4);
    std::vector<int> desired_leg_state = {STANCE, STANCE, STANCE, STANCE};
    std::vector<double> leg_phi = {0,0,0,0};

    // init contact state fsm
    ContactStateFSM contact_fsm(start_td_detecting);
    // init command shaper
    int adaptation_type = 0;
    CommandShaper cmd_shaper(module_dt, 1.0);
    cmd_shaper.set_body_adaptation_mode(adaptation_type);

    // swing controller
    VectorXd base_rpy_rate(3);
    VectorXd v(3);
    SwingController swing_controller(module_dt, 
                                    robot.bx, 
                                    robot.by, 
                                    robot.l1,
                                    interleave_x, // сделать правильное чтение данных из конфига
                                    interleave_y,
                                    dz_near_ground, 
                                    k1_fsp, 
                                    k2_fsp);
    std::vector<Eigen::Vector3d> p_ref(4), dp_ref(4), ddp_ref(4);
    Vector3d ref_body_vel;
    Vector3d base_pos;
    Vector3d base_lin_vel;

    double t = 0.0;
    vector<bool> active_legs = {true, true, true, true};
    bool enable = false;

    // ConvexMPC mpc;
    VectorXd x0(13);
    VectorXd x_ref(13);
    MatrixXd foot_positions(3, 4);
    MatrixXd R_body(3,3);
    MatrixXd inv_R_body(3,3);
    VectorXd grf_cmd(12);
    double phi0 = 0.0;
    // ConvexMPC mpc_thread;
    // mpc.set_physical_params(robot);
    // mpc.set_mpc_params(mpc_dt, horizon, friction, f_min, f_max, Q_vec, R_vec);
    ConvexMPCThread mpc_thread;
    mpc_thread.set_physical_params(robot);
    mpc_thread.set_gait_params(t_st, t_sw, phase_offsets, phase_init);
    mpc_thread.set_mpc_params(mpc_dt, horizon, friction, f_min, f_max, Q_vec, R_vec);
    mpc_thread.set_observation_data(robot_state, 
                                    leg_state, 
                                    x_ref, 
                                    R_body, 
                                    enable, 
                                    standing, 
                                    phase_signal, 
                                    phi0,
                                    active_legs);
    
    mpc_thread.start_thread();

    

    // gait transition
    GaitTransition gait_transition;
    gait_transition.set_gait_params(t_st, t_sw, phase_offsets);
    gait_transition.set_transition_duration(1.0);
    

    
    // int mpc_it = 0;

    cout << "[LocomotionController]: Started" << endl;

    while(true)
    {
        // Calculating current time
        const auto start{ now() };

        // Put your code here
        // -----------------------------------------------

        // ------------------
        // READ LCM
        // ------------------
        robot_cmd = lcmExch.getRobotCmd();
        robot_state = lcmExch.getBodyState();
        leg_state = lcmExch.getLegState();
        lcmExch.get_gait_params(t_st, t_sw, phase_offsets, standing, stride_height);
        enable = lcmExch.get_enable();
        lcmExch.get_active_legs(active_legs);
        adaptation_type = lcmExch.get_adaptation_type();
        
        if (enable == true)
        {
            // ------------------
            // GAIT SCHEDULER
            // ------------------
            // cout << "1" << endl;
            // gait_scheduler.reset_mpc_table();
            // gait transition
            
            gait_transition.set_gait_params(t_st, t_sw, phase_offsets);
            gait_transition.make_transition(t, t_st, t_sw, phase_offsets);
            // cout << t_sw << " " << t_st << endl;

            // scheduling
            gait_scheduler.set_gait_params(t_st, t_sw, phase_offsets, phase_init);
            gait_scheduler.step(t, standing, desired_leg_state, leg_phi);
            phase_signal = contact_fsm.step(leg_state.contacts, leg_phi, desired_leg_state);

            // ------------------
            // COMMAND SHAPER
            // ------------------
            // calc foot pos local
            R_body = mors_sys::euler2mat(robot_state.orientation(X), robot_state.orientation(Y), 0.0);//robot_state.orientation(Z));
            foot_pos_local[0] = R_body * leg_state.r1_pos;
            foot_pos_local[1] = R_body * leg_state.l1_pos;
            foot_pos_local[2] = R_body * leg_state.r2_pos;
            foot_pos_local[3] = R_body * leg_state.l2_pos;

            // calc foot pos global
            for (int i = 0; i < 4; i++)
                foot_pos_global[i] = foot_pos_local[i] + robot_state.pos;

            // step command shaper
            cmd_shaper.set_body_adaptation_mode(adaptation_type);
            x_ref = cmd_shaper.step(phase_signal, 
                                foot_pos_global, 
                                foot_pos_local, 
                                robot_cmd.lin_vel, 
                                robot_cmd.ang_vel[Z], 
                                robot_cmd.pos[Z]);
            R_body = mors_sys::euler2mat(robot_state.orientation(X), robot_state.orientation(Y), robot_state.orientation(Z));
            // ------------------
            // STANCE CONTROLLER
            // ------------------
            // if (static_cast<int>(module_freq/mpc_freq) == mpc_it)
            // {
            //     mpc_it = 0;

            //     // form x0 vector
            //     x0 <<   robot_state.orientation(X),
            //             robot_state.orientation(Y),
            //             robot_state.orientation(Z),
            //             robot_state.pos(X),
            //             robot_state.pos(Y),
            //             robot_state.pos(Z),
            //             robot_state.ang_vel(X),
            //             robot_state.ang_vel(Y),
            //             robot_state.ang_vel(Z),
            //             robot_state.lin_vel(X),
            //             robot_state.lin_vel(Y),
            //             robot_state.lin_vel(Z),
            //             -robot.g;

            //     // form foot_positions vector
                
            //     foot_positions.col(0) = R_body * leg_state.r1_pos;
            //     foot_positions.col(1) = R_body * leg_state.l1_pos;
            //     foot_positions.col(2) = R_body * leg_state.r2_pos;
            //     foot_positions.col(3) = R_body * leg_state.l2_pos;

            //     // predict future contact states
            //     gait_table = gait_scheduler.getMpcTable(t, standing, phase_signal, leg_phi);

            //     // solve mpc problem
            //     grf_cmd = mpc_thread.get_contact_forces(x0, x_ref, foot_positions, gait_table);
            // }
            // mpc_it += 1;
            // cout << "1" << endl;
            phi0 = gait_scheduler.get_phi();
            mpc_thread.set_gait_params(t_st, t_sw, phase_offsets, phase_init);
            mpc_thread.set_observation_data(robot_state, 
                                        leg_state, 
                                        x_ref, 
                                        R_body, 
                                        enable, 
                                        standing, 
                                        phase_signal, 
                                        phi0, 
                                        active_legs);
            grf_cmd = mpc_thread.get_ref_grf();

            // ------------------
            // SWING CONTROLLER
            // ------------------
            inv_R_body = R_body.inverse();
            v << robot_state.ang_vel(X), robot_state.ang_vel(Y), robot_state.ang_vel(Z);
            base_rpy_rate = inv_R_body * v;

            foot_pos_local[0] = R_body * leg_state.r1_pos;
            foot_pos_local[1] = R_body * leg_state.l1_pos;
            foot_pos_local[2] = R_body * leg_state.r2_pos;
            foot_pos_local[3] = R_body * leg_state.l2_pos;

            // calc foot pos global
            for (int i = 0; i < 4; i++)
                foot_pos_global[i] = foot_pos_local[i] + robot_state.pos;

            // convert data to swing controller format
            ref_body_vel << x_ref(9), x_ref(10), x_ref(11);
            base_pos << robot_state.pos(X), robot_state.pos(Y), robot_state.pos(Z);
            base_lin_vel << robot_state.lin_vel(X), robot_state.lin_vel(Y), robot_state.lin_vel(Z);

            swing_controller.set_gait_params(t_sw, t_st, stride_height);
            auto [p_ref, dp_ref, ddp_ref] = swing_controller.step(phase_signal,
                                                                    leg_phi,
                                                                    // x_ref(5),
                                                                    robot_cmd.pos(Z),
                                                                    x_ref(8),
                                                                    ref_body_vel,
                                                                    base_pos, 
                                                                    base_lin_vel,
                                                                    base_rpy_rate,
                                                                    R_body,
                                                                    foot_pos_global);

            // convert grf_cmd to convinient format
            if (active_legs[R1] == false)//phase_signal[R1] == SWING)
                leg_cmd.r1_grf.setZero();
            else
                leg_cmd.r1_grf = grf_cmd.segment(0, 3);

            if (active_legs[L1] == false)//phase_signal[L1] == SWING)
                leg_cmd.l1_grf.setZero();
            else  
                leg_cmd.l1_grf = grf_cmd.segment(3, 3);

            if (active_legs[R2] == false)//phase_signal[R2] == SWING)
                leg_cmd.r2_grf.setZero();
            else
                leg_cmd.r2_grf = grf_cmd.segment(6, 3);
            
            if (active_legs[L2] == false)//phase_signal[L2] == SWING)
                leg_cmd.l2_grf.setZero();
            else
                leg_cmd.l2_grf = grf_cmd.segment(9, 3);
            // convert desired leg pos, vel and acc
            leg_cmd.r1_pos = p_ref[R1];
            leg_cmd.l1_pos = p_ref[L1];
            leg_cmd.r2_pos = p_ref[R2];
            leg_cmd.l2_pos = p_ref[L2];
            leg_cmd.r1_vel = dp_ref[R1];
            leg_cmd.l1_vel = dp_ref[L1];
            leg_cmd.r2_vel = dp_ref[R2];
            leg_cmd.l2_vel = dp_ref[L2];
            leg_cmd.r1_acc = ddp_ref[R1];
            leg_cmd.l1_acc = ddp_ref[L1];
            leg_cmd.r2_acc = ddp_ref[R2];
            leg_cmd.l2_acc = ddp_ref[L2];
            
            if (phase_signal[R1] == SWING || phase_signal[R1] == LATE_CONTACT)
            {
                leg_cmd.r1_kp = leg_kp;
                leg_cmd.r1_kd = leg_kd;
            }
            else
            {
                leg_cmd.r1_kp.setZero();
                leg_cmd.r1_kd.setZero();
            }

            if (phase_signal[L1] == SWING || phase_signal[L1] == LATE_CONTACT)
            {
                leg_cmd.l1_kp = leg_kp;
                leg_cmd.l1_kd = leg_kd;
            }
            else
            {
                leg_cmd.l1_kp.setZero();
                leg_cmd.l1_kd.setZero();
            }

            if (phase_signal[R2] == SWING || phase_signal[R2] == LATE_CONTACT)
            {
                leg_cmd.r2_kp = leg_kp;
                leg_cmd.r2_kd = leg_kd;
            }
            else
            {
                leg_cmd.r2_kp.setZero();
                leg_cmd.r2_kd.setZero();
            }

            if (phase_signal[L2] == SWING || phase_signal[L2] == LATE_CONTACT)
            {
                leg_cmd.l2_kp = leg_kp;
                leg_cmd.l2_kd = leg_kd;
            }
            else
            {
                leg_cmd.l2_kp.setZero();
                leg_cmd.l2_kd.setZero();
            }
            // ------------------
            // SEND LCM
            // ------------------
            lcmExch.sendLegCmd(leg_cmd);
            lcmExch.sendPhaseSig(phase_signal, leg_phi, t);
            // lcmExch.sendPhaseSig(desired_leg_state, leg_phi, t);

            t += module_dt;
        }

        // cout << x_ref << endl;
        // -----------------------------------------------
        // std::chrono::duration<double, std::milli> elapsed_{now() - start};
        // cout << "[LocomotionController]: Waited for : " << elapsed_.count() << " ms" << endl;

        // Wait until spinning time
        while(true)
        {
            std::chrono::duration<double, std::milli> elapsed{now() - start};
            if (elapsed >= dt)
            {
                // if (elapsed.count() > 1000*module_dt+0.05)
                //     cout << "[LocomotionController]: Waited for : " << elapsed.count() << " ms" << endl;
                break;
            }
        }
    }

    return 0;
}