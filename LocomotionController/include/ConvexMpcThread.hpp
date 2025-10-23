#ifndef _convex_mpc_thread_hpp_
#define _convex_mpc_thread_hpp_

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <chrono>
#include <thread>

#include <vbmath.hpp>
#include "structs.hpp"
#include "system_functions.hpp"
#include "ConvexMpc.hpp"
#include "SimpleGaitScheduler.hpp"

using namespace Eigen;
using namespace std;

 
class ConvexMPCThread{
public:
    ConvexMPCThread();
    ~ConvexMPCThread();
    auto now();

    void callback();
    void start_thread();

    void set_physical_params(RobotPhysicalParams& robot);
    void set_mpc_params(double timestep, int horizon, double friction_coeff,
                        double f_min, double f_max, VectorXd &Q, VectorXd &R);
    void set_gait_params(double T_st,
                        double T_sw,
                        const std::vector<double>& phase_offsets,
                        const std::vector<int>& phase_init);
    void set_observation_data(RobotData& robot_state, LegData& leg_state, VectorXd& x_ref, 
                        MatrixXd& R_body, bool& en, bool& standing, std::vector<int>& phase_signal, 
                        double& phi0, vector<bool> active_legs);
    VectorXd get_ref_grf();

private:
    ConvexMPC mpc;
    SimpleGaitScheduler gait_scheduler;

    RobotData robot_state;
    LegData leg_state;
    VectorXd x_ref, des_state;
    VectorXd rpy_rate, com_vel_body_frame;
    MatrixXd R_body, R_z;
    bool en;
    bool standing;
    std::vector<int> phase_signal;
    double phi0;

    VectorXd ref_grf;

    VectorXd x0;
    MatrixXd foot_positions;
    vector<int> gait_table;
    double module_dt;
    std::chrono::duration<double> dt;
    vector<bool> active_legs;

    RobotPhysicalParams robot;

    double cos_yaw, sin_yaw;
    double pre_yaw;
};

#endif //_convex_mpc_thread_hpp_