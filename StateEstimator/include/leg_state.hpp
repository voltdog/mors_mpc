#ifndef _leg_state_hpp_
#define _leg_state_hpp_

#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <lcm/lcm-cpp.hpp>
#include "mors_msgs/imu_lcm_data.hpp"
#include "mors_msgs/servo_state_msg.hpp"
#include "mors_msgs/odometry_msg.hpp"
#include "mors_msgs/robot_state_msg.hpp"
#include "leg_model.hpp"
#include "system_functions.hpp"
#include <yaml-cpp/yaml.h>
#include "structs.hpp"

#define X 0
#define Y 1
#define Z 2

#define THRESHOLD 10.0;

using namespace std;
using namespace Eigen;
using namespace YAML;

class LegState
{
    public:
        LegState(RobotPhysicalParams &robot);
        ~LegState();

        void set_contact_threshold(double threshold);
        LegData get_leg_state(VectorXd theta, VectorXd d_theta, VectorXd tau);
        
    private:
        void calc_jacobians(VectorXd theta, VectorXd d_theta);
        void calc_pos_vel_acc(VectorXd theta, VectorXd d_theta);
        void calc_joint_space_matrices(VectorXd theta, VectorXd d_theta);
        void calc_grf(VectorXd tau);
        void calc_contacts();
        VectorXd inv_dyn_force_observer(VectorXd tau, MatrixXd J, VectorXd G, VectorXd V);

        LegData leg_data;
        LegModel leg_model;

        MatrixXd J_R1, J_R2, J_L1, J_L2;
        MatrixXd dJ_R1, dJ_R2, dJ_L1, dJ_L2;

        VectorXd X_R1, X_L1, X_R2, X_L2;
        VectorXd dX_R1, dX_L1, dX_R2, dX_L2;
        VectorXd ddX_R1, ddX_L1, ddX_R2, ddX_L2;
        VectorXd r1_offset, l1_offset, r2_offset, l2_offset;
        VectorXd d_theta_r1, d_theta_l1, d_theta_r2, d_theta_l2;

        MatrixXd M_R1, M_L1, M_R2, M_L2;
        VectorXd V_R1, V_L1, V_R2, V_L2;
        VectorXd G_R1, G_L1, G_R2, G_L2;
        VectorXd F_R1, F_L1, F_R2, F_L2;

        MatrixXd invJ_T;
        VectorXd f_hat;
        VectorXd f_hat_r1, f_hat_l1, f_hat_r2, f_hat_l2;
        VectorXd tau_r1, tau_l1, tau_r2, tau_l2;

        VectorXd contact;

        double threshold;

};

#endif //_leg_state_hpp_