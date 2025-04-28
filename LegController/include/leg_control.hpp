 #ifndef _leg_control_hpp_
#define _leg_control_hpp_

#include <iostream>
#include <Eigen/Dense>
#include <vbmath.hpp>
#include "structs.hpp"
#include "leg_model.hpp"
// #include <lcm/lcm-cpp.hpp>
// #include "lcm_msgs/mors_msgs/leg_cmd_msg.hpp"

using namespace Eigen;
using namespace std;

#define R1 0
#define L1 1
#define R2 2
#define L2 3

#define SWING  0
#define STANCE 1
#define LATE   2
#define EARLY_CONTACT   3

class LegControl{
public:
    LegControl();

    void set_leg_params(RobotPhysicalParams &robot);
    void set_feedback_params(MatrixXd& Kp, MatrixXd& Kd);

    // MatrixXd jacobian_R1(double q1, double q2, double q3);
    // MatrixXd jacobian_R2(double q1, double q2, double q3);
    // MatrixXd jacobian_L1(double q1, double q2, double q3);
    // MatrixXd jacobian_L2(double q1, double q2, double q3);

    // MatrixXd d_jacobian_R1(double q1, double q2, double q3, double dq1, double dq2, double dq3);
    // MatrixXd d_jacobian_R2(double q1, double q2, double q3, double dq1, double dq2, double dq3);
    // MatrixXd d_jacobian_L1(double q1, double q2, double q3, double dq1, double dq2, double dq3);
    // MatrixXd d_jacobian_L2(double q1, double q2, double q3, double dq1, double dq2, double dq3);

    // VectorXd fkine_R1(double q1, double q2, double q3);
    // VectorXd fkine_R2(double q1, double q2, double q3);
    // VectorXd fkine_L1(double q1, double q2, double q3);
    // VectorXd fkine_L2(double q1, double q2, double q3);
    // MatrixXd body_rotation_matrix(double roll, double pitch, double yaw);

    // void joint_space_matrices_R1(VectorXd theta, VectorXd d_theta, MatrixXd &M, VectorXd &V, VectorXd &G, VectorXd &F);
    // void joint_space_matrices_R2(VectorXd theta, VectorXd d_theta, MatrixXd &M, VectorXd &V, VectorXd &G, VectorXd &F);
    // void joint_space_matrices_L1(VectorXd theta, VectorXd d_theta, MatrixXd &M, VectorXd &V, VectorXd &G, VectorXd &F);
    // void joint_space_matrices_L2(VectorXd theta, VectorXd d_theta, MatrixXd &M, VectorXd &V, VectorXd &G, VectorXd &F);

    VectorXd cartesian_inverse_dynamics(VectorXd dd_x, VectorXd dq, MatrixXd M, VectorXd V, VectorXd G, VectorXd F_fric, MatrixXd J, MatrixXd dJ);
    VectorXd get_tau_ff(VectorXd dd_x_ref, VectorXd dq, MatrixXd M, VectorXd V, VectorXd G, VectorXd F_fric, MatrixXd J, MatrixXd dJ);

    VectorXd calculate(LegData &leg_cmd, VectorXd &theta, VectorXd &d_theta, VectorXd &rpy, Vector4i &phase_signal);
                        //MatrixXd &leg_pos_ref, MatrixXd &leg_vel_ref, MatrixXd &leg_acc_ref, MatrixXd &leg_grf_ref,
private:

    // double m1, m2, m3;
    // double l1, l2, l3;
    // double d1, d2, d3;
    // double l_cx_3, l_cz_2;
    // double bx, by;
    // double g;
    LegModel leg_model;
    RobotPhysicalParams robot_params;

    MatrixXd J_R1, J_R2, J_L1, J_L2;
    MatrixXd dJ_R1, dJ_R2, dJ_L1, dJ_L2;
    MatrixXd R_body;
    VectorXd X_R1, X_L1, X_R2, X_L2;
    VectorXd dX_R1, dX_L1, dX_R2, dX_L2;
    VectorXd r1_offset, l1_offset, r2_offset, l2_offset;
    VectorXd d_theta_r1, d_theta_l1, d_theta_r2, d_theta_l2;

    MatrixXd M_R1, M_L1, M_R2, M_L2;
    VectorXd V_R1, V_L1, V_R2, V_L2;
    VectorXd G_R1, G_L1, G_R2, G_L2;
    VectorXd F_R1, F_L1, F_R2, F_L2;

    MatrixXd invJ, J_T, invJ_T, Mx;
    VectorXd Vx, Gx, Fx;
    VectorXd F, tau;
    MatrixXd invR;

    MatrixXd Kp, Kd;
    VectorXd e_r1, de_r1, x_ref_r1, dx_ref_r1, ddx_ref_r1, u_r1;
    VectorXd e_l1, de_l1, x_ref_l1, dx_ref_l1, ddx_ref_l1, u_l1;
    VectorXd e_r2, de_r2, x_ref_r2, dx_ref_r2, ddx_ref_r2, u_r2;
    VectorXd e_l2, de_l2, x_ref_l2, dx_ref_l2, ddx_ref_l2, u_l2;
    VectorXd grf_ref_r1, grf_ref_l1, grf_ref_r2, grf_ref_l2;

    VectorXd imp_tau_ref_r1, imp_tau_ref_l1, imp_tau_ref_r2, imp_tau_ref_l2;
    VectorXd grf_tau_ref_r1, grf_tau_ref_l1, grf_tau_ref_r2, grf_tau_ref_l2;
    VectorXd tau_ref_r1, tau_ref_l1, tau_ref_r2, tau_ref_l2;
    VectorXd tau_ref;

    double j11, j12, j13, j21, j22, j23, j31, j32, j33;
    double dj11, dj12, dj13, dj21, dj22, dj23, dj31, dj32, dj33;
    double r11, r12, r13, r21, r22, r23, r31, r32, r33;

    double f1, f2, f3;

};

#endif //_grf_control_hpp_