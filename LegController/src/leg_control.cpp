#include "leg_control.hpp"

LegControl::LegControl()
{
    // X_R1.resize(3); X_L1.resize(3); X_R2.resize(3); X_L2.resize(3);
    M_R1.resize(3,3); M_L1.resize(3,3); M_R2.resize(3,3); M_L2.resize(3,3);
    H_R1.resize(3); H_L1.resize(3); H_R2.resize(3); H_L2.resize(3);

    tau_ref_r1.resize(3);
    tau_ref_l1.resize(3);
    tau_ref_r2.resize(3);
    tau_ref_l2.resize(3);

    robot.BuildPinocchioModel();
    q.resize(19);
    v.resize(18);
    q.setZero();
    q(6) = 1.0;
    v.setZero();
    M_legs.resize(4);
    h_legs.resize(4);
}

void LegControl::calculate(LegData &leg_cmd, VectorXd &theta, VectorXd &d_theta, VectorXd &theta_ref, VectorXd &d_theta_ref, VectorXd& tau_ref)
{
    auto start = std::chrono::steady_clock::now();

    // prepare data
    q.segment(7, 3) = theta.segment(3, 3); // order of legs in pinocchio model is L1, L2, R1, R2
    q.segment(10, 3) = theta.segment(9, 3);
    q.segment(13, 3) = theta.segment(0, 3);
    q.segment(16, 3) = theta.segment(6, 3);

    v.segment(6, 3) = d_theta.segment(3, 3); // order of legs in pinocchio model is L1, L2, R1, R2
    v.segment(9, 3) = d_theta.segment(9, 3);
    v.segment(12, 3) = d_theta.segment(0, 3);
    v.segment(15, 3) = d_theta.segment(6, 3);

    // compute forward kinematics
    robot.ComputeForwardKinematics(q, v);
    // leg_positions = robot.GetToePositionsInBaseFrame();
    // X_R1 = leg_positions[0];
    // X_L1 = leg_positions[1];
    // X_R2 = leg_positions[2];
    // X_L2 = leg_positions[3];

    // get leg Jacobians
    leg_jacobians = robot.GetFootJacobian(q);
    J_R1 = leg_jacobians[0];
    J_L1 = leg_jacobians[1];
    J_R2 = leg_jacobians[2];
    J_L2 = leg_jacobians[3];

    // calculate dynamics matrices
    // robot.ComputeAllLegDynamics(q, v, M_legs, h_legs);
    // M_R1 = M_legs[2]; M_L1 = M_legs[0]; M_R2 = M_legs[3]; M_L2 = M_legs[1];
    // H_R1 = h_legs[2]; H_L1 = h_legs[0]; H_R2 = h_legs[3]; H_L2 = h_legs[1];

    // compute torques with force control and gravity compensation
    tau_ref_r1 = J_R1.transpose() * leg_cmd.r1_grf;// - H_R1;
    tau_ref_l1 = J_L1.transpose() * leg_cmd.l1_grf;// - H_L1;
    tau_ref_r2 = J_R2.transpose() * leg_cmd.r2_grf;// - H_R2;
    tau_ref_l2 = J_L2.transpose() * leg_cmd.l2_grf;// - H_L2;
    tau_ref.segment(0, 3) = tau_ref_r1;
    tau_ref.segment(3, 3) = tau_ref_l1;
    tau_ref.segment(6, 3) = tau_ref_r2;
    tau_ref.segment(9, 3) = tau_ref_l2;

    // compute ref joint positions (inverse kinematics)
    std::vector<Eigen::Vector3d> x_ref_local(4);
    x_ref_local[0] = leg_cmd.l1_pos;
    x_ref_local[1] = leg_cmd.l2_pos;
    x_ref_local[2] = leg_cmd.r1_pos;
    x_ref_local[3] = leg_cmd.r2_pos;
     
    bool ik_success = robot.ComputeIK_CLIK(q, x_ref_local,
                                            50, 1e-4, 2.0, 1e-6);

    // cout << ik_success << endl;
    // cout << "-------" << endl;

    theta_ref << q.segment(13, 3), 
                 q.segment(7, 3), 
                 q.segment(16, 3), 
                 q.segment(10, 3);

    auto end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // std::cout << "Elapsed time: " << elapsed.count() << " us\n";

    // compute joint velocities
    d_theta_ref.setZero();
}