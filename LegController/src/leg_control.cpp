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
    q.resize(18);
    v.resize(18);
    q.setZero();
    v.setZero();
    M_legs.resize(4);
    h_legs.resize(4);
}

void LegControl::calculate(LegData &leg_cmd, VectorXd &theta, VectorXd &d_theta, Vector4i &phase_signal, VectorXd &theta_ref, VectorXd &d_theta_ref, VectorXd& tau_ref)
{
    // prepare data
    q.segment(6, 3) = theta.segment(3, 3); // order of legs in pinocchio model is L1, L2, R1, R2
    q.segment(9, 3) = theta.segment(9, 3);
    q.segment(12, 3) = theta.segment(0, 3);
    q.segment(15, 3) = theta.segment(6, 3);
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
    robot.ComputeAllLegDynamics(q, v, M_legs, h_legs);
    M_R1 = M_legs[R1]; M_L1 = M_legs[L1]; M_R2 = M_legs[R2]; M_L2 = M_legs[L2];
    H_R1 = h_legs[R1]; H_L1 = h_legs[L1]; H_R2 = h_legs[R2]; H_L2 = h_legs[L2];


    // compute torques with force control and gravity compensation
    tau_ref_r1 = J_R1.transpose() * leg_cmd.r1_grf + H_R1;
    tau_ref_l1 = J_L1.transpose() * leg_cmd.l1_grf + H_L1;
    tau_ref_r2 = J_R2.transpose() * leg_cmd.r2_grf + H_R2;
    tau_ref_l2 = J_L2.transpose() * leg_cmd.l2_grf + H_L2;
    tau_ref.segment(0, 3) = tau_ref_r1;
    tau_ref.segment(3, 3) = tau_ref_l1;
    tau_ref.segment(6, 3) = tau_ref_r2;
    tau_ref.segment(9, 3) = tau_ref_l2;


    // compute ref joint positions (inverse kinematics)
    std::vector<Eigen::Vector3d> x_ref_local(4);
    x_ref_local[0] = leg_cmd.r1_pos;
    x_ref_local[1] = leg_cmd.l1_pos;
    x_ref_local[2] = leg_cmd.r2_pos;
    x_ref_local[3] = leg_cmd.l2_pos;
    robot.ComputeIK_CLIK(q, x_ref_local);
    theta_ref = q.segment(6, 12);

    // compute joint velocities
    d_theta_ref.setZero();
}