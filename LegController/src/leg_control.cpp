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
    // order of legs in pinocchio model is L1, L2, R1, R2
    // order of legs in my model R1, L1, R2, L2
    // so we must consider it

    auto start = std::chrono::steady_clock::now();

    // prepare data
    q.segment(PIN_START_IDX+PIN_R1*3, 3) = theta.segment(R1*3, 3); // order of legs in pinocchio model is L1, L2, R1, R2
    q.segment(PIN_START_IDX+PIN_L1*3, 3) = theta.segment(L1*3, 3);
    q.segment(PIN_START_IDX+PIN_R2*3, 3) = theta.segment(R2*3, 3);
    q.segment(PIN_START_IDX+PIN_L2*3, 3) = theta.segment(L2*3, 3);

    v.segment(PIN_START_IDX+PIN_R1*3-1, 3) = d_theta.segment(R1*3, 3); // order of legs in pinocchio model is L1, L2, R1, R2
    v.segment(PIN_START_IDX+PIN_L1*3-1, 3) = d_theta.segment(L1*3, 3);
    v.segment(PIN_START_IDX+PIN_R2*3-1, 3) = d_theta.segment(R2*3, 3);
    v.segment(PIN_START_IDX+PIN_L2*3-1, 3) = d_theta.segment(L2*3, 3);

    // compute forward kinematics
    robot.ComputeForwardKinematics(q, v);
    // leg_positions = robot.GetToePositionsInBaseFrame();
    // X_R1 = leg_positions[PIN_R1];
    // X_L1 = leg_positions[PIN_L1];
    // X_R2 = leg_positions[PIN_R2]];
    // X_L2 = leg_positions[PIN_L2]];

    // get leg Jacobians
    leg_jacobians = robot.GetFootJacobian(q);
    J_R1 = leg_jacobians[PIN_R1];
    J_L1 = leg_jacobians[PIN_L1];
    J_R2 = leg_jacobians[PIN_R2];
    J_L2 = leg_jacobians[PIN_L2];

    // calculate dynamics matrices
    robot.ComputeAllLegDynamics(q, v, M_legs, h_legs);
    M_R1 = M_legs[PIN_R1]; M_L1 = M_legs[PIN_L1]; M_R2 = M_legs[PIN_R2]; M_L2 = M_legs[PIN_L2];
    H_R1 = h_legs[PIN_R1]; H_L1 = h_legs[PIN_L1]; H_R2 = h_legs[PIN_R2]; H_L2 = h_legs[PIN_L2];

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
    x_ref_local[PIN_R1] = leg_cmd.r1_pos;
    x_ref_local[PIN_L1] = leg_cmd.l1_pos;
    x_ref_local[PIN_R2] = leg_cmd.r2_pos;
    x_ref_local[PIN_L2] = leg_cmd.l2_pos;

    bool ik_success = robot.ComputeIK_CLIK(q, x_ref_local,
                                            50, 1e-4, 2.0, 1e-6);

    // cout << ik_success << endl;
    // cout << "-------" << endl;

    theta_ref << q.segment(PIN_START_IDX+PIN_R1*3, 3), 
                 q.segment(PIN_START_IDX+PIN_L1*3, 3), 
                 q.segment(PIN_START_IDX+PIN_R2*3, 3), 
                 q.segment(PIN_START_IDX+PIN_L2*3, 3);

    auto end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // std::cout << "Elapsed time: " << elapsed.count() << " us\n";

    // compute joint velocities
    d_theta_ref.setZero();
}