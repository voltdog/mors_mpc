#include "leg_control.hpp"

LegControl::LegControl()
{
    X_R1.resize(3); X_L1.resize(3); X_R2.resize(3); X_L2.resize(3);
    dX_R1.resize(3); dX_L1.resize(3); dX_R2.resize(3); dX_L2.resize(3);
    r1_offset.resize(3); l1_offset.resize(3); r2_offset.resize(3); l2_offset.resize(3);
    d_theta_r1.resize(3); d_theta_l1.resize(3); d_theta_r2.resize(3); d_theta_l2.resize(3);
    M_R1.resize(3,3); M_L1.resize(3,3); M_R2.resize(3,3); M_L2.resize(3,3);
    V_R1.resize(3); V_L1.resize(3); V_R2.resize(3); V_L2.resize(3);
    G_R1.resize(3); G_L1.resize(3); G_R2.resize(3); G_L2.resize(3);
    F_R1.resize(3); F_L1.resize(3); F_R2.resize(3); F_L2.resize(3);

    invJ.resize(3,3); J_T.resize(3,3); invJ_T.resize(3,3); Mx.resize(3,3);
    Vx.resize(3); Gx.resize(3);
    F.resize(3); tau.resize(3);
    invR.resize(3,3);

    Kp.resize(3,3); Kd.resize(3,3);
    e_r1.resize(3); de_r1.resize(3); x_ref_r1.resize(3); dx_ref_r1.resize(3); ddx_ref_r1.resize(3); u_r1.resize(3);
    e_l1.resize(3); de_l1.resize(3); x_ref_l1.resize(3); dx_ref_l1.resize(3); ddx_ref_l1.resize(3); u_l1.resize(3);
    e_r2.resize(3); de_r2.resize(3); x_ref_r2.resize(3); dx_ref_r2.resize(3); ddx_ref_r2.resize(3); u_r2.resize(3);
    e_l2.resize(3); de_l2.resize(3); x_ref_l2.resize(3); dx_ref_l2.resize(3); ddx_ref_l2.resize(3); u_l2.resize(3);
    grf_ref_r1.resize(3); grf_ref_l1.resize(3); grf_ref_r2.resize(3); grf_ref_l2.resize(3);

    tau_ref_r1.resize(3);
    tau_ref_l1.resize(3);
    tau_ref_r2.resize(3);
    tau_ref_l2.resize(3);
    imp_tau_ref_r1.resize(3);
    imp_tau_ref_l1.resize(3);
    imp_tau_ref_r2.resize(3);
    imp_tau_ref_l2.resize(3);
    grf_tau_ref_r1.resize(3);
    grf_tau_ref_l1.resize(3);
    grf_tau_ref_r2.resize(3);
    grf_tau_ref_l2.resize(3);
    tau_ref.resize(12);
    tau_ref.setZero();

    Kp.setZero();
    Kd.setZero();
}

void LegControl::set_leg_params(RobotPhysicalParams &robot)
{
    this->robot_params = robot;
    leg_model.set_leg_params(this->robot_params);

    r1_offset <<  robot_params.bx, -robot_params.by, 0;
    l1_offset <<  robot_params.bx,  robot_params.by, 0;
    r2_offset << -robot_params.bx, -robot_params.by, 0;
    l2_offset << -robot_params.bx,  robot_params.by, 0;
}

void LegControl::set_feedback_params(MatrixXd& Kp, MatrixXd& Kd)
{
    this->Kp = Kp;
    this->Kd = Kd;
}

VectorXd LegControl::cartesian_inverse_dynamics(VectorXd dd_x, VectorXd dq, MatrixXd M, VectorXd V, VectorXd G, VectorXd F_fric, MatrixXd J, MatrixXd dJ)
{
    invJ = J.inverse();
    J_T = J.transpose();
    invJ_T = J_T.inverse();

    Mx = invJ_T * M * invJ;
    Vx = invJ_T * (V - M * invJ * dJ * dq);
    Gx = invJ_T * G;
    Fx = invJ_T * F_fric;

    F = Mx * dd_x + Vx + Gx + Fx;
    tau = J_T * F;

    return tau;
}

VectorXd LegControl::get_tau_ff(VectorXd dd_x_ref, VectorXd dq, MatrixXd M, VectorXd V, VectorXd G, VectorXd F_fric, MatrixXd J, MatrixXd dJ)
{
    invJ = J.inverse();

    tau = M * invJ * (dd_x_ref - dJ * dq) + V + G + F_fric;

    return tau;
}

VectorXd LegControl::calculate(LegData &leg_cmd, VectorXd &theta, VectorXd &d_theta, VectorXd &rpy, Vector4d &phase_signal)
{
    // prepare data

    x_ref_r1 = leg_cmd.r1_pos;
    x_ref_l1 = leg_cmd.l1_pos;
    x_ref_r2 = leg_cmd.r2_pos;
    x_ref_l2 = leg_cmd.l2_pos;

    dx_ref_r1 = leg_cmd.r1_vel;
    dx_ref_l1 = leg_cmd.l1_vel;
    dx_ref_r2 = leg_cmd.r2_vel;
    dx_ref_l2 = leg_cmd.l2_vel;

    ddx_ref_r1 = leg_cmd.r1_acc;
    ddx_ref_l1 = leg_cmd.l1_acc;
    ddx_ref_r2 = leg_cmd.r2_acc;
    ddx_ref_l2 = leg_cmd.l2_acc;

    grf_ref_r1 = leg_cmd.r1_grf;
    grf_ref_l1 = leg_cmd.l1_grf;
    grf_ref_r2 = leg_cmd.r2_grf;
    grf_ref_l2 = leg_cmd.l2_grf;

    d_theta_r1 = d_theta.segment(0, 3);
    d_theta_l1 = d_theta.segment(3, 3);
    d_theta_r2 = d_theta.segment(6, 3);
    d_theta_l2 = d_theta.segment(9, 3);

    // get body rotation matrix and its inversion
    R_body = leg_model.body_rotation_matrix(rpy(0), rpy(1), rpy(2));
    invR = R_body.inverse();


    // calculate Jacobians and its derivatives
    J_R1 = leg_model.jacobian_R1(theta(0), theta(1), theta(2));
    J_L1 = leg_model.jacobian_L1(theta(3), theta(4), theta(5));
    J_R2 = leg_model.jacobian_R2(theta(6), theta(7), theta(8));
    J_L2 = leg_model.jacobian_L2(theta(9), theta(10), theta(11));
    
    dJ_R1 = leg_model.d_jacobian_R1(theta(0), theta(1), theta(2), d_theta(0), d_theta(1), d_theta(2));
    dJ_L1 = leg_model.d_jacobian_R1(theta(3), theta(4), theta(5), d_theta(3), d_theta(4), d_theta(5));
    dJ_R2 = leg_model.d_jacobian_R1(theta(6), theta(7), theta(8), d_theta(6), d_theta(7), d_theta(8));
    dJ_L2 = leg_model.d_jacobian_R1(theta(9), theta(10), theta(11), d_theta(9), d_theta(10), d_theta(11));

    // calculate kinematics    
    X_R1 = leg_model.fkine_R1(theta(0), theta(1),  theta(2))  + r1_offset;
    X_L1 = leg_model.fkine_L1(theta(3), theta(4),  theta(5))  + l1_offset;
    X_R2 = leg_model.fkine_R2(theta(6), theta(7),  theta(8))  + r2_offset;
    X_L2 = leg_model.fkine_L2(theta(9), theta(10), theta(11)) + l2_offset;

    dX_R1 = J_R1 * d_theta_r1;
    dX_L1 = J_L1 * d_theta_l1;
    dX_R2 = J_R2 * d_theta_r2;
    dX_L2 = J_L2 * d_theta_l2;

    // feedback control
    if (phase_signal(R1) != STANCE)
    {
        e_r1 = x_ref_r1 - X_R1;
        de_r1 = dx_ref_r1 - dX_R1;
        u_r1 = Kp * e_r1 + Kd * de_r1;// + ddx_ref_r1;
    }
    else
        u_r1.setZero();

    if (phase_signal(L1) != STANCE)
    {
        e_l1 = x_ref_l1 - X_L1;
        de_l1 = dx_ref_l1 - dX_L1;
        u_l1 = Kp * e_l1 + Kd * de_l1;// + ddx_ref_l1;
    }
    else
        u_l1.setZero();
    
    if (phase_signal(R2) != STANCE)
    {
        e_r2 = x_ref_r2 - X_R2;
        de_r2 = dx_ref_r2 - dX_R2;
        u_r2 = Kp * e_r2 + Kd * de_r2;// + ddx_ref_r2;
    }
    else
        u_r2.setZero();

    if (phase_signal(L2) != STANCE)
    {
        e_l2 = x_ref_l2 - X_L2;
        de_l2 = dx_ref_l2 - dX_L2;
        u_l2 = Kp * e_l2 + Kd * de_l2;// + ddx_ref_l2;
    }
    else
        u_l2.setZero();

    // calculate dynamics matrices
    leg_model.joint_space_matrices_R1(theta, d_theta, M_R1, V_R1, G_R1, F_R1);
    leg_model.joint_space_matrices_L1(theta, d_theta, M_L1, V_L1, G_L1, F_L1);
    leg_model.joint_space_matrices_R2(theta, d_theta, M_R2, V_R2, G_R2, F_R2);
    leg_model.joint_space_matrices_L2(theta, d_theta, M_L2, V_L2, G_L2, F_L2);

    // get desired tau for impedance control
    imp_tau_ref_r1 = J_R1.transpose() * u_r1 - get_tau_ff(ddx_ref_r1, d_theta_r1, M_R1, V_R1, G_R1, F_R1, J_R1, dJ_R1);
    imp_tau_ref_l1 = J_L1.transpose() * u_l1 + get_tau_ff(ddx_ref_l1, d_theta_l1, M_L1, V_L1, G_L1, F_L1, J_L1, dJ_L1);
    imp_tau_ref_r2 = J_R2.transpose() * u_r2 - get_tau_ff(ddx_ref_r2, d_theta_r2, M_R2, V_R2, G_R2, F_R2, J_R2, dJ_R2);
    imp_tau_ref_l2 = J_L2.transpose() * u_l2 + get_tau_ff(ddx_ref_l2, d_theta_l2, M_L2, V_L2, G_L2, F_L2, J_L2, dJ_L2);

    // get desired tau for GRF control
    grf_tau_ref_r1 = J_R1.transpose() * (invR * grf_ref_r1);
    grf_tau_ref_l1 = J_L1.transpose() * (invR * grf_ref_l1);
    grf_tau_ref_r2 = J_R2.transpose() * (invR * grf_ref_r2);
    grf_tau_ref_l2 = J_L2.transpose() * (invR * grf_ref_l2);

    tau_ref_r1 = imp_tau_ref_r1 + grf_tau_ref_r1;
    tau_ref_l1 = imp_tau_ref_l1 + grf_tau_ref_l1;
    tau_ref_r2 = imp_tau_ref_r2 + grf_tau_ref_r2;
    tau_ref_l2 = imp_tau_ref_l2 + grf_tau_ref_l2;

    tau_ref.segment(0, 3) = tau_ref_r1;
    tau_ref.segment(3, 3) = tau_ref_l1;
    tau_ref.segment(6, 3) = tau_ref_r2;
    tau_ref.segment(9, 3) = tau_ref_l2;

    return tau_ref;
}