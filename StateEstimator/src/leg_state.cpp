#include "leg_state.hpp"

LegState::LegState(RobotPhysicalParams &robot)
{
    leg_model.set_leg_params(robot);

    J_R1.resize(3,3); J_R2.resize(3,3); J_L1.resize(3,3); J_L2.resize(3,3);
    X_R1.resize(3); X_L1.resize(3); X_R2.resize(3); X_L2.resize(3);
    dX_R1.resize(3); dX_L1.resize(3); dX_R2.resize(3); dX_L2.resize(3);
    r1_offset.resize(3); l1_offset.resize(3); r2_offset.resize(3); l2_offset.resize(3);
    d_theta_r1.resize(3); d_theta_l1.resize(3); d_theta_r2.resize(3); d_theta_l2.resize(3);
    M_R1.resize(3,3); M_L1.resize(3,3); M_R2.resize(3,3); M_L2.resize(3,3);
    V_R1.resize(3); V_L1.resize(3); V_R2.resize(3); V_L2.resize(3);
    G_R1.resize(3); G_L1.resize(3); G_R2.resize(3); G_L2.resize(3);
    F_R1.resize(3); F_L1.resize(3); F_R2.resize(3); F_L2.resize(3);

    r1_offset <<  robot.bx, -robot.by, 0;
    l1_offset <<  robot.bx,  robot.by, 0;
    r2_offset << -robot.bx, -robot.by, 0;
    l2_offset << -robot.bx,  robot.by, 0;

    threshold = THRESHOLD;

    invJ_T.resize(3,3);
    f_hat.resize(3);

    contact.resize(4);
}

void LegState::set_grf_observer_params(double lamb, double dt, const Eigen::VectorXd& p)
{
    this->p = p;
    gm_observer_r1.set_params(lamb, dt, p);
    gm_observer_l1.set_params(lamb, dt, -p);
    gm_observer_r2.set_params(lamb, dt, -p);
    gm_observer_l2.set_params(lamb, dt, p);
}

LegState::~LegState()
{

}

void LegState::set_contact_threshold(double threshold)
{
    this->threshold = threshold;
}

LegData LegState::get_leg_state(VectorXd theta, VectorXd d_theta, VectorXd tau)
{
    calc_jacobians(theta, d_theta);
    calc_pos_vel_acc(theta, d_theta);
    calc_joint_space_matrices(theta, d_theta);
    calc_grf(tau, d_theta, theta);

    calc_contacts();

    leg_data.r1_pos = X_R1;
    leg_data.l1_pos = X_L1;
    leg_data.r2_pos = X_R2;
    leg_data.l2_pos = X_L2;

    leg_data.r1_vel = dX_R1;
    leg_data.l1_vel = dX_L1;
    leg_data.r2_vel = dX_R2;
    leg_data.l2_vel = dX_L2;

    leg_data.r1_acc = ddX_R1;
    leg_data.l1_acc = ddX_L1;
    leg_data.r2_acc = ddX_R2;
    leg_data.l2_acc = ddX_L2;

    leg_data.r1_grf = f_hat_r1;
    leg_data.l1_grf = f_hat_l1;
    leg_data.r2_grf = f_hat_r2;
    leg_data.l2_grf = f_hat_l2;

    leg_data.contacts = contact;

    return leg_data;
}

void LegState::calc_jacobians(VectorXd theta, VectorXd d_theta)
{
    J_R1 = leg_model.jacobian_R1(theta(0), theta(1), theta(2));
    J_L1 = leg_model.jacobian_L1(theta(3), theta(4), theta(5));
    J_R2 = leg_model.jacobian_R2(theta(6), theta(7), theta(8));
    J_L2 = leg_model.jacobian_L2(theta(9), theta(10), theta(11));
    
    dJ_R1 = leg_model.d_jacobian_R1(theta(0), theta(1), theta(2), d_theta(0), d_theta(1), d_theta(2));
    dJ_L1 = leg_model.d_jacobian_R1(theta(3), theta(4), theta(5), d_theta(3), d_theta(4), d_theta(5));
    dJ_R2 = leg_model.d_jacobian_R1(theta(6), theta(7), theta(8), d_theta(6), d_theta(7), d_theta(8));
    dJ_L2 = leg_model.d_jacobian_R1(theta(9), theta(10), theta(11), d_theta(9), d_theta(10), d_theta(11));
}

void LegState::calc_pos_vel_acc(VectorXd theta, VectorXd d_theta)
{
    d_theta_r1 = d_theta.segment(0, 3);
    d_theta_l1 = d_theta.segment(3, 3);
    d_theta_r2 = d_theta.segment(6, 3);
    d_theta_l2 = d_theta.segment(9, 3);

    X_R1 = leg_model.fkine_R1(theta(0), theta(1),  theta(2))  + r1_offset;
    X_L1 = leg_model.fkine_L1(theta(3), theta(4),  theta(5))  + l1_offset;
    X_R2 = leg_model.fkine_R2(theta(6), theta(7),  theta(8))  + r2_offset;
    X_L2 = leg_model.fkine_L2(theta(9), theta(10), theta(11)) + l2_offset;

    dX_R1 = J_R1 * d_theta_r1;
    dX_L1 = J_L1 * d_theta_l1;
    dX_R2 = J_R2 * d_theta_r2;
    dX_L2 = J_L2 * d_theta_l2;

    ddX_R1 = dJ_R1 * d_theta_r1;
    ddX_L1 = dJ_L1 * d_theta_l1;
    ddX_R2 = dJ_R2 * d_theta_r2;
    ddX_L2 = dJ_L2 * d_theta_l2;
}

void LegState::calc_joint_space_matrices(VectorXd theta, VectorXd d_theta)
{
    leg_model.joint_space_matrices_R1(theta, d_theta, M_R1, V_R1, G_R1, F_R1);
    leg_model.joint_space_matrices_L1(theta, d_theta, M_L1, V_L1, G_L1, F_L1);
    leg_model.joint_space_matrices_R2(theta, d_theta, M_R2, V_R2, G_R2, F_R2);
    leg_model.joint_space_matrices_L2(theta, d_theta, M_L2, V_L2, G_L2, F_L2);
}

VectorXd LegState::inv_dyn_force_observer(VectorXd tau, MatrixXd J, VectorXd G, VectorXd V)
{
    invJ_T = J.transpose().inverse();

    f_hat = -invJ_T * (tau - V - G);

    // for (int i = 0; i < 3; i++)
    // {
        if (f_hat(Z) < 0.0)
            f_hat(Z) = 0.0;
    // }

    return f_hat;
}

void LegState::calc_grf(VectorXd tau, VectorXd d_theta, VectorXd theta)
{
    tau_r1 = tau.segment(0, 3);
    tau_l1 = tau.segment(3, 3);
    tau_r2 = tau.segment(6, 3);
    tau_l2 = tau.segment(9, 3);
    dq_r1 = d_theta.segment(0, 3);
    dq_l1 = d_theta.segment(3, 3);
    dq_r2 = d_theta.segment(6, 3);
    dq_l2 = d_theta.segment(9, 3);
    // inverse dynamics force observer
    // f_hat_r1 = inv_dyn_force_observer(tau_r1, J_R1, G_R1, V_R1);
    // f_hat_l1 = inv_dyn_force_observer(tau_l1, J_L1, G_L1, V_L1);
    // f_hat_r2 = inv_dyn_force_observer(tau_r2, J_R2, G_R2, V_R2);
    // f_hat_l2 = inv_dyn_force_observer(tau_l2, J_L2, G_L2, V_L2);
    // cout << f_hat_r1 << endl;
    // general momentum based force observer
    // if (theta(2) > 0)
    //     gm_observer_r1.set_p(p);
    // else
    //     gm_observer_r1.set_p(-p);
    f_hat_r1 = gm_observer_r1.step(tau_r1, dq_r1, M_R1, V_R1, G_R1, J_R1);
    f_hat_l1 = gm_observer_l1.step(tau_l1, dq_l1, M_L1, V_L1, G_L1, J_L1);
    f_hat_r2 = gm_observer_r2.step(tau_r2, dq_r2, M_R2, V_R2, G_R2, J_R2);
    f_hat_l2 = gm_observer_l2.step(tau_l2, dq_l2, M_L2, V_L2, G_L2, J_L2);
    // cout << f_hat_r1 << endl;
    // cout << "===" << endl;
}

void LegState::calc_contacts()
{
    if (f_hat_r1(2) >= threshold)
        contact(0) = true;
    else
        contact(0) = false;

    if (f_hat_l1(2) >= threshold)
        contact(1) = true;
    else
        contact(1) = false;

    if (f_hat_r2(2) >= threshold)
        contact(2) = true;
    else
        contact(2) = false;

    if (f_hat_l2(2) >= threshold)
        contact(3) = true;
    else
        contact(3) = false;
}