#include "leg_state.hpp"

LegState::LegState(RobotPhysicalParams &robot)
{
    robot_.BuildPinocchioModel();

    q_ = Eigen::VectorXd::Zero(robot_.nq); // free-flyer base (pos + quat) + 12 joints
    v_ = Eigen::VectorXd::Zero(robot_.nv); // free-flyer base (lin + ang) + 12 joints
    q_(6) = 1.0;                           // identity quaternion (w = 1)

    J_.assign(4, Eigen::Matrix3d::Zero());
    dJ_.assign(4, Eigen::Matrix3d::Zero());
    X_.assign(4, Eigen::Vector3d::Zero());
    dX_.assign(4, Eigen::Vector3d::Zero());
    ddX_.assign(4, Eigen::Vector3d::Zero());
    M_.assign(4, Eigen::Matrix3d::Zero());
    C_.assign(4, Eigen::Vector3d::Zero());
    G_.assign(4, Eigen::Vector3d::Zero());
    f_hat_.assign(4, Eigen::Vector3d::Zero());

    threshold = THRESHOLD;

    contact.resize(4);
}

void LegState::set_grf_observer_params(double lamb, double dt, const Eigen::VectorXd& p, Eigen::VectorXd& cur_theta)
{
    this->p = p;
    if (cur_theta(1) < 0.0)
        gm_observer_r1.set_params(lamb, dt, p);
    else
        gm_observer_r1.set_params(lamb, dt, -p);

    if (cur_theta(4) > 0.0)
        gm_observer_l1.set_params(lamb, dt, -p);
    else
        gm_observer_l1.set_params(lamb, dt, p);

    if (cur_theta(7) < 0.0)
        gm_observer_r2.set_params(lamb, dt, p);
    else
        gm_observer_r2.set_params(lamb, dt, -p);

    if (cur_theta(10) > 0.0)
        gm_observer_l2.set_params(lamb, dt, -p);
    else
        gm_observer_l2.set_params(lamb, dt, p);
}

LegState::~LegState()
{

}

void LegState::set_contact_threshold(double threshold)
{
    this->threshold = threshold;
}

LegData LegState::get_leg_state(RobotData &body_state, VectorXd &theta, VectorXd &d_theta, VectorXd &tau)
{
    update_configuration(body_state, theta, d_theta);

    // forward kinematics, Jacobians and joint-space dynamics via pinocchio
    robot_.ComputeForwardKinematics(q_, v_);

    std::vector<Eigen::Vector3d> pin_pos = robot_.GetToePositionsInWorldFrame();
    std::vector<Eigen::Matrix3d> pin_jac = robot_.GetFootJacobianWorld(q_);
    std::vector<Eigen::Matrix3d> pin_djac = robot_.GetFootJacobianDotWorld(q_, v_);

    std::vector<Eigen::Matrix3d> pin_M(4);
    std::vector<Eigen::Vector3d> pin_C(4), pin_G(4);
    robot_.GetLegDynamics(q_, v_, pin_M, pin_C, pin_G);

    // remap from pinocchio order (L1, L2, R1, R2) to servo order (R1, L1, R2, L2)
    X_[R1] = pin_pos[PIN_R1]; X_[L1] = pin_pos[PIN_L1]; X_[R2] = pin_pos[PIN_R2]; X_[L2] = pin_pos[PIN_L2];
    J_[R1] = pin_jac[PIN_R1]; J_[L1] = pin_jac[PIN_L1]; J_[R2] = pin_jac[PIN_R2]; J_[L2] = pin_jac[PIN_L2];
    dJ_[R1] = pin_djac[PIN_R1]; dJ_[L1] = pin_djac[PIN_L1]; dJ_[R2] = pin_djac[PIN_R2]; dJ_[L2] = pin_djac[PIN_L2];
    M_[R1] = pin_M[PIN_R1]; M_[L1] = pin_M[PIN_L1]; M_[R2] = pin_M[PIN_R2]; M_[L2] = pin_M[PIN_L2];
    C_[R1] = pin_C[PIN_R1]; C_[L1] = pin_C[PIN_L1]; C_[R2] = pin_C[PIN_R2]; C_[L2] = pin_C[PIN_L2];
    G_[R1] = pin_G[PIN_R1]; G_[L1] = pin_G[PIN_L1]; G_[R2] = pin_G[PIN_R2]; G_[L2] = pin_G[PIN_L2];

    calc_pos_vel_acc(d_theta);
    calc_grf(tau, d_theta);
    calc_contacts();

    leg_data.r1_pos = X_[R1];
    leg_data.l1_pos = X_[L1];
    leg_data.r2_pos = X_[R2];
    leg_data.l2_pos = X_[L2];

    leg_data.r1_vel = dX_[R1];
    leg_data.l1_vel = dX_[L1];
    leg_data.r2_vel = dX_[R2];
    leg_data.l2_vel = dX_[L2];

    leg_data.r1_acc = ddX_[R1];
    leg_data.l1_acc = ddX_[L1];
    leg_data.r2_acc = ddX_[R2];
    leg_data.l2_acc = ddX_[L2];

    leg_data.r1_grf = f_hat_[R1];
    leg_data.l1_grf = f_hat_[L1];
    leg_data.r2_grf = f_hat_[R2];
    leg_data.l2_grf = f_hat_[L2];

    leg_data.contacts = contact;

    return leg_data;
}

void LegState::update_configuration(RobotData &body_state, VectorXd &theta, VectorXd &d_theta)
{
    // base pose (true position + orientation) -> global foot positions / gravity direction
    q_.segment(0, 3) = body_state.pos;
    q_.segment(3, 4) = body_state.orientation_quaternion;

    // joint angles, reordered servo (R1, L1, R2, L2) -> pinocchio (L1, L2, R1, R2)
    q_.segment(7, 3)  = theta.segment(3, 3); // L1
    q_.segment(10, 3) = theta.segment(9, 3); // L2
    q_.segment(13, 3) = theta.segment(0, 3); // R1
    q_.segment(16, 3) = theta.segment(6, 3); // R2

    // keep the base fixed for joint-space dynamics so the per-leg Coriolis terms stay
    // isolated (как в исходном поленогом наблюдателе); only joint velocities are used
    v_.segment(0, 6).setZero();
    v_.segment(6, 3)  = d_theta.segment(3, 3); // L1
    v_.segment(9, 3)  = d_theta.segment(9, 3); // L2
    v_.segment(12, 3) = d_theta.segment(0, 3); // R1
    v_.segment(15, 3) = d_theta.segment(6, 3); // R2
}

void LegState::calc_pos_vel_acc(VectorXd d_theta)
{
    // foot velocity / acceleration from joint motion only (world frame)
    dX_[R1]  = J_[R1] * d_theta.segment(0, 3);
    dX_[L1]  = J_[L1] * d_theta.segment(3, 3);
    dX_[R2]  = J_[R2] * d_theta.segment(6, 3);
    dX_[L2]  = J_[L2] * d_theta.segment(9, 3);

    ddX_[R1] = dJ_[R1] * d_theta.segment(0, 3);
    ddX_[L1] = dJ_[L1] * d_theta.segment(3, 3);
    ddX_[R2] = dJ_[R2] * d_theta.segment(6, 3);
    ddX_[L2] = dJ_[L2] * d_theta.segment(9, 3);
}

VectorXd LegState::inv_dyn_force_observer(const VectorXd& tau, const MatrixXd& J, const VectorXd& G, const VectorXd& V)
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

void LegState::calc_grf(VectorXd tau, VectorXd d_theta)
{
    // general momentum based force observer (f_hat in world frame, z = vertical)
    f_hat_[R1] = gm_observer_r1.step(tau.segment(0, 3), d_theta.segment(0, 3), M_[R1], C_[R1], G_[R1], J_[R1]);
    f_hat_[L1] = gm_observer_l1.step(tau.segment(3, 3), d_theta.segment(3, 3), M_[L1], C_[L1], G_[L1], J_[L1]);
    f_hat_[R2] = gm_observer_r2.step(tau.segment(6, 3), d_theta.segment(6, 3), M_[R2], C_[R2], G_[R2], J_[R2]);
    f_hat_[L2] = gm_observer_l2.step(tau.segment(9, 3), d_theta.segment(9, 3), M_[L2], C_[L2], G_[L2], J_[L2]);

    // inverse dynamics force observer
//     f_hat_[R1] = inv_dyn_force_observer(tau.segment(0, 3), J_[R1], G_[R1], C_[R1]);
//     f_hat_[L1] = inv_dyn_force_observer(tau.segment(3, 3), J_[L1], G_[L1], C_[L1]);
//     f_hat_[R2] = inv_dyn_force_observer(tau.segment(6, 3), J_[R2], G_[R2], C_[R2]);
//     f_hat_[L2] = inv_dyn_force_observer(tau.segment(9, 3), J_[L2], G_[L2], C_[L2]);
}

void LegState::calc_contacts()
{
    for (int i = 0; i < 4; i++)
        contact[i] = (f_hat_[i](Z) >= threshold);
}
