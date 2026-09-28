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

    // World-frame foot positions are used by the state estimator.
    robot_.ComputeForwardKinematics(q_, v_);
    std::vector<Eigen::Vector3d> pin_pos = robot_.GetToePositionsInWorldFrame();

    // The force observer works in the base-local frame. Fix the floating base
    // at the local-frame origin and exclude its motion from the per-leg model.
    Eigen::VectorXd q_local = q_;
    Eigen::VectorXd v_local = v_;
    q_local.segment(0, 7).setZero();
    q_local(6) = 1.0;
    v_local.segment(0, 6).setZero();

    robot_.ComputeForwardKinematics(q_local, v_local);
    std::vector<Eigen::Matrix3d> pin_jac = robot_.GetFootJacobian(q_local);

    std::vector<Eigen::Matrix3d> pin_M(4);
    std::vector<Eigen::Vector3d> pin_C(4), pin_G(4);
    robot_.GetLegDynamics(q_local, v_local, pin_M, pin_C, pin_G);

    // remap from pinocchio order (L1, L2, R1, R2) to servo order (R1, L1, R2, L2)
    X_[R1] = pin_pos[PIN_R1]; X_[L1] = pin_pos[PIN_L1]; X_[R2] = pin_pos[PIN_R2]; X_[L2] = pin_pos[PIN_L2];
    J_[R1] = pin_jac[PIN_R1]; J_[L1] = pin_jac[PIN_L1]; J_[R2] = pin_jac[PIN_R2]; J_[L2] = pin_jac[PIN_L2];
    M_[R1] = pin_M[PIN_R1]; M_[L1] = pin_M[PIN_L1]; M_[R2] = pin_M[PIN_R2]; M_[L2] = pin_M[PIN_L2];
    C_[R1] = pin_C[PIN_R1]; C_[L1] = pin_C[PIN_L1]; C_[R2] = pin_C[PIN_R2]; C_[L2] = pin_C[PIN_L2];
    G_[R1] = pin_G[PIN_R1]; G_[L1] = pin_G[PIN_L1]; G_[R2] = pin_G[PIN_R2]; G_[L2] = pin_G[PIN_L2];

    calc_pos_vel_acc(body_state, theta, d_theta);
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

    Eigen::Quaterniond q_body(
        body_state.orientation_quaternion(3),
        body_state.orientation_quaternion(0),
        body_state.orientation_quaternion(1),
        body_state.orientation_quaternion(2));
    const Eigen::Matrix3d r_body = q_body.toRotationMatrix();

    v_.segment(0, 3) = r_body.transpose() * body_state.lin_vel;
    v_.segment(3, 3) = body_state.ang_vel;
    v_.segment(6, 3)  = d_theta.segment(3, 3); // L1
    v_.segment(9, 3)  = d_theta.segment(9, 3); // L2
    v_.segment(12, 3) = d_theta.segment(0, 3); // R1
    v_.segment(15, 3) = d_theta.segment(6, 3); // R2
}

void LegState::calc_pos_vel_acc(RobotData &body_state, VectorXd &theta, VectorXd &d_theta)
{
    wbic_types::Vector12d joint_pos = theta;
    wbic_types::Vector12d joint_vel = d_theta;

    robot_.update(body_state, joint_pos, joint_vel);

    dX_[R1] = robot_.get_tip_vel_global(PIN_R1);
    dX_[L1] = robot_.get_tip_vel_global(PIN_L1);
    dX_[R2] = robot_.get_tip_vel_global(PIN_R2);
    dX_[L2] = robot_.get_tip_vel_global(PIN_L2);

    ddX_[R1] = robot_.get_leg_pos_jdqd(PIN_R1);
    ddX_[L1] = robot_.get_leg_pos_jdqd(PIN_L1);
    ddX_[R2] = robot_.get_leg_pos_jdqd(PIN_R2);
    ddX_[L2] = robot_.get_leg_pos_jdqd(PIN_L2);
}

VectorXd LegState::inv_dyn_force_observer(const VectorXd& tau, const MatrixXd& J, const VectorXd& G, const VectorXd& V)
{
    const MatrixXd jacobian_transpose = J.transpose();
    const auto decomposition = jacobian_transpose.completeOrthogonalDecomposition();
    VectorXd f_hat = decomposition.solve(-(tau - V - G));

    // for (int i = 0; i < 3; i++)
    // {
        if (f_hat(Z) < 0.0)
            f_hat(Z) = 0.0;
    // }

    return f_hat;
}

void LegState::calc_grf(VectorXd tau, VectorXd d_theta)
{
    // general momentum based force observer (f_hat in the base-local frame)
    // f_hat_[R1] = gm_observer_r1.step(tau.segment(0, 3), d_theta.segment(0, 3), M_[R1], C_[R1], G_[R1], J_[R1]);
    // f_hat_[L1] = gm_observer_l1.step(tau.segment(3, 3), d_theta.segment(3, 3), M_[L1], C_[L1], G_[L1], J_[L1]);
    // f_hat_[R2] = gm_observer_r2.step(tau.segment(6, 3), d_theta.segment(6, 3), M_[R2], C_[R2], G_[R2], J_[R2]);
    // f_hat_[L2] = gm_observer_l2.step(tau.segment(9, 3), d_theta.segment(9, 3), M_[L2], C_[L2], G_[L2], J_[L2]);

    // inverse dynamics force observer
    f_hat_[R1] = inv_dyn_force_observer(tau.segment(0, 3), J_[R1], G_[R1], C_[R1]);
    f_hat_[L1] = inv_dyn_force_observer(tau.segment(3, 3), J_[L1], G_[L1], C_[L1]);
    f_hat_[R2] = inv_dyn_force_observer(tau.segment(6, 3), J_[R2], G_[R2], C_[R2]);
    f_hat_[L2] = inv_dyn_force_observer(tau.segment(9, 3), J_[L2], G_[L2], C_[L2]);
}

void LegState::calc_contacts()
{
    for (int i = 0; i < 4; i++)
        contact[i] = (f_hat_[i](Z) >= threshold);
}
