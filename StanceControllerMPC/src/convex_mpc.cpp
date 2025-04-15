#include "convex_mpc.hpp"


ConvexMPC::ConvexMPC()
{
    num_legs = NUM_LEGS;
    // Q.resize(13);
    // R.resize(12);
    cur_rpy.resize(3);
    A.resize(13, 13);
    B.resize(13, 12);
    Ad.resize(13, 13);
    Bd.resize(13, 12);
    // solver_inited = false;
    ref_grf.resize(12);
    R_body.resize(3,3);
    R_body.setZero();
}

ConvexMPC::~ConvexMPC()
{
    
}

void ConvexMPC::set_physical_params(RobotPhysicalParams &robot_params)
{
    this->robot_params = robot_params;
    inv_mass = Eigen::Matrix3d::Identity()/robot_params.M_b;
    
}

void ConvexMPC::set_mpc_params(double timestep, int horizon, double friction_coeff,
    double f_min, double f_max, VectorXd &Q, VectorXd &R)
{
    this->dt = timestep;
    this->horizon = horizon;
    this->friction_coeff << friction_coeff, friction_coeff, friction_coeff, friction_coeff; // -x +x -y +y
    this->f_min = f_min;
    this->f_max = f_max;
    // this->Q = Q;
    // this->R = R;

    Ac_dense.resize(5 * num_legs * horizon, 12 * horizon);
    Ac.resize(5 * num_legs * horizon, 12 * horizon);

    Qqp.resize(13*horizon, 13*horizon);
    Rqp.resize(12*horizon, 12*horizon);

    VectorXd q_weights_mpc(13 * horizon);
    VectorXd r_weights_mpc(12 * horizon);

    for (int i = 0; i < horizon; ++i)
    {
        q_weights_mpc.segment(i * 13, 13) = Q;
    }
    Qqp.diagonal() = q_weights_mpc;
    for (int i = 0; i < horizon; ++i)
    {
        r_weights_mpc.segment(i * 12, 12) = R;
    }
    Rqp.diagonal() = r_weights_mpc;

    H.resize(12*horizon, 12*horizon);
    q.resize(12*horizon);

    lb.resize(5*num_legs*horizon);
    ub.resize(5*num_legs*horizon);
}

void ConvexMPC::calc_AB_matrices(VectorXd& body_rpy, MatrixXd& foot_positions,
                                MatrixXd& A_mat, MatrixXd &B_mat)
{
    // The CoM dynamics can be written as:
    // x_dot = A x + B u
    // where x is the 13-dimensional state vector (r, p, y, x, y, z, r_dot,
    // p_dot, y_dot, vx, vy, vz, -g) constructed from the CoM
    // roll/pitch/yaw/position, and their first order derivatives. 'g' is
    // the gravity constant. u is the 3*num_legs -dimensional input vector
    // ( (fx, fy, fz) for each leg )
    // Construct A matrix (13x13) in the linearized CoM dynamics equation
    // We assume that the input rotation is in X->Y->Z order in the
    // extrinsic/fixed frame, or z->y'->x'' order in the intrinsic frame.

    // calculate A
    cos_yaw = cos(body_rpy(2));
    sin_yaw = sin(body_rpy(2));
    cos_pitch = cos(body_rpy(1));
    tan_pitch = tan(body_rpy(1));

    R_xyz << cos_yaw / cos_pitch, sin_yaw / cos_pitch, 0, 
            -sin_yaw,             cos_yaw,             0, 
             cos_yaw * tan_pitch, sin_yaw * tan_pitch, 1;

    A_mat.setZero();
    A_mat.block<3, 3>(0, 6) = R_xyz;
    A_mat.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
    A_mat(11, 12) = 1;

    // calculate B
    // B (13x(num_legs*3)) contains non_zero elements only in row 6:12.
    R_body = mors_sys::euler2mat(body_rpy(0), body_rpy(1), body_rpy(2));
    inv_inertia = R_body * robot_params.I_b.inverse() * R_body.transpose();
    B_mat.setZero();
    // cout << "---foot_pos0---" << endl;
    // cout << foot_positions.col(0) << endl;
    for (int i = 0; i < num_legs; i++)
    {
        B_mat.block<3, 3>(6, i * 3) = inv_inertia * mors_sys::skew(foot_positions.col(i)); // r x f torque
        B_mat.block<3, 3>(9, i * 3) = inv_mass * Eigen::Matrix3d::Identity(); // f = ma
    }

    // cout << "---A_mat---" << endl;
    // cout << A_mat << endl;
    // cout << "---B_mat---" << endl;
    // cout << B_mat << endl;
}

void ConvexMPC::calc_discrete_matrices(MatrixXd& A_mat, MatrixXd& B_mat, double& planning_timestep,
                                        MatrixXd& Ad_mat, MatrixXd& Bd_mat)
{
    // Calculates the discretized space-time dynamics. Given the dynamics
    // equation:
    // xdot = A x + B u
    // and a timestep dt, we can estimate the snapshot of the state at t +
    // dt by:
    // x(t + dt) = = Ad x + Bd u
    // Using explicit 1st order Euler integration with zero-order hold on u

    Ad_mat.setZero();
    Bd_mat.setZero();
    Ad_mat = MatrixXd::Identity(13, 13) + A_mat * planning_timestep;
    Bd_mat = B_mat * planning_timestep;
}

// Calculate Hessian matrix H and gradient vector q for QP
void ConvexMPC::calc_QP_matrices(int& planning_horizon_steps, VectorXd& x0, VectorXd& state_ref,
    MatrixXd& Qqp, MatrixXd& Rqp,
    MatrixXd& Ad_mat, MatrixXd& Bd_mat, 
    Sparse_Matrix& H_sparse, VectorXd& q_dense)
{
    const int state_dim = Ad_mat.cols();
    const int action_dim = Bd_mat.cols();

    MatrixXd A_qp(state_dim*planning_horizon_steps, state_dim);
    A_qp.setZero();

    MatrixXd B_qp(state_dim * planning_horizon_steps, action_dim * planning_horizon_steps);
    B_qp.setZero();

    MatrixXd H_dense(action_dim * planning_horizon_steps, action_dim * planning_horizon_steps);
    H_dense.setZero();
    q_dense.setZero();

    VectorXd Xref(state_dim*planning_horizon_steps);
    it_Xref = 0;
    for (int i = 0; i < horizon; i++)
    {
        for (int j = 0; j < state_dim; j++)
        {
            Xref(it_Xref) = state_ref(j);
            it_Xref++;
        }
    }

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // calculate A_qp and B_qp
    // A_qp = [A,
    // A^2,
    // A^3,
    // ...
    // A^k]'
    // B_qp = [A^0*B(0),
    // A^1*B(0), B(1),
    // A^2*B(0), A*B(1), B(2),
    // ...
    // A^(k-1)*B(0), A^(k-2)*B(1), A^(k-3)*B(2), ... B(k-1)]
    for (int i = 0; i < planning_horizon_steps; i++)
    {
        if (i == 0)
        {
            A_qp.block(0, 0, state_dim, state_dim) = Ad_mat;
        }
        else
        {
            A_qp.block(state_dim * i, 0, state_dim, state_dim) = A_qp.block(state_dim * (i - 1), 0, state_dim, state_dim) * Ad_mat;
        }
        for (int j = 0; j < i + 1; j++)
        {
            if (i - j == 0)
            {
                B_qp.block(state_dim * i, action_dim * j, state_dim, action_dim) = Bd_mat; // Bd.block(j * state_dim, 0, state_dim, action_dim);
            }
            else
            {
                B_qp.block(state_dim * i, action_dim * j, state_dim, action_dim) =
                    A_qp.block(state_dim * (i - j - 1), 0, state_dim, state_dim) * Bd_mat; // Bd.block(j * state_dim, 0, state_dim, action_dim);
            }
        }
    }
    ///////////////////////////////////////////////////////////////////////
    // calculate hessian
    H_dense = 2 * (B_qp.transpose() * Qqp * B_qp + Rqp);
    H_sparse = H_dense.sparseView();
    // calculate gradient
    q_dense = 2 * B_qp.transpose() * Qqp * (A_qp * x0 - Xref);
}

void ConvexMPC::calc_constraint_matrix(int& planning_horizon_steps,
                                        Vector4d& friction_coeff, Sparse_Matrix& Ac_sparse)
{
    
    Ac_dense.setZero();

    for (int i = 0; i < planning_horizon_steps * num_legs; ++i)
    {
        Ac_dense.block<5, 3>(i * 5, i * 3)
        << -1, 0, friction_coeff(0), // -fx + mu * fz
            1, 0, friction_coeff(1), // fx + mu * fz
            0, -1, friction_coeff(2), // -fy + mu * fz
            0, 1, friction_coeff(3), // fy + mu * fz
            0, 0, 1.0; // fz
    }
    Ac_sparse = Ac_dense.sparseView();
}

void ConvexMPC::calc_constraint_bounds(
                    int& planning_horizon_steps, VectorXd& contact_state,
                    double& fz_max, double& fz_min,
                    VectorXd& l, VectorXd& u)
{
    VectorXd constraint_lb_C(5 * int(num_legs) *  planning_horizon_steps);
    VectorXd constraint_ub_C(5 * int(num_legs) *  planning_horizon_steps);

    for (int i = 0; i < planning_horizon_steps; i++)
    {
        for (int j = 0; j < num_legs; j++)
        {
            if (contact_state(j) != 0)
                friction_l_u = 1000000;
            else
                friction_l_u = 0.0;

            const int row = (i * num_legs + j) * 5;
            constraint_lb_C(row) = 0.0;
            constraint_lb_C(row + 1) = 0.0;
            constraint_lb_C(row + 2) = 0.0;
            constraint_lb_C(row + 3) = 0.0;
            constraint_lb_C(row + 4) = fz_min * contact_state(j);
            // const double friction_ub = 1000000 * contact_state(j);
            constraint_ub_C(row) = friction_l_u * contact_state(j);
            constraint_ub_C(row + 1) = friction_l_u * contact_state(j);
            constraint_ub_C(row + 2) = friction_l_u * contact_state(j);
            constraint_ub_C(row + 3) = friction_l_u * contact_state(j);
            constraint_ub_C(row + 4) = fz_max * contact_state(j);
            // if (i==1)
            //     cout << contact_state(j) << endl;
        }
    }
    // cout << "---" << endl;

    l = constraint_lb_C;
    u = constraint_ub_C;
}

void ConvexMPC::init_solver()
{
    // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.settings()->setPolish(false);
    // solver.settings()->setAdaptiveRhoInterval(25) ;
    solver.settings()->setAbsoluteTolerance(1e-02);//3);
    solver.settings()->setRelativeTolerance(1e-02);//3);
    solver.settings()->setCheckTermination(1);
    // solver.settings()->setScaledTerimination(1) ;
    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(Ac.cols());
    solver.data()->setNumberOfConstraints(Ac.rows());
    solver.data()->setHessianMatrix(H);
    solver.data()->setGradient(q);
    solver.data()->setLinearConstraintsMatrix(Ac);
    solver.data()->setLowerBound(lb);
    solver.data()->setUpperBound(ub);
    // instantiate the solver
    solver.initSolver();

    VectorXd primal_variable_init(horizon * 12);
    VectorXd primal_variable_vec(12);
    primal_variable_vec << 0.0, 0.0,
        (robot_params.M_b * robot_params.g) / 4.0, 0.0, 0.0,
        (robot_params.M_b * robot_params.g) / 4.0, 0.0, 0.0,
        (robot_params.M_b * robot_params.g) / 4.0, 0.0, 0.0,
        (robot_params.M_b * robot_params.g) / 4.0;

    primal_variable_init = primal_variable_vec.replicate(horizon, 1);
    solver.setPrimalVariable(primal_variable_init);

    // solver_inited = true;
}

void ConvexMPC::update_solver()
{
    // update the QP matrices and vectors
    solver.updateHessianMatrix(H);
    solver.updateGradient(q);
    // solver.updateLinearConstraintsMatrix( Ac );
    // solver.updateLowerBound( lb ) ;
    // solver.updateUpperBound( ub ) ;
    solver.updateBounds(lb, ub);
}

VectorXd ConvexMPC::get_contact_forces(VectorXd x0, VectorXd x_ref, MatrixXd& foot_positions, VectorXd& contact_state)
{
    cur_rpy << x0(0), x0(1), x0(2);
    // cout << foot_positions.col(0).transpose() << endl;
    calc_AB_matrices(cur_rpy, foot_positions, A, B);
    calc_discrete_matrices(A, B, dt, Ad, Bd);
    calc_QP_matrices(horizon, x0, x_ref, Qqp, Rqp, Ad, Bd, H, q);
    calc_constraint_matrix(horizon, friction_coeff, Ac);
    calc_constraint_bounds(horizon, contact_state, f_max, f_min, lb, ub);

    if (!solver.isInitialized())
        init_solver();
    else
        update_solver();

    solver.solveProblem();// != OsqpEigen::ErrorExitFlag::NoError;
    VectorXd qp_solution = solver.getSolution();

    ref_grf =  -qp_solution.head<12>();
    return ref_grf;
}   