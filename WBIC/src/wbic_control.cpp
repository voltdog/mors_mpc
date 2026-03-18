#include "../include/wbic_control.hpp"

WBIC_Control::WBIC_Control()
    : tip_pos_tasks{TipPosTask(0), TipPosTask(1), TipPosTask(2), TipPosTask(3)} {
    Sf.setZero();
    Sf.block<6, 6>(0, 0).setIdentity();

    I_nv.setIdentity();
    Nlast.setIdentity();
    task_projector_buf.setIdentity();

    dq_buf.setZero();
    qdot_buf.setZero();
    qddot_buf.setZero();
    qddot_result.setZero();
    tau_full_result.setZero();
    nle_buf.setZero();
    fr_result.setZero();
    fr_mpc_buf.setZero();
    fr_mpc_support_buf.setZero();
    dfr_result_buf.setZero();
    jcdqd_buf.setZero();
    joint_pos_result.setZero();
    joint_vel_result.setZero();
    joint_tau_result.setZero();
    da_result_buf.setZero();

    task_pos_err_buf.setZero();
    task_vel_des_buf.setZero();
    task_jtdqd_buf.setZero();
    task_acc_des_buf.setZero();
    dq_rhs_buf.setZero();
    qdot_rhs_buf.setZero();
    task_jacobian_buf.setZero();
    Jtpre_buf.setZero();
    X_task_buf.setZero();
    G_task_buf.setZero();

    contact_jacobian_buf.setZero();
    X_contact_buf.setZero();
    G_contact_buf.setZero();
    dyn_con_y_buf.setZero();
    dyn_con_jbar_buf.setZero();
    dyn_con_s_buf.setZero();
    dyn_con_s_inv_buf.setZero();
    dyn_con_identity_buf.setIdentity();

    qp_hess_buf.setZero();
    qp_grad_buf.setZero();
    qp_eq_buf.setZero();
    qp_eq_target_buf.setZero();
    qp_eq_offset_buf.setZero();
    qp_ineq_buf.setZero();
    qp_ineq_target_buf.setZero();
    qp_ineq_solver_buf.setZero();
    qp_ineq_offset_buf.setZero();
    qp_solution_buf.setZero();

    U_support_buf << 0,  0,  1,
                     0,  0, -1,
                     1,  0, -ground_fric,
                    -1,  0, -ground_fric,
                     0,  1, -ground_fric,
                     0, -1, -ground_fric;
    u_support_buf << fz_max, -fz_min, 0, 0, 0, 0;

    H_buf.setZero();
}

void WBIC_Control::update(Robot& dyn_model,
                          const ConfigVector& ref_x,
                          const GenCoordVector& ref_dx,
                          const GenCoordVector& ref_ddx,
                          const ForceVector& fr_mpc,
                          JointVector& new_q,
                          JointVector& new_dq,
                          JointVector& new_tau) {
    const int nj = n_leg * 3;

    fr_mpc_buf = fr_mpc;
    
    // ----
    // auto start{ std::chrono::steady_clock::now() };

    update_task(dyn_model, ref_x, ref_dx, ref_ddx);

    // std::chrono::duration<double, std::micro> elapsed{std::chrono::steady_clock::now() - start};
    // cout << "update_task : " << elapsed.count() << " us" << endl;
    // // ----
    // start = std::chrono::steady_clock::now();

    run_kin_wbc(dyn_model);

    // elapsed = std::chrono::steady_clock::now() - start;
    // cout << "run_kin_wbc : " << elapsed.count() << " us" << endl;
    // // ---
    // start = std::chrono::steady_clock::now();

    run_wbic(dyn_model, qddot_buf, fr_mpc_buf);

    // elapsed = std::chrono::steady_clock::now() - start;
    // cout << "run_wbic: " << elapsed.count() << " us" << endl;
    // // ---
    // cout << "===" << endl; 

    joint_pos_result =
        dyn_model.q.segment(PIN_START_IDX, nj) + dq_buf.segment(PIN_START_IDX - 1, nj);

    for (int i = 0; i < n_leg; ++i) {
        for (int j = 0; j < 3; ++j) {
            const int q_idx = i * 3 + j;
            const int v_idx = 6 + i * 3 + j;

            if (joint_pos_result[q_idx] > leg_jnt_range_max[j]) {
                joint_pos_result[q_idx] = leg_jnt_range_max[j];
                qdot_buf[v_idx] = 0.0;
                qddot_buf[v_idx] = 0.0;
                qddot_result[v_idx] = 0.0;
                joint_tau_result[q_idx] = 0.0;
            } else if (joint_pos_result[q_idx] < leg_jnt_range_min[j]) {
                joint_pos_result[q_idx] = leg_jnt_range_min[j];
                qdot_buf[v_idx] = 0.0;
                qddot_buf[v_idx] = 0.0;
                qddot_result[v_idx] = 0.0;
                joint_tau_result[q_idx] = 0.0;
            }
        }
    }

    joint_vel_result = qdot_buf.segment(PIN_START_IDX - 1, nj);

    new_q = joint_pos_result;
    new_dq = joint_vel_result;
    new_tau = joint_tau_result;
}

void WBIC_Control::set_q_entries(double Qa_entry, double Qf_entry)
{
    this->Qa_entry = Qa_entry;
    this->Qf_entry = Qf_entry;
}

void WBIC_Control::set_task_gains(double body_ori_task_kp, double body_ori_task_kd,
                                  double body_pos_task_kp, double body_pos_task_kd,
                                  double tip_pos_task_kp, double tip_pos_task_kd)
{
    body_ori_task.setKp(body_ori_task_kp * Eigen::Matrix3d::Identity());
    body_ori_task.setKd(body_ori_task_kd * Eigen::Matrix3d::Identity());

    body_pos_task.setKp(body_pos_task_kp * Eigen::Matrix3d::Identity());
    body_pos_task.setKd(body_pos_task_kd * Eigen::Matrix3d::Identity());

    this->tip_pos_task_kp = tip_pos_task_kp;
    this->tip_pos_task_kd = tip_pos_task_kd;

    for (auto& tip_task : tip_pos_tasks) {
        tip_task.setKp(tip_pos_task_kp * Eigen::Matrix3d::Identity());
        tip_task.setKd(tip_pos_task_kd * Eigen::Matrix3d::Identity());
    }
}

void WBIC_Control::update_task(Robot& dyn_model,
                               const ConfigVector& ref_x,
                               const GenCoordVector& ref_dx,
                               const GenCoordVector& ref_ddx) {
    active_task_count = 0;
    active_tasks[active_task_count++] = &body_ori_task;
    active_tasks[active_task_count++] = &body_pos_task;

    for (int i = 0; i < n_leg; ++i) {
        if (!dyn_model.is_leg_supporting(i)) {
            active_tasks[active_task_count++] = &tip_pos_tasks[i];
        }
    }

    for (int i = 0; i < active_task_count; ++i) {
        active_tasks[i]->update(dyn_model, ref_x, ref_dx, ref_ddx);
    }
}

void WBIC_Control::run_kin_wbc(Robot& dyn_model) {
    Nlast = I_nv;
    dq_buf.setZero();
    qdot_buf.setZero();
    qddot_buf.setZero();

    contact_jacobian_buf.setZero();
    int n_support = dyn_model.get_contact_jacobian_or_none(contact_jacobian_buf);
    if (n_support != 0) {
        const int rows = 3 * n_support;
        auto Jv = contact_jacobian_buf.topRows(rows);
        auto G = G_contact_buf.topLeftCorner(rows, rows);
        auto X = X_contact_buf.topRows(rows);

        G.noalias() = Jv * Jv.transpose();
        G.diagonal().array() += projector_reg;

        contact_ldlt.compute(G);
        if (contact_ldlt.info() != Eigen::Success) {
            throw std::runtime_error("LDLT failed for contact projector");
        }

        X = contact_ldlt.solve(Jv);
        Nlast.noalias() -= Jv.transpose() * X;
    }

    for (int i = 0; i < active_task_count; ++i) {
        WBCTask& task = *active_tasks[i];

        task_jacobian_buf = task.getJacobian();
        task_pos_err_buf = task.getPosErr();
        task_vel_des_buf = task.getVelDes();

        Jtpre_buf.noalias() = task_jacobian_buf * Nlast;

        G_task_buf.noalias() = Jtpre_buf * Jtpre_buf.transpose();
        G_task_buf.diagonal().array() += task_reg;

        task_ldlt.compute(G_task_buf);
        if (task_ldlt.info() != Eigen::Success) {
            throw std::runtime_error("LDLT failed for task projector");
        }

        dq_rhs_buf.noalias() = task_pos_err_buf - task_jacobian_buf * dq_buf;
        dq_buf.noalias() += Jtpre_buf.transpose() * task_ldlt.solve(dq_rhs_buf);

        qdot_rhs_buf.noalias() = task_vel_des_buf - task_jacobian_buf * qdot_buf;
        qdot_buf.noalias() += Jtpre_buf.transpose() * task_ldlt.solve(qdot_rhs_buf);

        X_task_buf.noalias() = task_ldlt.solve(Jtpre_buf);
        task_projector_buf.noalias() = Jtpre_buf.transpose() * X_task_buf;
        task_projector_buf = I_nv - task_projector_buf;
        Nlast = Nlast * task_projector_buf;
    }

    H_buf = dyn_model.get_mass_matix();
    H_buf.diagonal().array() += H_reg;
    H_ldlt.compute(H_buf);
    if (H_ldlt.info() != Eigen::Success) {
        throw std::runtime_error("LDLT failed for mass matrix");
    }

    Nlast = I_nv;
    jcdqd_buf.setZero();

    if (n_support != 0) {
        const int rows = 3 * n_support;
        int n_suplegs = 0;

        const auto& contact_jcdqd = dyn_model.get_contact_jcdqd_or_none(n_suplegs);
        jcdqd_buf.head(rows) = contact_jcdqd.head(rows);
        dyn_con_inv(contact_jacobian_buf.topRows(rows));

        qddot_buf.noalias() = dyn_con_jbar_buf.leftCols(rows) * (-jcdqd_buf.head(rows));
        Nlast.noalias() -= dyn_con_jbar_buf.leftCols(rows) * contact_jacobian_buf.topRows(rows);
    }

    for (int i = 0; i < active_task_count; ++i) {
        WBCTask& task = *active_tasks[i];

        task_jacobian_buf = task.getJacobian();
        task_jtdqd_buf = task.getJdqd();
        task_acc_des_buf = task.getAccDes();
        Jtpre_buf.noalias() = task_jacobian_buf * Nlast;

        dyn_con_inv(Jtpre_buf);

        dq_rhs_buf.noalias() = task_acc_des_buf - task_jtdqd_buf - task_jacobian_buf * qddot_buf;
        qddot_buf.noalias() += dyn_con_jbar_buf.leftCols(task_dim) * dq_rhs_buf;

        task_projector_buf.noalias() = dyn_con_jbar_buf.leftCols(task_dim) * Jtpre_buf;
        task_projector_buf = I_nv - task_projector_buf;
        Nlast = Nlast * task_projector_buf;
    }
}

void WBIC_Control::run_wbic(Robot& dyn_model,
                            const GenCoordVector& ddq_cmd,
                            const ForceVector& fr_mpc) {
    qp_var_count = 6 + dyn_model.get_n_support_legs() * 3;
    qp_contact_dim = qp_var_count - 6;

    qp_hess_buf.setZero();
    qp_grad_buf.setZero();
    qp_hess_buf.diagonal().head<6>().setConstant(Qa_entry);
    if (qp_contact_dim > 0) {
        qp_hess_buf.diagonal().segment(6, qp_contact_dim).setConstant(Qf_entry);
    }

    calc_dynamic_equation_constraint(dyn_model, ddq_cmd, fr_mpc);
    calc_support_constraint(dyn_model, fr_mpc);
    solve_qp(dyn_model, ddq_cmd, fr_mpc);
    solve_joint_tau(dyn_model);
}

void WBIC_Control::calc_dynamic_equation_constraint(Robot& dyn_model,
                                                    const GenCoordVector& ddq_cmd,
                                                    const ForceVector& fr_mpc) {
    const int n_support = dyn_model.get_n_support_legs();

    H_buf = dyn_model.get_mass_matix();
    nle_buf = dyn_model.get_nle_vector();

    contact_jacobian_buf.setZero();
    dyn_model.get_contact_jacobian_or_none(contact_jacobian_buf);

    qp_eq_buf.setZero();
    qp_eq_target_buf.noalias() = -Sf * (H_buf * ddq_cmd + nle_buf);
    qp_eq_buf.block<6, 6>(0, 0) = H_buf.block<6, 6>(0, 0);

    if (n_support > 0) {
        const int rows = 3 * n_support;
        auto Jc = contact_jacobian_buf.topRows(rows);

        qp_eq_buf.block(0, 6, 6, rows).noalias() = -Sf * Jc.transpose();

        fr_mpc_support_buf.setZero();
        int cnt = 0;
        for (int i = 0; i < n_leg; ++i) {
            if (dyn_model.is_leg_supporting(i)) {
                fr_mpc_support_buf.segment<3>(cnt * 3) = fr_mpc.segment<3>(i * 3);
                cnt++;
            }
        }

        qp_eq_target_buf.noalias() +=
            Sf * Jc.transpose() * fr_mpc_support_buf.head(rows);
    }
}

void WBIC_Control::calc_support_constraint(Robot& dyn_model,
                                           const ForceVector& fr_mpc) {
    qp_ineq_buf.setZero();
    qp_ineq_target_buf.setZero();
    qp_ineq_count = dyn_model.get_n_support_legs() * 6;

    if (qp_ineq_count > 0) {
        int cnt = 0;
        for (int i = 0; i < n_leg; ++i) {
            if (dyn_model.is_leg_supporting(i)) {
                qp_ineq_buf.block<6, 3>(cnt * 6, 6 + cnt * 3) = U_support_buf;
                qp_ineq_target_buf.segment<6>(cnt * 6) =
                    u_support_buf - U_support_buf * fr_mpc.segment<3>(i * 3);
                cnt++;
            }
        }
    }
}

void WBIC_Control::solve_qp(Robot& dyn_model,
                            const GenCoordVector& ddq_cmd,
                            const ForceVector& fr_mpc) {
    if (qp_var_count < max_qp_vars) {
        qp_hess_buf.diagonal().segment(qp_var_count, max_qp_vars - qp_var_count).setOnes();
        qp_grad_buf.tail(max_qp_vars - qp_var_count).setZero();
    }

    qp_eq_offset_buf = -qp_eq_target_buf;

    qp_ineq_solver_buf.setZero();
    qp_ineq_offset_buf.setOnes();
    if (qp_ineq_count > 0) {
        qp_ineq_solver_buf.topRows(qp_ineq_count) = -qp_ineq_buf.topRows(qp_ineq_count);
        qp_ineq_offset_buf.head(qp_ineq_count) = qp_ineq_target_buf.head(qp_ineq_count);
    }

    qp_solution_buf.setZero();

    auto status = qp.solve_quadprog(
        qp_hess_buf,
        qp_grad_buf,
        qp_eq_buf,
        qp_eq_offset_buf,
        qp_ineq_solver_buf,
        qp_ineq_offset_buf,
        qp_solution_buf);
    if (status != eiquadprog::solvers::RT_EIQUADPROG_OPTIMAL) {
        throw std::runtime_error("WBIC QP failed");
    }

    da_result_buf = qp_solution_buf.head<6>();
    dfr_result_buf.setZero();
    if (qp_contact_dim > 0) {
        dfr_result_buf.head(qp_contact_dim) = qp_solution_buf.segment(6, qp_contact_dim);
    }

    fr_result = fr_mpc;
    int cnt = 0;
    for (int i = 0; i < n_leg; ++i) {
        if (dyn_model.is_leg_supporting(i)) {
            fr_result.segment<3>(i * 3) += dfr_result_buf.segment<3>(cnt * 3);
            cnt++;
        }
    } 

    qddot_result = ddq_cmd;
    qddot_result.head<6>() += da_result_buf;
}

void WBIC_Control::solve_joint_tau(Robot& dyn_model) {
    tau_full_result = dyn_model.get_tau(qddot_result, fr_result);
    joint_tau_result = tau_full_result.segment<12>(6);
}

void WBIC_Control::dyn_con_inv(const Eigen::Ref<const MatrixXd>& J)
{
    const int rows = static_cast<int>(J.rows());
    auto Y = dyn_con_y_buf.leftCols(rows);
    auto S = dyn_con_s_buf.topLeftCorner(rows, rows);
    auto S_inv = dyn_con_s_inv_buf.topLeftCorner(rows, rows);
    auto J_bar = dyn_con_jbar_buf.leftCols(rows);

    Y.noalias() = H_ldlt.solve(J.transpose());

    S.noalias() = J * Y;
    S.diagonal().array() += projector_reg;

    dyn_con_ldlt.compute(S);
    if (dyn_con_ldlt.info() != Eigen::Success) {
        throw std::runtime_error("LDLT failed for dyn_con_inv");
    }

    S_inv.noalias() = dyn_con_ldlt.solve(dyn_con_identity_buf.topLeftCorner(rows, rows));
    J_bar.noalias() = Y * S_inv;
}
