#include "SwingController.hpp"
#include <Eigen/LU>  // For inverse
#include <algorithm>

SwingController::SwingController(double timestep, double bx, double by, double l1,
                                       const std::array<double, 4>& interleave_x,
                                       const std::array<double, 4>& interleave_y,
                                       double dz_near_ground, double k1_fsp, double k2_fsp)
    : dz_near_ground(dz_near_ground), cnt(4, -1), it_swing(4, 0.0), swing_traj_gen(1.0 / timestep),
      pre_phase_signal(4, STANCE), p_start(4), p_rise(4), p_finish(4), d_p_start(4, {0.0, 0.0, 0.0}),
      step_planner(4)
{
    for (int i = 0; i < 4; ++i) {
        step_planner[i] = FootStepPlanner();
        step_planner[i].set_coefficients(k1_fsp, k2_fsp);
    }
    step_planner[R1].set_robot_params(Eigen::Vector3d(bx + l1 + interleave_x[R1], -(by + interleave_y[R1]), 0.0));
    step_planner[L1].set_robot_params(Eigen::Vector3d(bx + l1 + interleave_x[L1],  (by + interleave_y[L1]), 0.0));
    step_planner[R2].set_robot_params(Eigen::Vector3d(-(bx + l1 + interleave_x[R2]), -(by + interleave_y[R2]), 0.0));
    step_planner[L2].set_robot_params(Eigen::Vector3d(-(bx + l1 + interleave_x[L2]),  (by + interleave_y[L2]), 0.0));
}

void SwingController::set_gait_params(double t_sw, double t_st, double ref_stride_height) {
    this->t_sw = t_sw;
    this->t_st = t_st;
    this->ref_stride_height = ref_stride_height;
    swing_traj_gen.set_parameters(t_sw, dz_near_ground);
}

std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>
SwingController::step(const std::vector<int>& phase_signal,
                         const std::vector<double>& phi_cur,
                         double ref_body_height,
                         double ref_body_yaw_vel,
                         const Eigen::Vector3d& ref_body_vel,
                         const Eigen::Vector3d& base_pos,
                         const Eigen::Vector3d& base_lin_vel,
                         const Eigen::Vector3d& base_rpy_rate,
                         const Eigen::Matrix3d& R_body,
                         const std::vector<Eigen::Vector3d>& foot_pos_global) {
    for (int i = 0; i < 4; ++i) {
        if (phase_signal[i] == SWING) {
            if (pre_phase_signal[i] == STANCE && phase_signal[i] == SWING) {
                for (int j = 0; j < 3; ++j) p_start[i][j] = foot_pos_global[i][j];
                cnt[i] = -1;
                swing_traj_gen.reset_offsets();
            }

            Eigen::Vector3d p_finish_i = step_planner[i].step(base_pos, R_body, base_lin_vel, base_rpy_rate,
                                                               ref_body_vel, ref_body_yaw_vel,
                                                               ref_body_height, t_st);
            for (int j = 0; j < 3; ++j) p_finish[i][j] = p_finish_i[j];

            double max_rise_z = step_planner[i].get_hip_location()[Z] + ref_body_height - 0.03;
            double p_rise_z = p_start[i][Z] + ref_stride_height;
            p_rise_z = std::min(p_rise_z, max_rise_z);

            p_rise[i] = {{
                p_start[i][X] + 0.5 * (p_finish[i][X] - p_start[i][X]),
                p_start[i][Y] + 0.5 * (p_finish[i][Y] - p_start[i][Y]),
                p_rise_z
            }};

            it_swing[i] = phi_cur[i] * t_sw;
            cnt[i] += 1;
        } else {
            p_start[i] = {{0.0, 0.0, 0.0}};
            p_rise[i] = {{0.0, 0.0, 0.0}};
            p_finish[i] = {{0.0, 0.0, 0.0}};
            cnt[i] = -1;
        }
    }

    swing_traj_gen.set_parameters(t_sw, dz_near_ground);
    swing_traj_gen.set_points(p_start, p_rise, p_finish, d_p_start);
    auto [x_ref_global, d_p_ref, dd_p_ref] = swing_traj_gen.step(it_swing, cnt, phase_signal);

    Eigen::Matrix3d inv_R_body = R_body.inverse();

    std::vector<Eigen::Vector3d> x_ref_local(4), dx_ref(4), ddx_ref(4);
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d x_global(&x_ref_global[i * 3]);
        Eigen::Vector3d dx_global(&d_p_ref[i * 3]);
        Eigen::Vector3d ddx_global(&dd_p_ref[i * 3]);

        x_ref_local[i] = inv_R_body * (x_global - base_pos);
        dx_ref[i] = inv_R_body * dx_global;
        ddx_ref[i] = inv_R_body * ddx_global;
    }

    pre_phase_signal = phase_signal;
    return {x_ref_local, dx_ref, ddx_ref};
}
