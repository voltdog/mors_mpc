#include "SwingController.hpp"
#include <Eigen/LU>  // For inverse
#include <algorithm>
#include <cmath>

namespace {
constexpr double kRetargetThreshold = 0.02;
constexpr double kRetargetCooldown = 0.02;
constexpr double kMinRetargetRemainingRatio = 0.25;

Eigen::Vector3d array_to_vector(const std::array<double, 3>& value)
{
    return Eigen::Vector3d(value[X], value[Y], value[Z]);
}

void vector_to_array(const Eigen::Vector3d& value, std::array<double, 3>& out)
{
    for (int axis = 0; axis < 3; ++axis) {
        out[axis] = value(axis);
    }
}

bool is_valid_finish(const std::array<double, 3>& value)
{
    const Eigen::Vector3d finish = array_to_vector(value);
    return finish.allFinite() && finish.squaredNorm() > 1e-10;
}

double phase_to_time(const std::vector<double>& phi_cur, int leg, double t_sw)
{
    if (leg >= static_cast<int>(phi_cur.size()) || !std::isfinite(phi_cur[leg])) {
        return 0.0;
    }

    return std::clamp(phi_cur[leg], 0.0, 1.0) * t_sw;
}
} // namespace

SwingController::SwingController(double timestep, 
                                double bx, double by, double l1,
                                double dz_near_ground)
    : dz_near_ground(dz_near_ground), control_dt(timestep),
      cnt(4, -1), it_swing(4, 0.0), swing_traj_gen(1.0 / timestep),
      pre_phase_signal(4, STANCE), p_start(4), p_rise(4), p_finish(4),
      d_p_start(4, {0.0, 0.0, 0.0}), dd_p_start(4, {0.0, 0.0, 0.0}),
      p0_b(4), p_finish_local(4),
      segment_start_time(4, 0.0), segment_duration(4, timestep),
      single_segment_z(4, false), has_last_ref(4, false),
      last_p_ref(4, Eigen::Vector3d::Zero()),
      last_dp_ref(4, Eigen::Vector3d::Zero()),
      last_ddp_ref(4, Eigen::Vector3d::Zero())
{
    p0_b[R1] = Eigen::Vector3d(bx + l1, -by, 0.0);
    p0_b[L1] = Eigen::Vector3d(bx + l1,  by, 0.0);  
    p0_b[R2] = Eigen::Vector3d(-(bx + l1), -by, 0.0);
    p0_b[L2] = Eigen::Vector3d(-(bx + l1),  by, 0.0);
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
                         const Eigen::Vector3d& base_pos,
                         const Eigen::Matrix3d& R_body,
                         const std::vector<Eigen::Vector3d>& foot_pos_global,
                         const std::vector<std::array<double, 3>>& p_finish_in) {
    
    p_finish_local = p_finish_in;

    for (int i = 0; i < 4; ++i) {
        if (phase_signal[i] == SWING) {
            const double swing_time = phase_to_time(phi_cur, i, t_sw);

            if (pre_phase_signal[i] == STANCE && phase_signal[i] == SWING) {
                for (int j = 0; j < 3; ++j) {
                    p_start[i][j] = foot_pos_global[i](j); 
                    p_finish[i][j] = is_valid_finish(p_finish_local[i]) ?
                        p_finish_local[i][j] :
                        foot_pos_global[i](j);
                    d_p_start[i][j] = 0.0;
                    dd_p_start[i][j] = 0.0;
                }

                segment_start_time[i] = 0.0;
                segment_duration[i] = std::max(t_sw, control_dt);
                single_segment_z[i] = false;
                has_last_ref[i] = false;
                cnt[i] = -1;
                swing_traj_gen.reset_offsets();
            } else {
                const Eigen::Vector3d requested_finish = array_to_vector(p_finish_local[i]);
                const Eigen::Vector3d active_finish = array_to_vector(p_finish[i]);
                const double remaining_time = std::max(t_sw - swing_time, control_dt);
                const double time_since_retarget = swing_time - segment_start_time[i];

                if (is_valid_finish(p_finish_local[i]) &&
                    time_since_retarget >= std::max(kRetargetCooldown, 2.0 * control_dt) &&
                    (requested_finish - active_finish).norm() > kRetargetThreshold &&
                    remaining_time >= std::max(kMinRetargetRemainingRatio * t_sw,
                                               4.0 * control_dt)) {
                    const Eigen::Vector3d retarget_pos =
                        has_last_ref[i] ? last_p_ref[i] : foot_pos_global[i];
                    const Eigen::Vector3d retarget_vel =
                        has_last_ref[i] ? last_dp_ref[i] : Eigen::Vector3d::Zero();
                    const Eigen::Vector3d retarget_acc =
                        has_last_ref[i] ? last_ddp_ref[i] : Eigen::Vector3d::Zero();

                    vector_to_array(retarget_pos, p_start[i]);
                    vector_to_array(retarget_vel, d_p_start[i]);
                    vector_to_array(retarget_acc, dd_p_start[i]);
                    p_finish[i] = p_finish_local[i];

                    segment_start_time[i] = swing_time;
                    segment_duration[i] = remaining_time;
                    single_segment_z[i] = swing_time >= t_sw * SwingTrajectoryGenerator::rising_proportion;
                    cnt[i] = -1;
                }
            }

            Eigen::Vector3d p_hip = Eigen::Vector3d(base_pos(X), base_pos(Y), base_pos(Z)) + R_body * p0_b[i];
            double max_rise_z = p_hip(Z) - 0.03; //
            double p_rise_z = 0.0;
            if (segment_start_time[i] > 0.0) {
                p_rise_z = std::max(p_start[i][Z], p_finish[i][Z] + ref_stride_height);
            } else if ((p_finish[i][Z] - p_start[i][Z]) > 0) {
                p_rise_z = p_finish[i][Z] + ref_stride_height + abs(p_finish[i][Z] - p_start[i][Z]) * 0.2;
            } else {
                p_rise_z = p_start[i][Z] + ref_stride_height;
            }
            p_rise_z = std::min(p_rise_z, max_rise_z); 

            p_rise[i] = {{
                p_start[i][X] + 0.5 * (p_finish[i][X] - p_start[i][X]),
                p_start[i][Y] + 0.5 * (p_finish[i][Y] - p_start[i][Y]),
                p_rise_z
            }};

            it_swing[i] = std::clamp(swing_time - segment_start_time[i],
                                     0.0,
                                     std::max(segment_duration[i], control_dt));
            cnt[i] += 1;
        } else {
            p_start[i] = {{0.0, 0.0, 0.0}};
            p_rise[i] = {{0.0, 0.0, 0.0}};
            d_p_start[i] = {{0.0, 0.0, 0.0}};
            dd_p_start[i] = {{0.0, 0.0, 0.0}};
            segment_start_time[i] = 0.0;
            segment_duration[i] = std::max(t_sw, control_dt);
            single_segment_z[i] = false;
            has_last_ref[i] = false;
            cnt[i] = -1;
        }
    }


    swing_traj_gen.set_parameters(t_sw, dz_near_ground);
    swing_traj_gen.set_points(p_start, p_rise, p_finish, d_p_start, dd_p_start);
    swing_traj_gen.set_leg_timing(segment_duration, single_segment_z);
    auto [x_ref_global, d_p_ref, dd_p_ref] = swing_traj_gen.step(it_swing, cnt, phase_signal);

    std::vector<Eigen::Vector3d> x_ref_glob_out(4), dx_ref(4), ddx_ref(4);
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d x_global(&x_ref_global[i * 3]);
        Eigen::Vector3d dx_global(&d_p_ref[i * 3]);
        Eigen::Vector3d ddx_global(&dd_p_ref[i * 3]);

        x_ref_glob_out[i] = x_global;
        dx_ref[i] = dx_global;
        ddx_ref[i] = ddx_global;

        if (phase_signal[i] == SWING) {
            last_p_ref[i] = x_global;
            last_dp_ref[i] = dx_global;
            last_ddp_ref[i] = ddx_global;
            has_last_ref[i] = true;
        }
    }

    pre_phase_signal = phase_signal;
    return {x_ref_glob_out, dx_ref, ddx_ref};
}
