#include "FootStepGenerator.hpp"
#include <algorithm>
#include <array>
#include <cmath>

namespace {

constexpr std::array<int, 2> kTrotDiagonalA = {R1, L2};
constexpr std::array<int, 2> kTrotDiagonalB = {L1, R2};

struct PredictedBodyPose {
    Eigen::Vector3d base_pos;
    Eigen::Matrix3d R_body;
};

bool is_actual_support_phase(int phase)
{
    return phase == STANCE || phase == EARLY_CONTACT;
}

bool is_scheduled_support_phase(int phase)
{
    return phase == STANCE || phase == LATE_CONTACT;
}

void clear_foot_sequence(FootPlanData& foot_plan)
{
    for (int i = 0; i < PREVIEW_STRIDES_HORIZON; ++i) {
        for (int j = 0; j < 4; ++j) {
            foot_plan.duration[i][j] = 0.0;
            foot_plan.foot_sequence[i][j] = {0.0, 0.0, 0.0};
        }
    }
}

double normalized_phase_or_zero(const std::vector<double>& phi_cur, int leg)
{
    if (leg < 0 || leg >= static_cast<int>(phi_cur.size()) ||
        !std::isfinite(phi_cur[leg])) {
        return 0.0;
    }

    return std::clamp(phi_cur[leg], 0.0, 1.0);
}

double remaining_stance_time(const std::vector<double>& phi_cur, int leg, double t_st)
{
    const double phi = normalized_phase_or_zero(phi_cur, leg);
    return (1.0 - phi) * t_st;
}

double diagonal_support_score(const std::vector<int>& phase_signal,
                              const std::vector<double>& phi_cur,
                              const std::array<int, 2>& diagonal,
                              double t_st)
{
    double score = 0.0;
    for (const int leg : diagonal) {
        if (leg >= 0 && leg < static_cast<int>(phase_signal.size()) &&
            is_scheduled_support_phase(phase_signal[leg])) {
            score += remaining_stance_time(phi_cur, leg, t_st);
        }
    }

    return score;
}

double diagonal_remaining_support_time(const std::vector<int>& phase_signal,
                                       const std::vector<double>& phi_cur,
                                       const std::array<int, 2>& diagonal,
                                       double t_st)
{
    double sum = 0.0;
    int count = 0;
    for (const int leg : diagonal) {
        if (leg >= 0 && leg < static_cast<int>(phase_signal.size()) &&
            is_scheduled_support_phase(phase_signal[leg])) {
            sum += remaining_stance_time(phi_cur, leg, t_st);
            ++count;
        }
    }

    if (count == 0) {
        return t_st;
    }

    const double average = sum / static_cast<double>(count);
    if (!std::isfinite(average) || average <= 0.0) {
        return t_st;
    }

    return average;
}

double positive_duration_or_fallback(double duration, double fallback)
{
    constexpr double kMinSegmentDuration = 1.0e-3;
    if (!std::isfinite(duration) || duration <= 0.0) {
        duration = fallback;
    }
    if (!std::isfinite(duration) || duration <= 0.0) {
        duration = kMinSegmentDuration;
    }

    return std::max(duration, kMinSegmentDuration);
}

bool opposite_diagonal(bool use_diagonal_a)
{
    return !use_diagonal_a;
}

void write_footstep(FootPlanData& foot_plan,
                    int stride,
                    int leg,
                    const Eigen::Vector3d& footstep_pos,
                    double duration)
{
    for (int axis = 0; axis < 3; ++axis) {
        foot_plan.foot_sequence[stride][leg][axis] = footstep_pos(axis);
    }
    foot_plan.duration[stride][leg] = duration;
}

PredictedBodyPose predicted_body_pose(const Eigen::Vector3d& base_pos,
                                      const Eigen::Matrix3d& R_body,
                                      const Eigen::Vector3d& ref_body_vel,
                                      double ref_body_yaw_vel,
                                      double touchdown_time)
{
    Eigen::Vector3d predicted_base_pos = base_pos + ref_body_vel * touchdown_time;
    predicted_base_pos(Z) = base_pos(Z);

    const Eigen::AngleAxisd yaw_prediction(ref_body_yaw_vel * touchdown_time,
                                           Eigen::Vector3d::UnitZ());
    return {predicted_base_pos, yaw_prediction.toRotationMatrix() * R_body};
}

} // namespace

FootStepGenerator::FootStepGenerator() : 
    step_planner(4),
    pre_avg_support_foot_z(0.0),
    avg_support_foot_z(0.0),
    support_leg_count(0),
    foot_plan{},
    t_sw(0.0),
    t_st(0.0)
{
    for (int i = 0; i < 4; ++i) {
        step_planner[i] = FootStepPlanner();
    }    
}

void FootStepGenerator::set_parameters(double bx, double by, double l1,
                                      double max_leg_length,
                                      const std::array<double, 4>& interleave_x,
                                      const std::array<double, 4>& interleave_y,
                                      double k1_fsp) {
    for (int i = 0; i < 4; ++i) {
        step_planner[i].set_coefficients(k1_fsp);
        step_planner[i].set_max_leg_length(max_leg_length);
    }

    step_planner[R1].set_robot_params(Eigen::Vector3d(bx + l1 + interleave_x[R1], -(by + interleave_y[R1]), 0.0));
    step_planner[L1].set_robot_params(Eigen::Vector3d(bx + l1 + interleave_x[L1],  (by + interleave_y[L1]), 0.0));
    step_planner[R2].set_robot_params(Eigen::Vector3d(-(bx + l1 + interleave_x[R2]), -(by + interleave_y[R2]), 0.0));
    step_planner[L2].set_robot_params(Eigen::Vector3d(-(bx + l1 + interleave_x[L2]),  (by + interleave_y[L2]), 0.0));

    step_planner[R1].set_physical_hip_anchor(Eigen::Vector3d(bx + l1, -by, 0.0));
    step_planner[L1].set_physical_hip_anchor(Eigen::Vector3d(bx + l1,  by, 0.0));
    step_planner[R2].set_physical_hip_anchor(Eigen::Vector3d(-(bx + l1), -by, 0.0));
    step_planner[L2].set_physical_hip_anchor(Eigen::Vector3d(-(bx + l1),  by, 0.0));
}

void FootStepGenerator::set_heightmap(const VisionBasedMap& vision_map) {
    for (auto& planner : step_planner) {
        planner.set_heightmap(vision_map);
    }
}

void FootStepGenerator::set_gait_params(double t_sw, double t_st)
{
    this->t_sw = t_sw;
    this->t_st = t_st;
}

void FootStepGenerator::generate(const std::vector<int>& phase_signal,
                                                        const std::vector<double>& phi_cur,
                                                        double ref_body_yaw_vel,
                                                        const Eigen::Vector3d& ref_body_vel,
                                                        const Eigen::Vector3d& base_pos,
                                                        const Eigen::Vector3d& base_lin_vel,
                                                        const Eigen::Vector3d& base_rpy_rate,
                                                        const Eigen::Matrix3d& R_body,
                                                        const std::vector<Eigen::Vector3d>& foot_pos_global,
                                                        FootPlanData& foot_plan) {

    clear_foot_sequence(foot_plan);

    // compute average legs Z position
    avg_support_foot_z = 0.0;
    support_leg_count = 0;
    for (int i = 0; i < 4; ++i) {
        if (is_actual_support_phase(phase_signal[i])) {
            avg_support_foot_z += foot_pos_global[i](Z);
            ++support_leg_count;
        }
    }
    if (support_leg_count > 0) {
        avg_support_foot_z /= static_cast<double>(support_leg_count);
    }
    else {
        avg_support_foot_z = pre_avg_support_foot_z;
    }

    const double half_cycle_period =
        positive_duration_or_fallback(0.5 * (t_sw + t_st), t_st);
    const double diagonal_a_score =
        diagonal_support_score(phase_signal, phi_cur, kTrotDiagonalA, t_st);
    const double diagonal_b_score =
        diagonal_support_score(phase_signal, phi_cur, kTrotDiagonalB, t_st);

    bool use_diagonal_a = diagonal_a_score >= diagonal_b_score;
    std::array<bool, PREVIEW_STRIDES_HORIZON> diagonal_queue{};
    diagonal_queue[0] = use_diagonal_a;
    for (int stride = 1; stride < PREVIEW_STRIDES_HORIZON; ++stride) {
        diagonal_queue[stride] = opposite_diagonal(diagonal_queue[stride - 1]);
    }

    const auto& current_diagonal =
        diagonal_queue[0] ? kTrotDiagonalA : kTrotDiagonalB;
    const double current_stride_duration =
        positive_duration_or_fallback(
            diagonal_remaining_support_time(phase_signal,
                                            phi_cur,
                                            current_diagonal,
                                            t_st),
            half_cycle_period);

    std::array<double, PREVIEW_STRIDES_HORIZON> segment_durations{};
    segment_durations[0] = current_stride_duration;
    for (int stride = 1; stride < PREVIEW_STRIDES_HORIZON; ++stride) {
        segment_durations[stride] = half_cycle_period;
    }

    for (const int leg : current_diagonal) {
        write_footstep(foot_plan,
                       0,
                       leg,
                       foot_pos_global[leg],
                       segment_durations[0]);
    }

    double touchdown_time = current_stride_duration;
    for (int stride = 1; stride < PREVIEW_STRIDES_HORIZON; ++stride) {
        const auto& active_diagonal =
            diagonal_queue[stride] ? kTrotDiagonalA : kTrotDiagonalB;
        const PredictedBodyPose body_pose =
            predicted_body_pose(base_pos,
                                R_body,
                                ref_body_vel,
                                ref_body_yaw_vel,
                                touchdown_time);

        for (const int leg : active_diagonal) {
            const Vector3d footstep_pos =
                step_planner[leg].step(body_pose.base_pos,
                                       body_pose.R_body,
                                       base_lin_vel,
                                       base_rpy_rate,
                                       ref_body_vel,
                                       ref_body_yaw_vel,
                                       0.0,
                                       avg_support_foot_z,
                                       t_st);
            write_footstep(foot_plan,
                           stride,
                           leg,
                           footstep_pos,
                           segment_durations[stride]);
        }

        touchdown_time += segment_durations[stride];
    }

    pre_avg_support_foot_z = avg_support_foot_z;

}
