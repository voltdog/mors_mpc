#ifndef SWING_LEG_CONTROLLER_HPP
#define SWING_LEG_CONTROLLER_HPP

#include "FootStepPlanner.hpp"
#include "SwingTrajectoryGenerator.hpp"
#include "structs.hpp"
#include <Eigen/Dense>
#include <vector>
#include <array>
#include <tuple>

class SwingController {
public:
    SwingController(double timestep, 
        double bx, double by, double l1,
        double dz_near_ground);

    void set_gait_params(double t_sw, double t_st, double ref_stride_height);
    // void set_heightmap(const VisionBasedMap& vision_map);

    std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>
    step(const std::vector<int>& phase_signal,
         const std::vector<double>& phi_cur,
         double ref_body_height,
        //  double ref_body_yaw_vel,
        //  const Eigen::Vector3d& ref_body_vel,
         const Eigen::Vector3d& base_pos,
        //  const Eigen::Vector3d& base_lin_vel,
        //  const Eigen::Vector3d& base_rpy_rate,
         const Eigen::Matrix3d& R_body,
         const std::vector<Eigen::Vector3d>& foot_pos_global,
         const std::vector<std::array<double, 3>>& p_finish);
    // std::vector<Eigen::Vector3d> get_p_finish() const;

private:
    double dz_near_ground;
    double control_dt;
    std::vector<int> cnt;
    std::vector<double> it_swing;
    SwingTrajectoryGenerator swing_traj_gen;
    std::vector<int> pre_phase_signal;
    std::vector<std::array<double, 3>> p_start, p_rise, p_finish, d_p_start, dd_p_start;
    double t_st, t_sw;
    double ref_stride_height;
    std::vector<FootStepPlanner> step_planner;

    double avg_support_foot_z;
    int support_leg_count;
    std::vector<Eigen::Vector3d> p0_b; // hip locations in body frame
    std::vector<std::array<double, 3>> p_finish_local;
    std::vector<double> segment_start_time, segment_duration;
    std::vector<bool> single_segment_z, has_last_ref;
    std::vector<Eigen::Vector3d> last_p_ref, last_dp_ref, last_ddp_ref;
};

#endif // SWING_LEG_CONTROLLER_HPP
