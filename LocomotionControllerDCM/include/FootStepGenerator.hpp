#pragma once

#include "FootStepPlanner.hpp"
#include "structs.hpp"
#include "data_types.hpp"

using namespace Eigen;
using namespace std;

#define PREVIEW_STRIDES_HORIZON 6

class FootStepGenerator {
public:
    FootStepGenerator();

    void set_parameters(double bx, double by, double l1,
                      double max_leg_length,
                      const std::array<double, 4>& interleave_x,
                      const std::array<double, 4>& interleave_y,
                      double k1_fsp);
    void set_gait_params(double t_sw, double t_st);
    void set_heightmap(const VisionBasedMap& vision_map);

    void generate(const std::vector<int>& phase_signal,
                                        const std::vector<double>& phi_cur,
                                        double ref_body_yaw_vel,
                                        const Eigen::Vector3d& ref_body_vel,
                                        const Eigen::Vector3d& base_pos,
                                        const Eigen::Vector3d& base_lin_vel,
                                        const Eigen::Vector3d& base_rpy_rate,
                                        const Eigen::Matrix3d& R_body,
                                        const std::vector<Eigen::Vector3d>& foot_pos_global,
                                        FootPlanData& foot_plan);

private:
    std::vector<FootStepPlanner> step_planner;

    double pre_avg_support_foot_z, avg_support_foot_z;
    int support_leg_count;
    FootPlanData foot_plan;
    double t_sw, t_st;
};
