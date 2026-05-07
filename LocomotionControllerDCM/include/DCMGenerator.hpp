#pragma once

#include "structs.hpp"
#include "data_types.hpp"

#include <iostream>
#include <Eigen/Dense>
#include <array>
#include <optional>
#include <vector>

using namespace std;
using namespace Eigen;

class DCMGenerator {
public:
    DCMGenerator();
    ~DCMGenerator();

    void set_parameters(double dt) {
        this->dt = dt;
    }

    void generate(const FootPlanData& foot_plan,
                  const RobotData& robot_state,
                  const RobotData& robot_cmd,
                  std::vector<Eigen::Vector3d>& x_com_traj,
                  std::vector<Eigen::Vector3d>& d_x_com_traj,
                  std::vector<Eigen::Vector3d>& dd_x_com_traj);


private:
    std::optional<Eigen::Vector2d> segment_intersection(const Eigen::Vector2d& p1,
                                                        const Eigen::Vector2d& p2,
                                                        const Eigen::Vector2d& p3,
                                                        const Eigen::Vector2d& p4) const;
    Eigen::Vector3d closest_point_on_segment_to_body(const Eigen::Vector3d& body_pos,
                                                     const Eigen::Vector3d& p1,
                                                     const Eigen::Vector3d& p2) const;

    void find_r_f(const FootPlanData& foot_plan,
                  const RobotData& robot_state,
                  std::vector<Eigen::Vector3d>& r_f);

    void find_r_vrp_d(const std::vector<Eigen::Vector3d>& r_f,
                      const double& delta_z,
                      std::vector<Eigen::Vector3d>& r_vrp_d);

    void find_t_step(const FootPlanData& foot_plan,
                     const std::vector<Eigen::Vector3d>& r_f,
                     std::vector<double>& t_step);
    
    void find_dcm_d_eos(const std::vector<Eigen::Vector3d>& r_vrp_d,
                                    const double& omega,
                                    const std::vector<double>& t_step,
                                    std::vector<Eigen::Vector3d>& dcm_d_eos);

    void find_dcm_d(const std::vector<Eigen::Vector3d>& r_vrp_d,
                    const double& omega,
                    const std::vector<double>& t_step,
                    const std::vector<Eigen::Vector3d>& dcm_d_eos,
                    std::vector<Eigen::Vector3d>& dcm_d,
                    std::vector<Eigen::Vector3d>& d_dcm_d);

    void find_x_com_trajectory(const RobotData& robot_state,
                               const std::vector<Eigen::Vector3d>& dcm_d,
                               const std::vector<Eigen::Vector3d>& d_dcm_d,
                               const double omega,
                               std::vector<Eigen::Vector3d>& x_com_traj,
                               std::vector<Eigen::Vector3d>& d_x_com_traj,
                               std::vector<Eigen::Vector3d>& dd_x_com_traj);

    const double g;    
    double dt;
    double delta_z;
    double omega;
    std::vector<Eigen::Vector3d> r_f, r_vrp_d, dcm_d_eos;
    std::vector<Eigen::Vector3d> dcm_d, d_dcm_d;
    std::vector<Eigen::Vector3d> x_com_traj, d_x_com_traj, dd_x_com_traj;

    Vector3d first_support_p1;
    Vector3d first_support_p2;
};
