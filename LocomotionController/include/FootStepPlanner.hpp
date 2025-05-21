#ifndef FOOT_STEP_PLANNER_HPP
#define FOOT_STEP_PLANNER_HPP

#include <Eigen/Dense>
#include "structs.hpp"

class FootStepPlanner {
public:
    FootStepPlanner();

    void set_robot_params(const Eigen::Vector3d& p0_b);
    void set_coefficients(double k1, double k2);
    void set_start_position(const Eigen::Vector3d& base_pos,
                            const Eigen::Vector3d& base_orient);
    Eigen::Vector3d get_hip_location() const;

    Eigen::Vector3d step(const Eigen::Vector3d& body_pos,
                         const Eigen::Matrix3d& R_body,
                         const Eigen::Vector3d& body_lin_vel,
                         const Eigen::Vector3d& body_ang_vel,
                         const Eigen::Vector3d& body_lin_vel_cmd,
                         double body_yaw_vel_cmd,
                         double body_height_cmd,
                         double Tst);

private:
    Eigen::Vector3d p0_b;
    Eigen::Vector3d base_pos;
    Eigen::Vector3d base_orient;
    Eigen::Vector3d hip_location;

    double g;
    double h;
    double k1;
    double k2;
};

#endif // FOOT_STEP_PLANNER_HPP
