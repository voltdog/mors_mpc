#include "FootStepPlanner.hpp"

FootStepPlanner::FootStepPlanner()
    : p0_b(0.106, -0.067, 0.0), g(9.81), h(0.22), k1(0.03), k2(0.17) {}

void FootStepPlanner::set_robot_params(const Eigen::Vector3d& p0_b_) {
    p0_b = p0_b_;
}

void FootStepPlanner::set_coefficients(double k1_, double k2_) {
    k1 = k1_;
    k2 = k2_;
}

void FootStepPlanner::set_start_position(const Eigen::Vector3d& base_pos_,
                                         const Eigen::Vector3d& base_orient_) {
    base_pos = base_pos_;
    base_orient = base_orient_;
}

Eigen::Vector3d FootStepPlanner::get_hip_location() const {
    return hip_location;
}

Eigen::Vector3d FootStepPlanner::step(const Eigen::Vector3d& body_pos,
                                      const Eigen::Matrix3d& R_body,
                                      const Eigen::Vector3d& body_lin_vel,
                                      const Eigen::Vector3d& body_ang_vel,
                                      const Eigen::Vector3d& body_lin_vel_cmd,
                                      double body_yaw_vel_cmd,
                                      double body_height_cmd,
                                      double Tst) {
    h = body_height_cmd;

    Eigen::Vector3d twisting_speed_cmd(0.0, 0.0, body_yaw_vel_cmd);

    Eigen::Vector3d p_hip = body_pos + R_body * p0_b;

    Eigen::Vector3d p_cross_omega = p0_b.cross(body_ang_vel);
    p_cross_omega.z() = 0.0;

    Eigen::Vector3d dp_hip = body_lin_vel + p_cross_omega;
    Eigen::Vector3d dp_hip_cmd = body_lin_vel_cmd + p0_b.cross(twisting_speed_cmd);

    hip_location = p_hip;

    double k_raibert = 0.5;
    Eigen::Vector3d raibert_heuristic = k_raibert * Tst * dp_hip_cmd;
    Eigen::Vector3d capture_point = k1 * (dp_hip - dp_hip_cmd);
    Eigen::Vector3d centrifugal_term = k2 * dp_hip.cross(twisting_speed_cmd);

    Eigen::Vector3d p_ef_cmd = hip_location + raibert_heuristic + capture_point + centrifugal_term
                               - Eigen::Vector3d(0.0, 0.0, h + 0.02);

    return p_ef_cmd;
}
