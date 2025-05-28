#include "ikine.hpp"
#include <cmath>
#include <algorithm>

// IKineQuadruped::IKineQuadruped(double bx_, double by_, double bz_,
//                                double L1_, double L2_, double L3_,
//                                const Eigen::VectorXd& theta_offset_)
//     : bx(bx_), by(by_), bz(bz_), L1(L1_), L2(L2_), L3(L3_), theta_offset(theta_offset_)
// {
//     theta_ref.setZero(12);
//     theta2_prev = 0.0;

//     MAX_EF = {{
//         {0, 0.5}, {-0.3, 0.05}, {-0.35, -0.1},  // R1
//         {0, 0.5}, {-0.05, 0.3}, {-0.35, -0.1},  // L1
//         {-0.5, 0}, {-0.3, 0.05}, {-0.35, -0.1}, // R2
//         {-0.5, 0}, {-0.05, 0.3}, {-0.35, -0.1}  // L2
//     }};
// }

IKineQuadruped::IKineQuadruped()
{
    bx = 0.1655;
    by = 0.066;
    bz = 0.0;
    L1 = 0.09585;
    L2 = 0.13;
    L3 = 0.1485;
    theta_offset.resize(3);
    theta_offset << 0, -M_PI/2, 0;

    theta_ref.setZero(12);
    theta2_prev = 0.0;

    MAX_EF = {{
        {0, 0.5}, {-0.3, 0.05}, {-0.35, -0.1},  // R1
        {0, 0.5}, {-0.05, 0.3}, {-0.35, -0.1},  // L1
        {-0.5, 0}, {-0.3, 0.05}, {-0.35, -0.1}, // R2
        {-0.5, 0}, {-0.05, 0.3}, {-0.35, -0.1}  // L2
    }};
}

Eigen::VectorXd IKineQuadruped::restrict_pos(const Eigen::VectorXd& p_ref) {
    Eigen::VectorXd result = p_ref;
    for (int i = 0; i < 12; ++i) {
        result[i] = std::clamp(p_ref[i], MAX_EF[i][0], MAX_EF[i][1]);
    }
    return result;
}

Eigen::VectorXd IKineQuadruped::calculate(Eigen::VectorXd p_ref, const std::string& config) {
    for (int i = 0; i < 4; ++i) {
        if (p_ref[3 * i + 2] >= -0.1) {
            p_ref[3 * i + 2] = -0.1;
        }
    }

    Eigen::VectorXd r1 = ikine_R1(p_ref.segment<3>(0), config);
    Eigen::VectorXd l1 = ikine_L1(p_ref.segment<3>(3), config);
    Eigen::VectorXd r2 = ikine_R2(p_ref.segment<3>(6), config);
    Eigen::VectorXd l2 = ikine_L2(p_ref.segment<3>(9), config);

    for (int i = 0; i < 3; ++i) {
        theta_ref(i)     = r1[i];
        theta_ref(i + 3) = l1[i];
        theta_ref(i + 6) = r2[i];
        theta_ref(i + 9) = l2[i];
    }

    return theta_ref;
}

// Helper function for common calculations
static double safe_sqrt(double val) {
    return std::sqrt(std::max(0.0, val));
}

static double safe_acos(double val) {
    return std::acos(std::clamp(val, -1.0, 1.0));
}

Eigen::VectorXd IKineQuadruped::ikine_R1(const Eigen::VectorXd& p, const std::string& config) {
    int sign1 = -1;//, sign2 = -1;
    // if (config == "x") sign2 = 1;
    // else if (config == "o") sign1 = 1;
    if (config == "o") sign1 = 1;

    double px = p[2];
    double py = -by - p[1];
    double pz = p[0] - bx;

    double a2 = px*px + py*py - L1*L1;
    double theta1 = (py > 0) ?
        (std::atan2(py, px) - std::atan2(L1, -safe_sqrt(a2))) :
        (std::atan2(py, px) + 2 * M_PI - std::atan2(L1, -safe_sqrt(a2)));

    double a3 = (px*px + py*py + pz*pz - L1*L1 - L2*L2 - L3*L3) / (2 * L2 * L3);
    double theta3 = sign1 * safe_acos(a3);

    double a1 = std::pow(L3 * std::sin(theta3), 2) + std::pow(L3 * std::cos(theta3) + L2, 2) - pz*pz;
    double theta2 = std::atan2(L3 * std::cos(theta3) + L2, L3 * std::sin(theta3)) - std::atan2(pz, safe_sqrt(a1));

    // return Eigen::VectorXd(theta1, -theta2, -theta3) - theta_offset;
    Eigen::VectorXd result(3);
    result << theta1, -theta2, -theta3;
    return result - theta_offset;
}

Eigen::VectorXd IKineQuadruped::ikine_R2(const Eigen::VectorXd& p, const std::string& config) {
    int sign1 = (config == "x") ? 1 : -1;

    double px = p[2];
    double py = -by - p[1];
    double pz = p[0] + bx;

    double a2 = px*px + py*py - L1*L1;
    double theta1 = (py > 0) ?
        (std::atan2(py, px) - std::atan2(L1, -safe_sqrt(a2))) :
        (std::atan2(py, px) + 2 * M_PI - std::atan2(L1, -safe_sqrt(a2)));

    double a3 = (px*px + py*py + pz*pz - L1*L1 - L2*L2 - L3*L3) / (2 * L2 * L3);
    double theta3 = sign1 * safe_acos(a3);

    double a1 = std::pow(L3 * std::sin(theta3), 2) + std::pow(L3 * std::cos(theta3) + L2, 2) - pz*pz;
    double theta2 = std::atan2(L3 * std::cos(theta3) + L2, L3 * std::sin(theta3)) - std::atan2(pz, safe_sqrt(a1)) - M_PI;

    // return Eigen::VectorXd(-theta1, -theta2, -theta3) + theta_offset;
    Eigen::VectorXd result(3);
    result << -theta1, -theta2, -theta3;
    return result + theta_offset;
}

Eigen::VectorXd IKineQuadruped::ikine_L1(const Eigen::VectorXd& p, const std::string& config) {
    int sign1 = (config == "o") ? 1 : -1;

    double px = p[2];
    double py = by - p[1];
    double pz = p[0] - bx;

    double a2 = px*px + py*py - L1*L1;
    double theta1 = (py > 0) ?
        (std::atan2(py, px) - std::atan2(L1, safe_sqrt(a2))) - M_PI :
        (std::atan2(py, px) + 2 * M_PI - std::atan2(L1, safe_sqrt(a2))) - M_PI;

    double a3 = (px*px + py*py + pz*pz - L1*L1 - L2*L2 - L3*L3) / (2 * L2 * L3);
    double theta3 = sign1 * safe_acos(a3);

    double a1 = std::pow(L3 * std::sin(theta3), 2) + std::pow(L3 * std::cos(theta3) + L2, 2) - pz*pz;
    double theta2 = std::atan2(L3 * std::cos(theta3) + L2, L3 * std::sin(theta3)) - std::atan2(pz, safe_sqrt(a1));

    // return Eigen::VectorXd(theta1, theta2, theta3) + theta_offset;
    Eigen::VectorXd result(3);
    result << theta1, theta2, theta3;
    return result + theta_offset;
}

Eigen::VectorXd IKineQuadruped::ikine_L2(const Eigen::VectorXd& p, const std::string& config) {
    int sign1 = (config == "x") ? 1 : -1;

    double px = p[2];
    double py = by - p[1];
    double pz = p[0] + bx;

    double a2 = px*px + py*py - L1*L1;
    double theta1 = -((py > 0) ?
        (std::atan2(py, px) - std::atan2(L1, safe_sqrt(a2))) - M_PI :
        (std::atan2(py, px) + 2 * M_PI - std::atan2(L1, safe_sqrt(a2))) - M_PI);

    double a3 = (px*px + py*py + pz*pz - L1*L1 - L2*L2 - L3*L3) / (2 * L2 * L3);
    double theta3 = sign1 * safe_acos(a3);

    double a1 = std::pow(L3 * std::sin(theta3), 2) + std::pow(L3 * std::cos(theta3) + L2, 2) - pz*pz;
    double theta2 = std::atan2(L3 * std::cos(theta3) + L2, L3 * std::sin(theta3)) - std::atan2(pz, safe_sqrt(a1)) - M_PI;

    // return Eigen::VectorXd(theta1, theta2, theta3) - theta_offset;
    Eigen::VectorXd result(3);
    result << theta1, theta2, theta3;
    return result - theta_offset;
}
