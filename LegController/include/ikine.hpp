#pragma once

#include <Eigen/Dense>
#include <vector>
#include <array>
#include <string>
#include "structs.hpp"

class IKineQuadruped {
public:
    IKineQuadruped();
    // IKineQuadruped(double bx = 0.1655, double by = 0.066, double bz = 0.0,
    //                double L1 = 0.09585, double L2 = 0.13, double L3 = 0.1485,
    //                const Eigen::VectorXd& theta_offset = Eigen::VectorXd::Zero());

    

    Eigen::VectorXd calculate(Eigen::VectorXd p_ref, const std::string& config = "m");
    Eigen::VectorXd restrict_pos(const Eigen::VectorXd& p_ref);

    Eigen::VectorXd ikine_R1(const Eigen::VectorXd& p_ref, const std::string& config);
    Eigen::VectorXd ikine_L1(const Eigen::VectorXd& p_ref, const std::string& config);
    Eigen::VectorXd ikine_R2(const Eigen::VectorXd& p_ref, const std::string& config);
    Eigen::VectorXd ikine_L2(const Eigen::VectorXd& p_ref, const std::string& config);

private:
    double bx, by, bz;
    double L1, L2, L3;
    Eigen::VectorXd theta_offset;
    Eigen::VectorXd theta_ref;
    double theta2_prev;

    std::array<std::array<double, 2>, 12> MAX_EF;

    
};
