 #ifndef _leg_control_hpp_
#define _leg_control_hpp_

#include <iostream>
#include <Eigen/Dense>
#include <vbmath.hpp>
#include "structs.hpp"
#include "Robot.hpp"


using namespace Eigen;
using namespace std;

// #define PIN_R1 2
// #define PIN_L1 0
// #define PIN_R2 3
// #define PIN_L2 1
// #define PIN_START_IDX 7

// #define SWING  0
// #define STANCE 1
// #define LATE_CONTACT   2
// #define EARLY_CONTACT   3

class LegControl{
public:
    LegControl();

    void calculate(LegData &leg_cmd, VectorXd &theta, VectorXd &d_theta, VectorXd &theta_ref, VectorXd &d_theta_ref, VectorXd& tau_ref);

private:
    MatrixXd J_R1, J_R2, J_L1, J_L2;
    MatrixXd M_R1, M_L1, M_R2, M_L2;
    VectorXd H_R1, H_L1, H_R2, H_L2;
    VectorXd tau_ref_r1, tau_ref_l1, tau_ref_r2, tau_ref_l2;

    Robot robot;
    VectorXd q, v;
    std::vector<Eigen::Vector3d> leg_positions;
    std::vector<Eigen::Matrix3d> leg_jacobians;
    std::vector<Eigen::Matrix3d> M_legs;
    std::vector<Eigen::Vector3d> h_legs;
};

#endif //_grf_control_hpp_