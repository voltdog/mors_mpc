#ifndef _robot_hpp_
#define _robot_hpp_

#include <iostream>
#include "system_functions.hpp"
#include <Eigen/Dense>
#include <cmath>
// #include <vbmath.hpp>
// #include "structs.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
// #include <pinocchio/algorithm/com.hpp>

// using namespace Eigen;
using namespace std;

class Robot{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Robot();
    ~Robot() = default;

    void set_timestep(double dt);
    void BuildPinocchioModel();
    void ComputeForwardKinematics(const VectorXd &q, const VectorXd &v);
    std::vector<Eigen::Vector3d> GetToePositionsInBaseFrame();
    std::vector<Eigen::Vector3d> GetToePositionsInWorldFrame();
    std::vector<Eigen::Matrix3d> GetFootJacobian(const VectorXd &q);
    void ComputeInertiaMatrix(Eigen::VectorXd& q, Eigen::MatrixXd& inertia_matrix);
    void ComputeBiasTerms(Eigen::VectorXd& q, Eigen::VectorXd& q_dot, Eigen::VectorXd& bias_terms);
    Eigen::Vector3d getFootVelocityGlobal(int leg_id);
    bool ComputeIK_CLIK(Eigen::VectorXd &q,
                        const std::vector<Eigen::Vector3d> &desired_pos_base,
                        int max_iter = 50,
                        double tol = 1e-4,
                        double gain = 1.0,
                        double damping = 1e-6);
    void ComputeAllLegDynamics(
                        const Eigen::VectorXd& q,
                        const Eigen::VectorXd& q_dot,
                        std::vector<Eigen::Matrix3d>& M_legs,
                        std::vector<Eigen::Vector3d>& h_legs);

private:
    pinocchio::Model model_;
    pinocchio::Data data_;

    std::vector<std::string> ef_frames;
    std::string base_frame;
    std::vector<int> jac_pos_indicies;

    pinocchio::FrameIndex base_frame_id;

    Eigen::Vector3d trans_base;
    Eigen::Vector3d trans_toe;
    Eigen::Vector3d toe_position_body_aligned;

    double dt;
};

#endif //_robot_hpp_