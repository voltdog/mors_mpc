#ifndef _leg_state_hpp_
#define _leg_state_hpp_

#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <vector>
#include <Eigen/Dense>
#include <lcm/lcm-cpp.hpp>
#include "mors_msgs/imu_lcm_data.hpp"
#include "mors_msgs/servo_state_msg.hpp"
#include "mors_msgs/odometry_msg.hpp"
#include "mors_msgs/robot_state_msg.hpp"
#include "Robot.hpp"
#include "system_functions.hpp"
#include <yaml-cpp/yaml.h>
#include "structs.hpp"
#include "gm_force_observer.hpp"

#define X 0
#define Y 1
#define Z 2

#define THRESHOLD 10.0;

using namespace std;
using namespace Eigen;
using namespace YAML;

class LegState
{
    public:
        LegState(RobotPhysicalParams &robot);
        ~LegState();

        void set_contact_threshold(double threshold);
        void set_grf_observer_params(double lamb, double dt, const Eigen::VectorXd& p, Eigen::VectorXd& cur_theta);
        LegData get_leg_state(RobotData &body_state, VectorXd &theta, VectorXd &d_theta, VectorXd &tau);

    private:
        void update_configuration(RobotData &body_state, VectorXd &theta, VectorXd &d_theta);
        void calc_pos_vel_acc(RobotData &body_state, VectorXd &theta, VectorXd &d_theta);
        void calc_grf(VectorXd tau, VectorXd d_theta);
        void calc_contacts();
        VectorXd inv_dyn_force_observer(const VectorXd& tau, const MatrixXd& J, const VectorXd& G, const VectorXd& V);

        Robot robot_;
        GMBasedForceObserver gm_observer_r1, gm_observer_l1, gm_observer_r2, gm_observer_l2;

        LegData leg_data;

        // pinocchio configuration / velocity vectors (free-flyer base + 12 joints)
        VectorXd q_, v_;

        // per-leg quantities in servo order: R1, L1, R2, L2 (global/world frame)
        std::vector<Eigen::Matrix3d> J_;      // foot Jacobians (world)
        std::vector<Eigen::Matrix3d> dJ_;     // foot Jacobian time derivatives (world)
        std::vector<Eigen::Vector3d> X_;      // foot positions (world)
        std::vector<Eigen::Vector3d> dX_;     // foot velocities (world)
        std::vector<Eigen::Vector3d> ddX_;    // foot accelerations (world)
        std::vector<Eigen::Matrix3d> M_;      // joint-space inertia matrices
        std::vector<Eigen::Vector3d> C_;      // Coriolis/centrifugal vectors (C(q,v)*v)
        std::vector<Eigen::Vector3d> G_;      // gravity vectors
        std::vector<Eigen::Vector3d> f_hat_;  // estimated ground reaction forces (world)

        vector<bool> contact;
        Eigen::VectorXd p;

        double threshold;

        MatrixXd invJ_T;
        VectorXd f_hat;

};

#endif //_leg_state_hpp_
