#ifndef COMMAND_SHAPER_HPP
#define COMMAND_SHAPER_HPP

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "LowPassFilter.hpp"

using namespace std;
using namespace Eigen;
// using namespace YAML;

class CommandShaper {
public:
    // Constants
    static constexpr int NOADAPT = 0;
    static constexpr int HEIGHT_ADAPT = 1;
    static constexpr int INCL_ADAPT = 1;

    static constexpr int SWING = 0;
    static constexpr int STANCE = 1;
    static constexpr int LATE = 2;

    static constexpr int R1 = 0;
    static constexpr int L1 = 1;
    static constexpr int R2 = 2;
    static constexpr int L2 = 3;

    static constexpr int X = 0;
    static constexpr int Y = 1;
    static constexpr int Z = 2;

    // Constructor
    CommandShaper(double dt, double c_freq = 1.0);
    ~CommandShaper();

    // Methods
    void set_body_adaptation_mode(int mode);
    Eigen::VectorXd step(const std::vector<int>& phase_signal,
                         const std::vector<Eigen::Vector3d>& foot_pos_global,
                         const std::vector<Eigen::Vector3d>& foot_pos_local,
                         const Eigen::Vector3d& ref_body_vel,
                         double ref_body_yaw_vel,
                         double ref_body_height);

private:
    // Parameters
    double c_freq;
    double dt;
    LowPassFilter lpf_x_vel, lpf_y_vel, lpf_z_vel;
    LowPassFilter lpf_pitch_vel, lpf_yaw_vel;

    // State variables
    std::vector<int> pre_phase_signal;
    Eigen::Matrix<double, 4, 3> foot_pos_global_just_stance;
    Eigen::Matrix<double, 4, 3> foot_pos_local_just_stance;
    Eigen::VectorXd x_ref;

    // Reference values
    double ref_yaw_pos;
    double ref_x_pos;
    double ref_y_pos;
    int body_adapt_mode;

    // Helper methods
    double compute_ref_z_pos() const;
    double compute_ref_pitch_pos() const;

    Eigen::VectorXd ref_body_vel_filtered;
};

#endif // COMMAND_SHAPER_HPP