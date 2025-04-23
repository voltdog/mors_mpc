#include "CommandShaper.hpp"
#include "LowPassFilter.hpp" // Include the low-pass filter implementation
#include <cmath>

// Constructor
CommandShaper::CommandShaper(double dt, double c_freq)
    // : c_freq(c_freq), dt(dt), pre_phase_signal(4, STANCE),
    //   foot_pos_global_just_stance(Eigen::Matrix<double, 4, 3>::Zero()),
    //   foot_pos_local_just_stance(Eigen::Matrix<double, 4, 3>::Zero()),
    //   x_ref(12), ref_yaw_pos(0.0), ref_x_pos(0.0), ref_y_pos(0.0), body_adapt_mode(INCL_ADAPT) {
{
    this->c_freq = c_freq;
    this->dt = dt;
    pre_phase_signal = {STANCE, STANCE, STANCE, STANCE};
    foot_pos_global_just_stance.resize(4,3);
    foot_pos_global_just_stance << 0, 0, -0.035,
                                    0, 0, -0.035,
                                    0, 0, -0.035,
                                    0, 0, -0.035;
    foot_pos_local_just_stance.resize(4,3);
    foot_pos_local_just_stance.setZero();
    x_ref.resize(13);
    x_ref.setZero();
    ref_yaw_pos = 0.0;
    ref_x_pos = 0.0;
    ref_y_pos = 0.0;
    body_adapt_mode = INCL_ADAPT;

    // for (int i = 0; i < pre_phase_signal.size(); i++) 
    //     cout << pre_phase_signal[0] << endl;

    lpf_x_vel.reconfigureFilter(dt, c_freq);
    lpf_y_vel.reconfigureFilter(dt, c_freq);
    lpf_z_vel.reconfigureFilter(dt, c_freq);
    lpf_pitch_vel.reconfigureFilter(dt, c_freq);
    lpf_yaw_vel.reconfigureFilter(dt, c_freq);
    lpf_z_pos.reconfigureFilter(dt, 6.0);

    // Initialize foot positions in local frame
    double ref_z_pos = 0.0;
    double ref_body_height = 0.21;
    for (int i = 0; i < 4; ++i) {
        foot_pos_local_just_stance(i, Z) = ref_z_pos - ref_body_height;
    }

    // Initialize reference vector
    x_ref << 0.0, 0.0, 0.0, // orientation
             0.0, 0.0, 0.2, // position
             0.0, 0.0, -0.0, // angular velocity
             0.0, 0.0, 0.0, // linear velocity
             -9.81; // gravity

    ref_body_vel_filtered.resize(3);
}

// Destructor
CommandShaper::~CommandShaper() {

}

// Set adaptation mode
void CommandShaper::set_body_adaptation_mode(int mode) {
    this->body_adapt_mode = mode;
}

// Step function
Eigen::VectorXd CommandShaper::step(const std::vector<int>& phase_signal,
                                    const std::vector<Eigen::Vector3d>& foot_pos_global,
                                    const std::vector<Eigen::Vector3d>& foot_pos_local,
                                    const Eigen::Vector3d& ref_body_vel,
                                    double ref_body_yaw_vel,
                                    double ref_body_height) {
    // Apply low-pass filters to reference velocities
    ref_body_vel_filtered(X) = lpf_x_vel.update(ref_body_vel(X));
    ref_body_vel_filtered(Y) = lpf_y_vel.update(ref_body_vel(Y));
    ref_body_vel_filtered(Z) = lpf_z_vel.update(ref_body_vel(Z));
    double ref_body_yaw_vel_filtered = lpf_yaw_vel.update(ref_body_yaw_vel);

    // Update foot positions just after stance
    for (int i = 0; i < 4; ++i) {
        if ((pre_phase_signal[i] == SWING && phase_signal[i] == STANCE) ||
            (pre_phase_signal[i] == LATE && phase_signal[i] == STANCE)) {
            foot_pos_global_just_stance.row(i) = foot_pos_global[i];
            foot_pos_local_just_stance.row(i) = foot_pos_local[i];
        }
    }
    
    // Compute reference z position
    double ref_z_pos = lpf_z_pos.update(compute_ref_z_pos() + ref_body_height + 0.035);
    // double ref_z_pos = ref_body_height;

    // Compute reference pitch position
    double ref_pitch_pos = compute_ref_pitch_pos();//0.0;//

    // Update reference yaw and position
    ref_yaw_pos += ref_body_yaw_vel_filtered * dt;
    ref_x_pos += ref_body_vel_filtered(X) * dt;
    ref_y_pos += ref_body_vel_filtered(Y) * dt;

    // Update reference vector
    x_ref << 0.0, ref_pitch_pos, ref_yaw_pos,
             ref_x_pos, ref_y_pos, ref_z_pos,
             0.0, 0.0, ref_body_yaw_vel_filtered,
             ref_body_vel_filtered(X), ref_body_vel_filtered(Y), 0.0, 
             -9.81;

    // Update previous phase signal
    pre_phase_signal = phase_signal;

    return x_ref;
}

// Helper method to compute reference z position
double CommandShaper::compute_ref_z_pos() const {
    double mean_z = 0.0;
    for (int i = 0; i < 4; ++i) {
        mean_z += foot_pos_global_just_stance(i, Z);
    }
    return mean_z / 4.0;
}

// Helper method to compute reference pitch position
double CommandShaper::compute_ref_pitch_pos() const {
    Eigen::Vector3d virtual_leg1_pos = (foot_pos_local_just_stance.row(R1) + foot_pos_local_just_stance.row(L1)) / 2.0;
    Eigen::Vector3d virtual_leg2_pos = (foot_pos_local_just_stance.row(R2) + foot_pos_local_just_stance.row(L2)) / 2.0;

    double virtual_a = -(virtual_leg1_pos(Z) - virtual_leg2_pos(Z));
    double virtual_b = std::abs(virtual_leg1_pos(X) - virtual_leg2_pos(X));
    double virtual_c = std::sqrt(virtual_a * virtual_a + virtual_b * virtual_b);

    if (virtual_c != 0.0) {
        return std::asin(virtual_a / virtual_c);
    } else {
        return 0.0;
    }
}