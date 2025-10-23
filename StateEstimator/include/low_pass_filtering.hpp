#ifndef _low_pass_filtering_hpp_
#define _low_pass_filtering_hpp_

#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <LowPassFilter.hpp>
#include "structs.hpp"
 
using namespace std;
using namespace Eigen;

// #define ODOM_POS_CF 25.0
// #define ODOM_LIN_VEL_CF 25.0
// #define ODOM_RPY_CF 25.0
// #define ODOM_ANG_VEL_CF 25.0

// #define SERVO_POS_CF 30.0
// #define SERVO_VEL_CF 30.0
// #define SERVO_TORQ_CF 30.0

// #define IMU_RPY_CF 30.0
// #define IMU_ANG_VEL_CF 30.0
// #define IMU_ACC_CF 200.0

#define ORIENTATION_CF 100.0 //24.0
#define RPY_RATE_CF 100.0 //24.0
#define POSITION_CF 200.0 //24.0
#define LIN_VEL_CF 100.0 //24.0

class LowPassFiltering
{
    public:
        LowPassFiltering(float dt);
        ~LowPassFiltering();

        // void set_input_data(ServoData servo_in, ImuData imu_in, Odometry odom_in);
        VectorXd update_orientation(VectorXd &orientation);
        VectorXd update_rpy_rate(VectorXd &rpy_rate);
        VectorXd update_position(VectorXd &pos);
        VectorXd update_lin_vel(VectorXd &lin_vel);
        // void step(ServoData &servo_out, ImuData &imu_out, Odometry &odom_out);
    private:
        // LowPassFilter lpf_odom_pos[3];
        // LowPassFilter lpf_odom_lin_vel[3];
        // LowPassFilter lpf_odom_rpy[3];
        // LowPassFilter lpf_odom_ang_vel[3];
        // LowPassFilter lpf_servo_pos[12];
        // LowPassFilter lpf_servo_vel[12];
        // LowPassFilter lpf_servo_torq[12];
        // LowPassFilter lpf_imu_rpy[3];
        // LowPassFilter lpf_imu_ang_vel[3];
        // LowPassFilter lpf_imu_acc[3];

        LowPassFilter lpf_orientation[3];
        LowPassFilter lpf_rpy_rate[3];
        LowPassFilter lpf_pos[3];
        LowPassFilter lpf_lin_vel[3];

        ServoData servo_in;
        ImuData imu_in;
        Odometry odom_in;

        VectorXd orientation_filt;
        VectorXd rpy_rate_filt;
        VectorXd pos_filt;
        VectorXd lin_vel_filt;
};


#endif //_low_pass_filtering_hpp_