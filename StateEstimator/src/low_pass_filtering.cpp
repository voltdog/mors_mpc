#include "low_pass_filtering.hpp"

LowPassFiltering::LowPassFiltering(float dt)
{
    for (int i=0; i<3; i++)
    {
        lpf_orientation[i].reconfigureFilter(dt, ORIENTATION_CF);
        lpf_rpy_rate[i].reconfigureFilter(dt, RPY_RATE_CF);
        lpf_pos[i].reconfigureFilter(dt, POSITION_CF);
        lpf_lin_vel[i].reconfigureFilter(dt, LIN_VEL_CF); 
    }
    orientation_filt.resize(3);
    orientation_filt.setZero();
    rpy_rate_filt.resize(3);
    rpy_rate_filt.setZero();
    pos_filt.resize(3);
    pos_filt.setZero();
    lin_vel_filt.resize(3);
    lin_vel_filt.setZero();
}

// void LowPassFiltering::set_input_data(ServoData servo_in, ImuData imu_in, Odometry odom_in)
// {
//     this->servo_in = servo_in;
//     this->imu_in = imu_in;
//     this->odom_in = odom_in;
// }

VectorXd LowPassFiltering::update_orientation(VectorXd &orientation)
{
    for (int i=0; i<3; i++)
    {
        orientation_filt(i) = lpf_orientation[i].update(orientation(i));
    }
    return orientation_filt;
}

VectorXd LowPassFiltering::update_rpy_rate(VectorXd &rpy_rate)
{
    for (int i=0; i<3; i++)
    {
        rpy_rate_filt(i) = lpf_rpy_rate[i].update(rpy_rate(i));
    }
    return rpy_rate_filt;
}

VectorXd LowPassFiltering::update_position(VectorXd &pos)
{
    for (int i=0; i<3; i++)
    {
        pos_filt(i) = lpf_pos[i].update(pos(i));
    }
    return pos_filt;
}

VectorXd LowPassFiltering::update_lin_vel(VectorXd &lin_vel)
{
    for (int i=0; i<3; i++)
    {
        lin_vel_filt(i) = lpf_lin_vel[i].update(lin_vel(i));
    }
    return lin_vel_filt;
}


// void LowPassFiltering::step(ServoData &servo_out, ImuData &imu_out, Odometry &odom_out)
// {
//     for (int i=0; i<3; i++)
//     {
//         odom_out.position(i) = lpf_odom_pos[i].update(odom_in.position(i));
//         odom_out.lin_vel(i) = lpf_odom_lin_vel[i].update(odom_in.lin_vel(i));
//         odom_out.orientation(i) = lpf_odom_rpy[i].update(odom_in.orientation(i));
//         odom_out.ang_vel(i) = lpf_odom_ang_vel[i].update(odom_in.ang_vel(i));

//         imu_out.orientation_euler(i) = lpf_imu_rpy[i].update(imu_in.orientation_euler(i));
//         imu_out.ang_vel(i) = lpf_imu_ang_vel[i].update(imu_in.ang_vel(i));
//         imu_out.lin_accel(i) = lpf_imu_acc[i].update(imu_in.lin_accel(i));
//     }

//     for (int i=0; i<12; i++)
//     {
//         servo_out.pos(i) = lpf_servo_pos[i].update(servo_in.pos(i));
//         servo_out.vel(i) = lpf_servo_vel[i].update(servo_in.vel(i));
//         servo_out.torq(i) = lpf_servo_torq[i].update(servo_in.torq(i));
//     }
// }

LowPassFiltering::~LowPassFiltering()
{

}