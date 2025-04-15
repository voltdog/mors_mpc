#include "sensor_fusion.hpp"

SensorFusion::SensorFusion()
{
    in1.resize(3);
    in1.setZero();
    in2.resize(3);
    in2.setZero();

    yaw1 = 0.0;
    yaw2 = 0.0;
}

SensorFusion::~SensorFusion()
{

}

VectorXd SensorFusion::update_orientation(VectorXd orientation_in1, VectorXd orientation_in2)
{
    // make multirotating yaw
    // imu 1
    yaw1_tmp = fmod((2*M_PI + orientation_in1[Z] - pre_yaw1), (2*M_PI));
    if (yaw1_tmp > M_PI)
        yaw1 += (yaw1_tmp - 2*M_PI);
    else if (yaw1_tmp < -M_PI)
        yaw1 += (yaw1_tmp + 2*M_PI);
    else
        yaw1 += yaw1_tmp;
    pre_yaw1 = yaw1;

    if (first_yaw1 && yaw1 != 0.0)
    {
        first_yaw1 = false;
        offset_yaw1 = yaw1;
        yaw1_final = 0.0;
    }
    else
    {
        yaw1_final = yaw1 - offset_yaw1;
    }

    // imu 2
    yaw2_tmp = fmod((2*M_PI + orientation_in2[Z] - pre_yaw2), (2*M_PI));
    if (yaw2_tmp > M_PI)
        yaw2 += (yaw2_tmp - 2*M_PI);
    else if (yaw2_tmp < -M_PI)
        yaw2 += (yaw2_tmp + 2*M_PI);
    else
        yaw2 += yaw2_tmp;
    pre_yaw2 = yaw2;

    in1 = orientation_in1;
    in1[Z] = yaw1_final;

    in2 = orientation_in2;
    in2[Z] = yaw2;

    orientation_out = (in1 + in2)/2;
    // orientation_out(Z) = yaw2;

    return orientation_out;
}

VectorXd SensorFusion::update_rpy_rate(VectorXd rpy_rate_in1, VectorXd rpy_rate_in2)
{
    rpy_rate_out = (rpy_rate_in1 + rpy_rate_in2)/2;
    return rpy_rate_out;
}