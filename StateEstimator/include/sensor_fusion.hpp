#ifndef _sensor_fusion_hpp_
#define _sensor_fusion_hpp_

#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <cmath>
#include <Eigen/Dense>
#include <LowPassFilter.hpp>
#include "structs.hpp"

#define X 0
#define Y 1
#define Z 2

using namespace std;
using namespace Eigen;

class SensorFusion
{
    public:
        SensorFusion();
        ~SensorFusion();

        VectorXd update_orientation(VectorXd orientation_in1, VectorXd orientation_in2);
        VectorXd update_rpy_rate(VectorXd rpy_in1, VectorXd rpy_in2);
    private:
        double yaw1_tmp, yaw1, pre_yaw1, yaw1_final;
        double yaw2_tmp, yaw2, pre_yaw2;

        VectorXd in1, in2;
        VectorXd orientation_out;
        VectorXd rpy_rate_out;

        bool first_yaw1 = true;
        double offset_yaw1 = 0.0;

};


#endif //_sensor_fusion_hpp_