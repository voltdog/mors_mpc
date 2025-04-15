#ifndef _structs_hpp_
#define _structs_hpp_

#include <Eigen/Dense>
#include <iostream>

using namespace std;
using namespace Eigen;

struct RobotData
{
    VectorXd pos;
    VectorXd lin_vel;
    VectorXd orientation;
    VectorXd ang_vel;
};

struct LegData
{
    VectorXd r1_grf;
    VectorXd l1_grf;
    VectorXd r2_grf;
    VectorXd l2_grf;

    VectorXd r1_pos;
    VectorXd l1_pos;
    VectorXd r2_pos; 
    VectorXd l2_pos;

    VectorXd r1_vel; 
    VectorXd l1_vel; 
    VectorXd r2_vel; 
    VectorXd l2_vel;

    VectorXd r1_acc; 
    VectorXd l1_acc; 
    VectorXd r2_acc; 
    VectorXd l2_acc;

    VectorXd contacts;

    VectorXd kp; 
    VectorXd kd; 
};

struct RobotPhysicalParams
{
    double M_b;
    MatrixXd I_b;
    double bx, by;

    double m1, m2, m3;
    double l1, l2, l3;
    double d1, d2, d3;
    double l_cx_3, l_cz_2;
    
    double g;

    double tau_max_array[3];
    double kt;
    double gear_ratio;
};

struct ImuData
{
    VectorXd orientation_euler;
    VectorXd orientation_quaternion;
    VectorXd ang_vel;
    VectorXd lin_accel;
};

struct ServoData
{
    VectorXd pos;
    VectorXd vel;
    VectorXd torq;
    VectorXd kp;
    VectorXd kd;
};

struct Odometry
{
    VectorXd position;
    VectorXd orientation;
    VectorXd lin_vel;
    VectorXd ang_vel;
};



#endif //_structs_hpp_