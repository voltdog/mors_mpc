#ifndef _structs_hpp_
#define _structs_hpp_

#include <Eigen/Dense>
#include <iostream>

using namespace std;
using namespace Eigen;

constexpr int R1 = 0;
constexpr int L1 = 1;
constexpr int R2 = 2;
constexpr int L2 = 3;

constexpr int PIN_R1 = 2;
constexpr int PIN_L1 = 0;
constexpr int PIN_R2 = 3;
constexpr int PIN_L2 = 1;
constexpr int PIN_START_IDX = 7;

constexpr int X = 0;
constexpr int Y = 1;
constexpr int Z = 2;

static constexpr int INCL_ADAPT = 0;
static constexpr int HEIGHT_ADAPT = 1;
static constexpr int NOADAPT = 2;

static constexpr int SWING = 0;
static constexpr int STANCE = 1;
static constexpr int LATE_CONTACT = 2;
static constexpr int EARLY_CONTACT = 3;

static constexpr int NUM_LEGS = 4;

static constexpr int LEG_CONTROL   = 1;
static constexpr int SERVO_CONTROL = 2;

struct alignas(16) RobotData
{
    Vector3d pos;
    Vector3d lin_vel;
    Vector3d orientation;
    Vector4d orientation_quaternion;
    Vector3d ang_vel;
};

struct alignas(16) LegData
{
    Vector3d r1_grf;
    Vector3d l1_grf;
    Vector3d r2_grf;
    Vector3d l2_grf;

    Vector3d r1_pos;
    Vector3d l1_pos;
    Vector3d r2_pos; 
    Vector3d l2_pos;

    Vector3d r1_vel; 
    Vector3d l1_vel; 
    Vector3d r2_vel; 
    Vector3d l2_vel;

    Vector3d r1_acc; 
    Vector3d l1_acc; 
    Vector3d r2_acc; 
    Vector3d l2_acc;

    vector<bool> contacts;

    Vector3d r1_kp; 
    Vector3d l1_kp; 
    Vector3d r2_kp; 
    Vector3d l2_kp; 

    Vector3d r1_kd; 
    Vector3d l1_kd; 
    Vector3d r2_kd; 
    Vector3d l2_kd; 
};

struct alignas(16) RobotPhysicalParams
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

struct alignas(16) ImuData
{
    Vector3d orientation_euler;
    Vector4d orientation_quaternion;
    Vector3d ang_vel;
    Vector3d lin_accel;
};

struct alignas(16) ServoData
{
    VectorXd pos;
    VectorXd vel;
    VectorXd torq;
    VectorXd kp;
    VectorXd kd;
};

struct alignas(16) Odometry
{
    Vector3d position;
    Vector3d orientation_euler;
    Vector4d orientation_quaternion;
    Vector3d lin_vel;
    Vector3d ang_vel;
};



#endif //_structs_hpp_