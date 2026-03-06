#include "lcm_data_exchange_se_lkf.hpp"
// #include "structs.hpp"
#include "Robot.hpp"
#include <LinearKalmanFilter.hpp>
#include <cmath>

using namespace std;
using namespace Eigen;
using namespace YAML;
using namespace std::chrono;

// current time
auto now()
{
  return std::chrono::steady_clock::now(); 
}

// void init_vectors(RobotData &robot_state, LegData &leg_state)
// {
//     robot_state.pos.resize(3);
//     robot_state.pos.setZero();
//     robot_state.lin_vel.resize(3);
//     robot_state.lin_vel.setZero();
//     robot_state.orientation.resize(3);
//     robot_state.orientation.setZero();
//     robot_state.ang_vel.resize(3);
//     robot_state.ang_vel.setZero();

//     leg_state.contacts.resize(4);
//     leg_state.contacts = {false, false, false, false};
//     leg_state.r1_pos.resize(3);
//     leg_state.r1_pos.setZero();
//     leg_state.l1_pos.resize(3);
//     leg_state.l1_pos.setZero();
//     leg_state.r2_pos.resize(3);
//     leg_state.r2_pos.setZero();
//     leg_state.l2_pos.resize(3);
//     leg_state.l2_pos.setZero();

//     leg_state.r1_vel.resize(3);
//     leg_state.r1_vel.setZero();
//     leg_state.l1_vel.resize(3);
//     leg_state.l1_vel.setZero();
//     leg_state.r2_vel.resize(3);
//     leg_state.r2_vel.setZero();
//     leg_state.l2_vel.resize(3);
//     leg_state.l2_vel.setZero();

//     leg_state.r1_acc.resize(3);
//     leg_state.r1_acc.setZero();
//     leg_state.l1_acc.resize(3);
//     leg_state.l1_acc.setZero();
//     leg_state.r2_acc.resize(3);
//     leg_state.r2_acc.setZero();
//     leg_state.l2_acc.resize(3);
//     leg_state.l2_acc.setZero();

//     leg_state.r1_grf.resize(3);
//     leg_state.r1_grf.setZero();
//     leg_state.l1_grf.resize(3);
//     leg_state.l1_grf.setZero();
//     leg_state.r2_grf.resize(3);
//     leg_state.r2_grf.setZero();
//     leg_state.l2_grf.resize(3);
//     leg_state.l2_grf.setZero();
// }

void init_lkf_matrices(double dt, MatrixXd &F, MatrixXd &G, MatrixXd &H, MatrixXd &Q, MatrixXd &R, MatrixXd &P0)
{
    // state transition matrix
    F.setIdentity();
    F(0, 1) = dt;
    F(1, 2) = dt;
    F(3, 4) = dt;
    F(4, 5) = dt;
    F(6, 7) = dt;
    F(7, 8) = dt;
    F(0, 2) = 0.5 * dt * dt;
    F(3, 5) = 0.5 * dt * dt;
    F(6, 8) = 0.5 * dt * dt;
    // control input matrix
    G.setZero();
    // observation matrix
    // H.setZero();
    // H.block(0, 1, 2, 2) = MatrixXd::Identity(2, 2);
    // H.block(2, 4, 2, 2) = MatrixXd::Identity(2, 2);
    // H.block(4, 7, 2, 2) = MatrixXd::Identity(2, 2);
    MatrixXd Hm(4, 3);
    Hm << 1, 0, 0,
          0, 1, 0,
          0, 1, 0,
          0, 0, 1;
    H.setZero();
    H.block(0, 0, 4, 3) = Hm;
    H.block(4, 3, 4, 3) = Hm;
    H.block(8, 6, 4, 3) = Hm;

    // process noise covariance
    double sigma_a = 1e-1;
    MatrixXd Qm(3, 3);
    Qm << pow(dt, 4) / 4, pow(dt, 3) / 2, pow(dt, 2) / 2,
          pow(dt, 3) / 2, pow(dt, 2),     dt,
          pow(dt, 2) / 2, dt,             1;
    Q.setIdentity();
    Q.block(0, 0, 3, 3) = sigma_a * Qm;
    Q.block(3, 3, 3, 3) = sigma_a * Qm;
    Q.block(6, 6, 3, 3) = sigma_a * Qm;
         
    // measurement noise covariance
    double sigma_z = 1e-11;
    R.setIdentity();
    R = R * sigma_z;

    // initial estimation error covariance
    P0.setIdentity();
    P0 = P0 * 1e-1;
}

int main() 
{
    cout << "[StateEstimatorLKF]: starting..." << endl;
    // load config
    string config_address = mors_sys::GetEnv("CONFIGPATH");
    string robot_config_address = config_address + "/robot_config.yaml";

    YAML::Node robot_config = YAML::LoadFile(robot_config_address);
    RobotPhysicalParams robot_prms;
    robot_prms.g = robot_config["g"].as<double>(); 
    robot_prms.kt = robot_config["kt"].as<double>(); 
    robot_prms.gear_ratio = robot_config["gear_ratio"].as<double>(); 

    string dt_config_address = config_address + "/timesteps.yaml";
    YAML::Node dt_config = YAML::LoadFile(dt_config_address);
    double module_dt = dt_config["state_estimator_dt"].as<double>(); 

    auto dt = std::chrono::duration<double>(module_dt);//1ms;

    // define lcm exchanger
    LCMExchanger lcmExch;
    lcmExch.start_exchanger();
    std::this_thread::sleep_for(5ms);

    // define input data
    ImuData imu_data;
    ServoData servo_state;
    Odometry odometry;
    std::vector<bool> contact_states(4, false);

    // defin output data
    RobotData robot_state;
    LegData leg_state;
    leg_state.contacts.resize(4);
    leg_state.contacts = {false, false, false, false};
    // init_vectors(robot_state, leg_state);

    // define robot model
    Robot robot;
    robot.BuildPinocchioModel();

    // define data for forward kinematics
    VectorXd q(19);
    q.setZero();
    VectorXd v(18);
    v.setZero();

    // define data for leg odometry
    std::vector<Eigen::Vector3d> pin_leg_positions(4); 
    std::vector<Eigen::Vector3d> leg_positions(4);
    std::vector<Eigen::Vector3d> leg_positions_world(4);
    std::vector<Eigen::Matrix3d> leg_jacobians(4);
    std::vector<Eigen::Matrix3d> pin_leg_jacobians(4);
    std::vector<Eigen::Vector3d> foot_vels(4);
    // foot_vels.resize(4);
    Eigen::Vector3d leg_odom_velocity;
    Eigen::Vector3d rel_vel;
    Eigen::Vector3d foot_vel_global;
    Eigen::Matrix3d w_R_b; // base rotation matrix

    // define data for LKF
    Eigen::Vector3d gravity(0, 0, robot_prms.g);
    int lkf_type = 1; // 0 for imu and leg odometry, 1 for imu, leg odometry and vision
    Eigen::MatrixXd F_lkf(9, 9); // state transition matrix
    Eigen::MatrixXd G_lkf(9, 1); // control input matrix
    Eigen::MatrixXd Q_lkf(9, 9); // process noise covariance
    Eigen::MatrixXd R_lkf; // measurement noise covariance
    Eigen::MatrixXd P0_lkf(9, 9); // initial estimation error covariance
    Eigen::VectorXd x_lkf(9); // state vector
    Eigen::VectorXd u_lkf(1); // control input vector
    Eigen::MatrixXd H_lkf;
    Eigen::VectorXd z_lkf;
    if (lkf_type == 0)
    {
        R_lkf.resize(6, 6); // measurement noise covariance
        H_lkf.resize(6, 9); // measurement matrix
        z_lkf.resize(6); // measurement vector
    }
    else if (lkf_type == 1)
    {
        R_lkf.resize(12, 12); // measurement noise covariance
        H_lkf.resize(12, 9); // measurement matrix
        z_lkf.resize(12); // measurement vector
    }
    init_lkf_matrices(module_dt, F_lkf, G_lkf, H_lkf, Q_lkf, R_lkf, P0_lkf);
    x_lkf.setZero();
    x_lkf(6) = 0.036;
    LinearKalmanFilter lkf(F_lkf, G_lkf, H_lkf, Q_lkf, R_lkf);
    lkf.set_initial_estimate(x_lkf, P0_lkf);


    cout << "[StateEstimatorLKF]: started" << endl;

    double t = 0.0;
    while(true)
    {
        // Calculating current time
        const auto start{ now() };

        // Put your code here
        // -----------------------------------------------

        // get raw sensor data
        imu_data = lcmExch.getImuData();
        servo_state = lcmExch.getServoStateData();
        servo_state.torq *= (robot_prms.kt/robot_prms.gear_ratio);
        odometry = lcmExch.getOdometry();
        contact_states = lcmExch.getContactData();

        // compute forward kinematics
        q.segment(0, 3) = odometry.position;
        q.segment(3, 4) = odometry.orientation_quaternion; 
        q.segment(7, 3) = servo_state.pos.segment(3, 3); // order of legs in pinocchio model is L1, L2, R1, R2
        q.segment(10, 3) = servo_state.pos.segment(9, 3);
        q.segment(13, 3) = servo_state.pos.segment(0, 3);
        q.segment(16, 3) = servo_state.pos.segment(6, 3);
        
        v.segment(0, 3) = odometry.lin_vel;
        v.segment(3, 3) = odometry.ang_vel; 
        v.segment(6, 3) = servo_state.vel.segment(3, 3); // order of legs in pinocchio model is L1, L2, R1, R2
        v.segment(9, 3) = servo_state.vel.segment(9, 3);
        v.segment(12, 3) = servo_state.vel.segment(0, 3);
        v.segment(15, 3) = servo_state.vel.segment(6, 3);
        robot.ComputeForwardKinematics(q, v);
        pin_leg_positions = robot.GetToePositionsInBaseFrame(); // поменять порядок
        leg_positions[R1] = pin_leg_positions[PIN_R1]; leg_positions[L1] = pin_leg_positions[PIN_L1];
        leg_positions[R2] = pin_leg_positions[PIN_R2]; leg_positions[L2] = pin_leg_positions[PIN_L2];

        // get leg Jacobians
        pin_leg_jacobians = robot.GetFootJacobian(q);
        leg_jacobians[R1] = pin_leg_jacobians[PIN_R1]; leg_jacobians[L1] = pin_leg_jacobians[PIN_L1];
        leg_jacobians[R2] = pin_leg_jacobians[PIN_R2]; leg_jacobians[L2] = pin_leg_jacobians[PIN_L2];

        // get rotation matrix
        w_R_b = mors_sys::euler2mat(odometry.orientation_euler(0), 
                                    odometry.orientation_euler(1), 
                                    odometry.orientation_euler(2));

        // get leg odometry
        Eigen::Vector3d dq;
        Eigen::Vector3d ang_vel = imu_data.ang_vel;
        double sum_stance = 0;
        for (int i = 0; i < 4; i++)
        {
            dq << servo_state.vel(3*i), servo_state.vel(3*i+1), servo_state.vel(3*i+2);
            if (contact_states[i] == STANCE)
            {
                foot_vel_global = leg_jacobians[i] * dq;
                Eigen::Vector3d omega_cross_r = ang_vel.cross(leg_positions[i]); // Compute velocity contribution from base angular motion: ω x r
                rel_vel = -(foot_vel_global + omega_cross_r); // Compute linear velocity of the foot relative to base
                sum_stance++;
            }
            else
            {
                rel_vel.setZero();
            }
            foot_vels[i] = rel_vel;
        }
        leg_odom_velocity = (contact_states[0]*foot_vels[0] + contact_states[1]*foot_vels[1] + contact_states[2]*foot_vels[2] + contact_states[3]*foot_vels[3]) / (sum_stance + 1e-5);
        leg_odom_velocity = w_R_b * leg_odom_velocity;

        // Kalman Filtering
        // correct imu acceleration by gravity and yaw rotation
        Eigen::Vector3d imu_accel_corrected = w_R_b * (imu_data.lin_accel - gravity);
        // build measurement vector
        if (lkf_type == 0)
            z_lkf << leg_odom_velocity(0), imu_accel_corrected(0),
                    leg_odom_velocity(1), imu_accel_corrected(1),
                    leg_odom_velocity(2), imu_accel_corrected(2);
        else if (lkf_type == 1)
            z_lkf << odometry.position(0), odometry.lin_vel(0), leg_odom_velocity(0), imu_accel_corrected(0),
                    odometry.position(1), odometry.lin_vel(1), leg_odom_velocity(1), imu_accel_corrected(1),
                    odometry.position(2), odometry.lin_vel(2), leg_odom_velocity(2), imu_accel_corrected(2);
        // lkf step
        x_lkf = lkf.step(u_lkf, z_lkf);


        // fill robot state and leg state
        // robot_state.pos = odometry.position;
        // robot_state.lin_vel = odometry.lin_vel;
        robot_state.pos << x_lkf(0), x_lkf(3), x_lkf(6);
        robot_state.lin_vel << x_lkf(1), x_lkf(4), x_lkf(7);
        robot_state.orientation = odometry.orientation_euler;
        robot_state.ang_vel = odometry.ang_vel;
        leg_state.r1_pos = leg_positions[0];
        leg_state.l1_pos = leg_odom_velocity; //leg_positions[1];
        leg_state.r2_pos = leg_positions[2];
        leg_state.l2_pos = leg_positions[3];
        leg_state.contacts = contact_states;

        // send filtered full robot state
        lcmExch.sendRobotState(robot_state, leg_state);
        // -----------------------------------------------
        t += module_dt;
        // Wait until spinning time
        while(true)
        {
            std::chrono::duration<double, std::milli> elapsed{now() - start};
            if (elapsed >= dt)
            {
                // cout << "Waited for : " << elapsed.count() << " ms" << endl;
                break;
            }
        }
    }

    return 0;
}