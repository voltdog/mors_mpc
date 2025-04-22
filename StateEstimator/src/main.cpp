#include "lcm_data_exchange_se.hpp"
#include "low_pass_filtering.hpp"
#include "sensor_fusion.hpp"
#include "leg_state.hpp"

using namespace std;
using namespace Eigen;
using namespace YAML;
using namespace std::chrono;

#define CAMERA_OFFSET_X (0.26045)
#define CAMERA_OFFSET_Y (0.01675)
#define CAMERA_OFFSET_Z (-0.01)

// current time
auto now() 
{
  return std::chrono::steady_clock::now(); 
}

void init_vectors(RobotData &robot_state, LegData &leg_state)
{
    robot_state.pos.resize(3);
    robot_state.pos.setZero();
    robot_state.lin_vel.resize(3);
    robot_state.lin_vel.setZero();
    robot_state.orientation.resize(3);
    robot_state.orientation.setZero();
    robot_state.ang_vel.resize(3);
    robot_state.ang_vel.setZero();

    leg_state.contacts.resize(4);
    leg_state.contacts = {false, false, false, false};
    leg_state.r1_pos.resize(3);
    leg_state.r1_pos.setZero();
    leg_state.l1_pos.resize(3);
    leg_state.l1_pos.setZero();
    leg_state.r2_pos.resize(3);
    leg_state.r2_pos.setZero();
    leg_state.l2_pos.resize(3);
    leg_state.l2_pos.setZero();

    leg_state.r1_vel.resize(3);
    leg_state.r1_vel.setZero();
    leg_state.l1_vel.resize(3);
    leg_state.l1_vel.setZero();
    leg_state.r2_vel.resize(3);
    leg_state.r2_vel.setZero();
    leg_state.l2_vel.resize(3);
    leg_state.l2_vel.setZero();

    leg_state.r1_acc.resize(3);
    leg_state.r1_acc.setZero();
    leg_state.l1_acc.resize(3);
    leg_state.l1_acc.setZero();
    leg_state.r2_acc.resize(3);
    leg_state.r2_acc.setZero();
    leg_state.l2_acc.resize(3);
    leg_state.l2_acc.setZero();

    leg_state.r1_grf.resize(3);
    leg_state.r1_grf.setZero();
    leg_state.l1_grf.resize(3);
    leg_state.l1_grf.setZero();
    leg_state.r2_grf.resize(3);
    leg_state.r2_grf.setZero();
    leg_state.l2_grf.resize(3);
    leg_state.l2_grf.setZero();
}

int main() 
{
    cout << "StateEstimator starting..." << endl;
    // load config
    string config_address = mors_sys::GetEnv("CONFIGPATH");
    string robot_config_address = config_address + "/robot_config.yaml";

    YAML::Node robot_config = YAML::LoadFile(robot_config_address);
    RobotPhysicalParams robot;
    robot.bx = robot_config["bx"].as<double>(); 
    robot.by = robot_config["by"].as<double>(); 
    robot.m1 = robot_config["m1"].as<double>(); 
    robot.m2 = robot_config["m2"].as<double>(); 
    robot.m3 = robot_config["m3"].as<double>(); 
    robot.l1 = robot_config["l1"].as<double>(); 
    robot.l2 = robot_config["l2"].as<double>(); 
    robot.l3 = robot_config["l3"].as<double>(); 
    robot.d1 = robot_config["d1"].as<double>(); 
    robot.d2 = robot_config["d2"].as<double>(); 
    robot.d3 = robot_config["d3"].as<double>(); 
    robot.l_cz_2 = robot_config["Pc2"][2].as<double>(); 
    robot.l_cx_3 = robot_config["Pc3"][0].as<double>(); 
    robot.g = robot_config["g"].as<double>(); 
    robot.kt = robot_config["kt"].as<double>(); 
    robot.gear_ratio = robot_config["gear_ratio"].as<double>(); 

    string dt_config_address = config_address + "/timesteps.yaml";
    YAML::Node dt_config = YAML::LoadFile(dt_config_address);
    double module_dt = dt_config["state_estimator_dt"].as<double>(); 

    string state_estimator_config_address = config_address + "/state_estimator.yaml";
    YAML::Node state_estimator_config = YAML::LoadFile(state_estimator_config_address);
    double contact_threshold = state_estimator_config["contact_threshold"].as<double>(); 

    auto dt = std::chrono::duration<double>(module_dt);//1ms;
    // cout << module_dt << endl;

    // define lcm exchanger
    LCMExchanger lcmExch;
    lcmExch.start_exchanger();
    std::this_thread::sleep_for(5ms);

    // define input data
    ImuData imu_data;
    ServoData servo_state;
    Odometry odometry;
    // defin output data
    RobotData robot_state;
    LegData leg_state;

    // defin sensor fusion
    SensorFusion sensor_fusion;
    RobotData data_fusioned;
    
    // define other data
    LegState leg_state_estimator(robot);
    VectorXd p(3);
    // R1
    // double lamb = 100.0;
    p << 0.032, -0.03, 0.001;
    double lamb = 100.0;
    // p << -0.032, 0.01, -0.001;
    // p.setZero();
    leg_state_estimator.set_grf_observer_params(lamb, module_dt, p);
    leg_state_estimator.set_contact_threshold(contact_threshold);

    LowPassFiltering lpf(module_dt);
    init_vectors(robot_state, leg_state);

    MatrixXd R_body(3,3);
    VectorXd rpy_rate(3);
    MatrixXd rpy_rate_cross(3,3);

    VectorXd P_body_cam(3);
    P_body_cam << CAMERA_OFFSET_X, CAMERA_OFFSET_Y, CAMERA_OFFSET_Z;

    VectorXd body_pos(3);
    VectorXd offset(3);
    offset.setZero();
    bool first = true;

    // yaw variables
    // double yaw_tmp = 0.0;
    // double pre_yaw = 0.0;
    // double yaw = 0.0;
    
    cout << "StateEstimator started" << endl;

    double t = 0.0;
    while(true)
    {
        // Calculating current time
        const auto start{ now() };

        // Put your code here
        // -----------------------------------------------
        imu_data = lcmExch.getImuData();
        servo_state = lcmExch.getServoStateData();
        servo_state.torq *= (0.73/10.0);
        odometry = lcmExch.getOdometry();

        // do smth
        // orientation data fusion
        data_fusioned.orientation = sensor_fusion.update_orientation(imu_data.orientation_euler, odometry.orientation); //odometry.orientation;//
        // fused orientation filtering
        robot_state.orientation = lpf.update_orientation(data_fusioned.orientation); //odometry.orientation;//
        // convert odometry.ang_vel to rpy_rate
        R_body = mors_sys::euler2mat(robot_state.orientation(0), robot_state.orientation(1), robot_state.orientation(2));
        rpy_rate = imu_data.ang_vel;//odometry.ang_vel; //R_body * 
        // angular velocity fusion
        data_fusioned.ang_vel = odometry.ang_vel; //imu_data.ang_vel; //sensor_fusion.update_rpy_rate(imu_data.ang_vel, rpy_rate); //rpy_rate;// 
        // fused rpy rate filtering
        robot_state.ang_vel = lpf.update_rpy_rate(data_fusioned.ang_vel);
        // convert position from camera frame to body frame
        body_pos = odometry.position + R_body * P_body_cam  - offset;

        if (first)
        {
            offset = body_pos;
            first = false;
        }
        // position and linear velocity low pass filtering
        robot_state.pos = lpf.update_position(body_pos);
        
        
        // rpy_rate_cross << 0,                     -robot_state.ang_vel(Z), robot_state.ang_vel(Y),
        //                  robot_state.ang_vel(Z), 0,                     -robot_state.ang_vel(X),
        //                 -robot_state.ang_vel(Y), robot_state.ang_vel(X), 0;
        rpy_rate_cross << 0,         -rpy_rate(Z),  rpy_rate(Y),
                         rpy_rate(Z), 0,           -rpy_rate(X),
                        -rpy_rate(Y), rpy_rate(X),  0;
        robot_state.lin_vel =  (lpf.update_lin_vel(odometry.lin_vel) + (rpy_rate_cross) * R_body * P_body_cam);
        // cout << robot_state.ang_vel.cross(P_body_cam) << endl;

        // get leg states
        leg_state = leg_state_estimator.get_leg_state(servo_state.pos, servo_state.vel, servo_state.torq);

        
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