// leg_state.hpp pulls in pinocchio (via Robot.hpp) before the X/Y/Z macros are
// defined, so it must be included first — those macros otherwise clash with
// pinocchio's templated identifiers (e.g. the Inertia variable `Y`).
#include "leg_state.hpp"
#include "lcm_data_exchange_se.hpp"
#include "low_pass_filtering.hpp"
#include "sensor_fusion.hpp"

using namespace std;
using namespace Eigen;
using namespace YAML;
using namespace std::chrono;

// #define CAMERA_OFFSET_X (0.26045)
// #define CAMERA_OFFSET_Y (0.01675)
// #define CAMERA_OFFSET_Z (-0.01)

// current time
auto now() 
{
  return std::chrono::steady_clock::now(); 
}

int main() 
{
    cout << "[StateEstimator]: starting..." << endl;
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
    double camera_offset_x = state_estimator_config["camera_offset_x"].as<double>(); 
    double camera_offset_y = state_estimator_config["camera_offset_y"].as<double>(); 
    double camera_offset_z = state_estimator_config["camera_offset_z"].as<double>(); 

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
    servo_state = lcmExch.getServoStateData();
    // cout << servo_state.pos << endl;

    // p << 0.04, -0.01, 0.001;
    p << 0.032, -0.01, 0.001; // for m leg configuration
    double lamb = 100.0; //100.0;
    // p << -0.032, 0.01, -0.001; // for x leg configuration
    // p.setZero();
    leg_state_estimator.set_grf_observer_params(lamb, module_dt, p, servo_state.pos);
    leg_state_estimator.set_contact_threshold(contact_threshold);

    LowPassFiltering lpf(module_dt);
    // init_vectors(robot_state, leg_state);
    leg_state.contacts.resize(4);
    leg_state.contacts = {false, false, false, false};

    Matrix3d R_body;
    Vector3d ang_vel_body, ang_vel_world;
    Matrix3d ang_vel_world_cross;

    Vector3d P_body_cam;
    P_body_cam << camera_offset_x, camera_offset_y, camera_offset_z;

    Vector3d body_pos;
    Vector3d pos_offset;
    pos_offset.setZero();
    double yaw_offset;
    yaw_offset = 0;
    bool first_pos = true;
    bool first_yaw = true;

    Matrix3d R_z;

    // double cos_yaw, sin_yaw;
    std::vector<bool> contact_states(4, false);

    // yaw variables
    // double yaw_tmp = 0.0;
    // double pre_yaw = 0.0;
    // double yaw = 0.0;
    
    cout << "[StateEstimator]: started" << endl;

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
        // contact_states = lcmExch.getContactData();
        // std::fill(contact_states.begin(), contact_states.end(), true);

        // ------------------------
        //        ORIENTATION 
        // ------------------------

        // orientation data fusion
        data_fusioned.orientation = sensor_fusion.update_orientation(imu_data.orientation_euler, imu_data.orientation_euler); 
        // fused orientation filtering
        robot_state.orientation_quaternion = imu_data.orientation_quaternion;
        robot_state.orientation = lpf.update_orientation(data_fusioned.orientation); 
        if (first_yaw)
        {
            yaw_offset = data_fusioned.orientation(2);
            first_yaw = false;
        }
        robot_state.orientation(2) -= yaw_offset;

        // ------------------------
        //    ANGULAR VELOCITY
        // ------------------------
        
        // convert odometry.ang_vel to rpy_rate
        R_body = mors_sys::euler2mat(robot_state.orientation(0), robot_state.orientation(1), robot_state.orientation(2));
        ang_vel_body = imu_data.ang_vel;//odometry.ang_vel; //R_body * 

        // fused rpy rate filtering
        robot_state.ang_vel = lpf.update_rpy_rate(ang_vel_body);//ang_vel_world);

        // ------------------------
        //         POSITION
        // ------------------------
        // convert position from camera frame to body frame
        body_pos = odometry.position - pos_offset + R_body * P_body_cam;

        if (first_pos)
        {
            pos_offset = body_pos - Eigen::Vector3d(0.0, 0.0, 0.038);// 0.044);
            first_pos = false;
        }
        // position and linear velocity low pass filtering
        robot_state.pos = lpf.update_position(body_pos);
        
        // ------------------------
        //     LINEAR VELOCITY
        // ------------------------
        // Express the angular velocity in the world frame so it matches the
        // lever arm (R_body * P_body_cam), which is also in the world frame,
        // when transporting the camera velocity to the body origin:
        // v_body = v_cam + omega_world x (R_body * P_body_cam)
        ang_vel_world = R_body * ang_vel_body;
        ang_vel_world_cross <<   0,                -ang_vel_world(Z),  ang_vel_world(Y),
                                 ang_vel_world(Z),  0,                -ang_vel_world(X),
                                -ang_vel_world(Y),  ang_vel_world(X),  0;
        robot_state.lin_vel =  lpf.update_lin_vel(odometry.lin_vel) + (ang_vel_world_cross) * R_body *  P_body_cam;// - (rpy_rate_cross) * body_pos;//

        // ------------------------
        //        LEG STATES
        // ------------------------

        // get leg states
        leg_state = leg_state_estimator.get_leg_state(robot_state, servo_state.pos, servo_state.vel, servo_state.torq);
        // leg_state.contacts = contact_states;

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