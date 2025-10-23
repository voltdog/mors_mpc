#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <lcm/lcm-cpp.hpp>
#include <yaml-cpp/yaml.h>
#include "structs.hpp"
#include "system_functions.hpp"
#include "mors_msgs/odometry_msg.hpp"
#include "example-utils.hpp"
#include <Eigen/Dense>
#include <cmath>

#define X 0
#define Y 1
#define Z 2

#define OFFSET_X (0.26045)
#define OFFSET_Y (0.01675)
#define OFFSET_Z (0.01)
#define OFFSET_ROLL (0.0)
#define OFFSET_PITCH (M_PI_2)
#define OFFSET_YAW (M_PI_2)

using namespace YAML;
using namespace std;

double r11, r12, r13, r21, r22, r23, r31, r32, r33;

MatrixXd euler2mat(double roll, double pitch, double yaw)
{
    r11 = cos(yaw)*cos(pitch);
    r12 = cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll);
    r13 = cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll);
    r21 = sin(yaw)*cos(pitch);
    r22 = sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll);
    r23 = sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll);
    r31 = -sin(pitch);
    r32 = cos(pitch)*sin(roll);
    r33 = cos(pitch)*cos(roll);

    MatrixXd R(3,3);
    R << r11, r12, r13,
         r21, r22, r23,
         r31, r32, r33;
    return R;
}

void quaternionToEuler(float qx, float qy, float qz, float qw, float& roll, float& pitch, float& yaw) 
{
    // pitch = atan2(2.0 * (qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz);
    // roll = asin(-2.0 * (qx*qz - qw*qy));
    // yaw = atan2(2.0 * (qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz);
    pitch = -asin(2.0 * (qx*qz - qw*qy));
    roll = atan2(2.0 * (qw*qx + qy*qz), qw*qw - qx*qx - qy*qy + qz*qz);
    yaw = atan2(2.0 * (qw*qz + qx*qy), qw*qw + qx*qx - qy*qy - qz*qz);


    // double sinr_cosp = 2 * (qw * qx + qy * qz);
    // double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    // pitch = atan2(sinr_cosp, cosr_cosp);


    // double sinp = 2 * (qw * qy - qz * qx);
    // if (std::abs(sinp) >= 1) {
    //     roll = std::copysign(M_PI / 2, sinp); 
    // } else {
    //     roll = asin(sinp);
    // }

}

int main(int argc, char * argv[]) try
{
    cout << "[RealsenseCamera]: starting..." << endl;
    float roll, pitch, yaw;

    // Get config address
    string config_address = mors_sys::GetEnv("CONFIGPATH");//cwd;
    config_address += "/channels.yaml";
    // Read config
    YAML::Node channel_config = YAML::LoadFile(config_address);//"/home/user/mors_mpc_control/config/channels.yaml");
    string odometry_channel = channel_config["odometry"].as<string>();

    // LCM init
    lcm::LCM odometry_publisher;
    mors_msgs::odometry_msg odometryMsg;

    // T265 init
    std::string serial;
    if (!device_with_streams({ RS2_STREAM_POSE}, serial))
        return EXIT_SUCCESS;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    if (!serial.empty())
        cfg.enable_device(serial);
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Start pipeline with chosen configuration
    pipe.start(cfg);

    // define camera translation
    // MatrixXd R_x(3,3), R_y(3,3), R_z(3,3), R_cam_offset(3,3);
    // // MatrixXd T_cam_body(4,4), T_body_glob(4,4);
    // MatrixXd R_cam_glob(3,3), R_body_glob(3,3);
    // VectorXd P_cam_glob(3);
    // VectorXd P_body_glob(3);
    // VectorXd P_body_cam(3);
    // R_x = euler2mat(OFFSET_ROLL, 0.0, 0.0);
    // R_y = euler2mat(0.0, OFFSET_PITCH, 0.0);
    // R_z = euler2mat(0.0, 0.0, OFFSET_YAW);
    // R_cam_offset = R_x * R_y * R_z;
    // P_body_cam << OFFSET_X, OFFSET_Y, OFFSET_Z;
    // VectorXd rpy_cam(3);
    // VectorXd drpy_cam(3), drpy_cam_glob(3);
    // VectorXd rpy_cam_glob(3);
    // VectorXd dP_cam_glob(3), dP_body_glob(3);
    // VectorXd offset(3);

    // bool first = true;

    cout << "[RealsenseCamera]: started" << endl;

    // Main loop
    while (true)
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        // Get a frame from the pose stream
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();


        // Print the x, y, z values of the translation, relative to initial position
        // std::cout << "\r" << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " <<
        //     pose_data.translation.y << " " << pose_data.translation.z << " (meters)";
        
        // convert quaternion to rpy
        quaternionToEuler(pose_data.rotation.z,
                                    pose_data.rotation.x,
                                    pose_data.rotation.y,
                                    pose_data.rotation.w,
                                    roll, pitch, yaw);
        // get robot positions and velocities in global coordinates
        // rpy_cam << roll, pitch, yaw;
        // rpy_cam_glob =  R_cam_offset * rpy_cam;

        // R_cam_glob = euler2mat(rpy_cam(0), rpy_cam(1), rpy_cam(2));

        // P_cam_glob << pose_data.translation.x, pose_data.translation.y, pose_data.translation.z;
        // P_cam_glob = R_cam_offset * P_cam_glob;
        
        // P_body_glob = R_cam_glob * P_body_cam + P_cam_glob;

        // dP_cam_glob << pose_data.velocity.x, pose_data.velocity.y, pose_data.velocity.z;
        // dP_body_glob = R_cam_offset * dP_cam_glob;

        // drpy_cam << pose_data.angular_velocity.x, pose_data.angular_velocity.y, pose_data.angular_velocity.z;
        // drpy_cam_glob =  R_cam_offset * drpy_cam;

        // if (first)
        // {
        //     offset = P_body_glob;
        //     first = false;
        // }

        // publish to lcm
        odometryMsg.position[X] = pose_data.translation.z;//P_body_glob(X) - offset(X);
        odometryMsg.position[Y] = pose_data.translation.x;//P_body_glob(Y) - offset(Y);
        odometryMsg.position[Z] = pose_data.translation.y;//P_body_glob(Z) - offset(Z);
        odometryMsg.orientation[X] = roll;//rpy_cam(X);//pose_data.rotation.x;//
        odometryMsg.orientation[Y] = pitch;//rpy_cam(Y);//pose_data.rotation.y;//
        odometryMsg.orientation[Z] = yaw;//rpy_cam(Z);//pose_data.rotation.z;//
        odometryMsg.lin_vel[X] = pose_data.velocity.z;//dP_body_glob(X);
        odometryMsg.lin_vel[Y] = pose_data.velocity.x;//dP_body_glob(Y);
        odometryMsg.lin_vel[Z] = pose_data.velocity.y;//dP_body_glob(Z);
        odometryMsg.ang_vel[X] = pose_data.angular_velocity.z;//drpy_cam_glob(X);
        odometryMsg.ang_vel[Y] = pose_data.angular_velocity.x;//drpy_cam_glob(Y);
        odometryMsg.ang_vel[Z] = pose_data.angular_velocity.y;//drpy_cam_glob(Z);
        odometry_publisher.publish(odometry_channel, &odometryMsg);
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "[RealsenseCamera]: error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}