#ifndef _lcm_data_exchange_se_hpp_
#define _lcm_data_exchange_se_hpp_

#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <lcm/lcm-cpp.hpp>
#include "mors_msgs/imu_lcm_data.hpp"
#include "mors_msgs/servo_state_msg.hpp"
#include "mors_msgs/odometry_msg.hpp"
#include "mors_msgs/robot_state_msg.hpp"
#include "system_functions.hpp"
#include <yaml-cpp/yaml.h>
#include "structs.hpp"

#define X 0
#define Y 1
#define Z 2

#define SERVO_FILTERED_CHANNEL "SERVO_STATE_FILTERED"

using namespace std;
using namespace Eigen;
using namespace YAML;

class LCMExchanger
{
    public:
        LCMExchanger();
        ~LCMExchanger();

        void start_exchanger();

        void imuHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan, 
                            const mors_msgs::imu_lcm_data* msg);
        void servoStateHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::servo_state_msg* msg);
        void odometryHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::odometry_msg* msg);                            

        void imuThread();
        void servoStateThread();
        void odometryThread();

        ImuData getImuData();
        ServoData getServoStateData();
        Odometry getOdometry();

        void sendRobotState(RobotData robot_state, LegData leg_state);
        void sendServoFiltered(ServoData servo_filtered);

    private:
        string servo_state_channel, imu_channel, odometry_channel;
        string robot_state_channel;

        lcm::LCM servo_state_subscriber;
        lcm::LCM imu_subscriber;
        lcm::LCM odometry_subscriber;
        lcm::LCM robot_state_publisher;
        lcm::LCM servo_filtered_publisher;

        ImuData imu_data;
        ServoData servo_state;
        Odometry odometry;

        unique_ptr<thread> thImu;
        unique_ptr<thread> thServoState;
        unique_ptr<thread> thOdometry;

        mors_msgs::robot_state_msg rs_msg;
        mors_msgs::servo_state_msg servo_filtered_msg;
};

#endif //_lcm_data_exchange_se_hpp_