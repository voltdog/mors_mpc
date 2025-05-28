#ifndef _lcm_data_exchange_rm_hpp_
#define _lcm_data_exchange_rm_hpp_

#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <lcm/lcm-cpp.hpp>
#include "mors_msgs/leg_cmd_msg.hpp"
#include "mors_msgs/imu_lcm_data.hpp"
#include "mors_msgs/servo_cmd_msg.hpp"
#include "mors_msgs/servo_state_msg.hpp"
#include "mors_msgs/enable_msg.hpp"
#include "system_functions.hpp"
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace Eigen;
using namespace YAML;

// #define GRF_CMD_CHANNEL "LEG_CMD"
// #define SERVO_STATE_CHANNEL "SERVO_STATE"
// #define SERVO_CMD_CHANNEL "SERVO_CMD"
// #define IMU_CHANNEL "IMU_DATA"

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

        void imuThread();
        void servoStateThread();

        void getImuData(VectorXd &orientation_euler);
        void getServoStateData(VectorXd &position, VectorXd &velocity, VectorXd &torque);

        void sendServoCmd(VectorXd &position, VectorXd &velocity, VectorXd &torque, VectorXd &kp, VectorXd &kd);
        void sendEnableData(bool &locomotion_en, bool &locomotion_reset,
                            bool &leg_controller_en, bool &leg_controller_reset);

    private:
        string servo_state_channel, servo_cmd_channel, imu_channel;
        string enable_channel;

        lcm::LCM servo_state_subscriber;
        lcm::LCM imu_subscriber;

        lcm::LCM servo_cmd_publisher;
        lcm::LCM enable_publisher;

        mors_msgs::servo_cmd_msg servoCmdMsg;
        mors_msgs::enable_msg enableMsg;

        VectorXd imu_orientation_euler;
        VectorXd servo_pos, servo_vel, servo_torq;

        unique_ptr<thread> thLegCmd;
        unique_ptr<thread> thImu;
        unique_ptr<thread> thServoState;
        
        bool leg_controller_en, leg_controller_reset;

        double gear_ratio;
        bool imp_mode[4];
        bool grf_mode[4];

};

#endif //_lcm_data_exchange_rm_hpp_