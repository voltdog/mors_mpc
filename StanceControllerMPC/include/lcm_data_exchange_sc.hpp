#ifndef _lcm_data_exchange_sc_hpp_
#define _lcm_data_exchange_sc_hpp_

#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <lcm/lcm-cpp.hpp>
 
#include "mors_msgs/grf_cmd_msg.hpp"
#include "mors_msgs/robot_cmd_msg.hpp"
#include "mors_msgs/robot_state_msg.hpp"
#include "mors_msgs/phase_signal_msg.hpp"
#include "mors_msgs/enable_msg.hpp"

#include "system_functions.hpp"
#include <yaml-cpp/yaml.h>
#include "structs.hpp"

using namespace std;
using namespace Eigen;
using namespace YAML;

#define LEG_CONTROL   1
#define SERVO_CONTROL 2

class LCMExchanger
{
    public:
        LCMExchanger();
        ~LCMExchanger();

        void start_exchanger();

        void robotCmdHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::robot_cmd_msg* msg);
        
        void robotStateHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::robot_state_msg* msg);

        void phaseSignalHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::phase_signal_msg* msg);


        void robotCmdThread();
        void robotStateThread();
        void phaseSignalThread();

        RobotData getRobotCmd();
        RobotData getBodyState();
        LegData getLegsState();
        VectorXd getPhaseSignals();

        void sendGrfCmd(LegData &leg_data);

    private:
        string robot_cmd_channel, robot_state_channel, phase_signal_channel;
        string grf_cmd_channel;

        lcm::LCM robot_cmd_subscriber;
        lcm::LCM robot_state_subscriber;
        lcm::LCM phase_signal_subscriber;

        lcm::LCM grf_cmd_publisher;

        mors_msgs::grf_cmd_msg grfCmdMsg;

        unique_ptr<thread> thRobotCmd;
        unique_ptr<thread> thRobotState;
        unique_ptr<thread> thPhaseSignal;

        // VectorXd cmd_vel;
        // VectorXd cmd_pose;
        RobotData robot_state;
        RobotData robot_cmd;
        LegData leg_state;
        VectorXd phase;
};

#endif //_lcm_data_exchange_sc_hpp_