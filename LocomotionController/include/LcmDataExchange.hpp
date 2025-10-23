#ifndef _lcm_data_exchange_hpp_
#define _lcm_data_exchange_hpp_

#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <lcm/lcm-cpp.hpp>
 
#include "mors_msgs/robot_cmd_msg.hpp"
#include "mors_msgs/robot_state_msg.hpp"
#include "mors_msgs/gait_params_msg.hpp"
#include "mors_msgs/enable_msg.hpp"
#include "mors_msgs/phase_signal_msg.hpp"
 
#include "mors_msgs/foot_cmd_msg.hpp"
#include "mors_msgs/grf_cmd_msg.hpp"

#include "system_functions.hpp"
#include <yaml-cpp/yaml.h>
#include "structs.hpp"

using namespace std;
using namespace Eigen;
using namespace YAML;

// #define LEG_CONTROL   1
// #define SERVO_CONTROL 2

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

        void gaitParamsHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::gait_params_msg* msg);
        void enableHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::enable_msg* msg);

        RobotData getRobotCmd();
        RobotData getBodyState();
        LegData getLegState();
        void get_gait_params(double& t_st, double& t_sw, vector<double>& gait_type, bool& standing, double& stride_height);
        bool get_enable();
        void get_active_legs(vector<bool>& active_legs);
        int get_adaptation_type();

        void sendLegCmd(LegData &leg_data);
        void sendPhaseSig(vector<int>& phase, vector<double>& phi, double t);
        void sendRobotRef(RobotData& robot_cmd, vector<bool> &active_legs, int8_t adaptation_type);

    private:
        void robotCmdThread();
        void robotStateThread();
        void gaitParamsThread();
        void enableThread();

        string robot_cmd_channel, robot_state_channel, gait_params_channel, enable_channel;
        string grf_cmd_channel, foot_cmd_channel, phase_signal_channel;
        string robot_ref_channel;

        lcm::LCM robot_cmd_subscriber;
        lcm::LCM robot_state_subscriber;
        lcm::LCM gait_params_subscriber;
        lcm::LCM enable_subscriber;

        unique_ptr<thread> thRobotCmd;
        unique_ptr<thread> thRobotState;
        unique_ptr<thread> thGaitParams;
        unique_ptr<thread> thEnable;

        lcm::LCM grf_cmd_publisher;
        lcm::LCM foot_cmd_publisher;
        lcm::LCM phase_sig_publisher;
        lcm::LCM robot_ref_publisher;

        mors_msgs::grf_cmd_msg grfCmdMsg;
        mors_msgs::foot_cmd_msg footCmdMsg;
        mors_msgs::phase_signal_msg phaseSigMsg;
        mors_msgs::robot_cmd_msg robotRefMsg;

        // VectorXd cmd_vel;
        // VectorXd cmd_pose;
        RobotData robot_state;
        RobotData robot_cmd;
        LegData leg_state;
        // VectorXd phase;
        double t_st;
        double t_sw;
        vector<double> gait_type;
        bool standing;
        double stride_height;
        vector<bool> active_legs;
        int adaptation_type;

        bool enable;
};

#endif //_lcm_data_exchange_hpp_