#ifndef _lcm_data_exchange_gs_hpp_
#define _lcm_data_exchange_gs_hpp_

#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <lcm/lcm-cpp.hpp>
 
#include "mors_msgs/robot_cmd_msg.hpp"
#include "mors_msgs/robot_state_msg.hpp"
#include "mors_msgs/phase_signal_msg.hpp"
#include "mors_msgs/gait_params_msg.hpp"

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

        void robotRefHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::robot_cmd_msg* msg);
        
        void robotStateHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::robot_state_msg* msg);

        void gaitParamsHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::gait_params_msg* msg);


        void robotRefThread();
        void robotStateThread();
        void gaitParamsThread();

        RobotData getRobotRef();
        RobotData getBodyState();
        LegData getLegState();
        void get_gait_params(double& t_st, double& t_sw, vector<double>& gait_type, bool& standing, double& stride_height);

        void sendPhaseSig(vector<int>& phase, vector<double>& phi, double t);//, VectorXi &gait_table);
        void sendRobotCmd(RobotData& robot_cmd);

    private:
        string robot_cmd_channel, robot_ref_channel, robot_state_channel, phase_signal_channel;
        string gait_params_channel;

        lcm::LCM robot_ref_subscriber;
        lcm::LCM robot_state_subscriber;
        lcm::LCM gait_params_subscriber;

        lcm::LCM phase_sig_publisher;
        lcm::LCM robot_cmd_publisher;

        mors_msgs::phase_signal_msg phaseSigMsg;
        mors_msgs::robot_cmd_msg robotCmdMsg;

        unique_ptr<thread> thRobotRef;
        unique_ptr<thread> thRobotState;
        unique_ptr<thread> thGaitParams;

        // VectorXd cmd_vel;
        // VectorXd cmd_pose;
        RobotData robot_state;
        // RobotData robot_cmd;
        RobotData robot_ref;
        LegData leg_state;
        // VectorXd phase;
        double t_st;
        double t_sw;
        vector<double> gait_type;
        bool standing;
        double stride_height;
};

#endif //_lcm_data_exchange_gs_hpp_