#ifndef _lcm_data_exchange_wbic_hpp_
#define _lcm_data_exchange_wbic_hpp_

#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <lcm/lcm-cpp.hpp>

#include "data_types.hpp"
#include "mors_msgs/enable_msg.hpp"
#include "mors_msgs/phase_signal_msg.hpp"
#include "mors_msgs/wbc_cmd_msg.hpp"
#include "mors_msgs/wbc_state_msg.hpp"
#include "mors_msgs/servo_cmd_msg.hpp"
#include "mors_msgs/robot_state_msg.hpp"
#include "mors_msgs/servo_state_msg.hpp"

#include "system_functions.hpp"
#include "structs.hpp"

using namespace std;
using namespace Eigen;
using namespace YAML;
namespace wbic_types = mors::wbic;

// #define LEG_CONTROL   1
// #define SERVO_CONTROL 2

class LCMExchanger
{
    public:
        using JointVector = wbic_types::Vector12d;
        using ForceVector = wbic_types::Vector12d;
        using PhaseVector = wbic_types::Vector4i;

        LCMExchanger();
        ~LCMExchanger();

        void start_exchanger();

        void wbcCmdHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::wbc_cmd_msg* msg);
        void robotStateHandler(const lcm::ReceiveBuffer* rbuf, 
                            const std::string& chan, 
                            const mors_msgs::robot_state_msg* msg);
        void servoStateHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::servo_state_msg* msg);
        void enableHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::enable_msg *msg);
        void phaseSigHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::phase_signal_msg* msg);

        void wbcCmdThread();
        void robotStateThread();
        void servoStateThread();
        void enableThread();
        void phaseSigThread();

        void getWbcCmdData(RobotData& body_cmd, LegData& leg_cmd);
        void getServoStateData(JointVector& position, JointVector& velocity, JointVector& torque);
        void getEnableData(bool& en, bool& reset);
        PhaseVector getGaitPhaseSignal() const;
        RobotData getBodyState();
        LegData getLegState();

        void sendServoCmd(const JointVector& position,
                          const JointVector& velocity,
                          const JointVector& torque,
                          const JointVector& kp,
                          const JointVector& kd);
        void sendWbcState(const ForceVector& grf);

        bool initialized;

    private:
        string wbc_cmd_channel, servo_cmd_channel, wbc_state_channel;
        string robot_state_channel, servo_state_channel;
        string enable_channel, gait_phase_sig_channel;

        lcm::LCM wbc_cmd_subscriber;
        lcm::LCM robot_state_subscriber;
        lcm::LCM servo_state_subscriber;
        lcm::LCM enable_subscriber;
        lcm::LCM gait_phase_sig_subscriber;

        lcm::LCM servo_cmd_publisher;
        lcm::LCM wbc_state_publisher;

        mors_msgs::servo_cmd_msg servoCmdMsg;
        mors_msgs::wbc_state_msg wbcStateMsg;

        JointVector servo_pos;
        JointVector servo_vel;
        JointVector servo_torq;
        LegData leg_cmd;
        RobotData body_cmd;
        RobotData body_state;
        LegData leg_state;

        unique_ptr<thread> thWbcCmd;
        unique_ptr<thread> thRobotState;
        unique_ptr<thread> thServoState;
        unique_ptr<thread> thEnable;
        unique_ptr<thread> thGaitPhaseSig;
        
        bool enable, reset;
        double gear_ratio;

        PhaseVector phase_signal;

        
};

#endif //_lcm_data_exchange_wbic_hpp_
