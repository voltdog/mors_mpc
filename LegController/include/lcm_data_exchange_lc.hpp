#ifndef _lcm_data_exchange_lc_hpp_
#define _lcm_data_exchange_lc_hpp_

#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <lcm/lcm-cpp.hpp>
#include "mors_msgs/wbc_cmd_msg.hpp"
#include "mors_msgs/imu_lcm_data.hpp"
#include "mors_msgs/servo_cmd_msg.hpp"
#include "mors_msgs/servo_state_msg.hpp"
#include "mors_msgs/enable_msg.hpp"
#include "mors_msgs/std_int.hpp"
#include "mors_msgs/phase_signal_msg.hpp"
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

        void wbcCmdHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::wbc_cmd_msg* msg);
        void imuHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan, 
                            const mors_msgs::imu_lcm_data* msg);
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
        void imuThread();
        void servoStateThread();
        void enableThread();
        void phaseSigThread();

        void getLegCmdData(LegData &leg_cmd);
        void getImuData(VectorXd &orientation_euler);
        void getServoStateData(VectorXd &position, VectorXd &velocity, VectorXd &torque);
        void getEnableData(bool &en, bool &reset);
        Vector4i getPhaseSignal();

        void sendServoCmd(VectorXd &position, VectorXd &velocity, VectorXd &torque, VectorXd &kp, VectorXd &kd);

    private:
        string wbc_cmd_channel, servo_state_channel, servo_cmd_channel, imu_channel;
        string enable_channel, phase_sig_channel;

        lcm::LCM wbc_cmd_subscriber;
        lcm::LCM servo_state_subscriber;
        lcm::LCM imu_subscriber;
        lcm::LCM enable_subscriber;
        lcm::LCM controle_type_subscriber;
        lcm::LCM phase_sig_subscriber;

        lcm::LCM servo_cmd_publisher;

        mors_msgs::servo_cmd_msg servoCmdMsg;

        Vector3d imu_orientation_euler;
        VectorXd servo_pos, servo_vel, servo_torq;
        LegData leg_cmd;

        unique_ptr<thread> thWbcCmd;
        unique_ptr<thread> thImu;
        unique_ptr<thread> thServoState;
        unique_ptr<thread> thEnable;
        unique_ptr<thread> thCtrlType;
        unique_ptr<thread> thPhaseSig;
        
        bool enable, reset;
        double gear_ratio;

        Vector4i phase_signal;
};

#endif //_lcm_data_exchange_lc_hpp_