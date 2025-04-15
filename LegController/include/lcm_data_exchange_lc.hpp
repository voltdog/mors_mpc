#ifndef _lcm_data_exchange_lc_hpp_
#define _lcm_data_exchange_lc_hpp_

#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <lcm/lcm-cpp.hpp>
#include "mors_msgs/foot_cmd_msg.hpp"
#include "mors_msgs/grf_cmd_msg.hpp"
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

#define LEG_CONTROL   1
#define SERVO_CONTROL 2

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

        void footCmdHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::foot_cmd_msg* msg);
        void grfCmdHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::grf_cmd_msg* msg);
        void imuHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan, 
                            const mors_msgs::imu_lcm_data* msg);
        void servoStateHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::servo_state_msg* msg);
        void enableHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::enable_msg *msg);
        void ctrlTypeHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::std_int* msg);
        void phaseSigHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::phase_signal_msg* msg);


        void footCmdThread();
        void grfCmdThread();
        void imuThread();
        void servoStateThread();
        void enableThread();
        void ctrlTypeThread();
        void phaseSigThread();

        void getLegCmdData(LegData &leg_cmd);
        void getImuData(VectorXd &orientation_euler);
        void getServoStateData(VectorXd &position, VectorXd &velocity, VectorXd &torque);
        void getEnableData(bool &en, bool &reset);
        int getCtrlTypeData();
        Vector4d getPhaseSignal();

        void sendServoCmd(VectorXd &position, VectorXd &velocity, VectorXd &torque, VectorXd &kp, VectorXd &kd);

    private:
        string foot_cmd_channel, grf_cmd_channel, servo_state_channel, servo_cmd_channel, imu_channel;
        string enable_channel, control_type_channel, phase_sig_channel;

        lcm::LCM foot_cmd_subscriber;
        lcm::LCM grf_cmd_subscriber;
        lcm::LCM servo_state_subscriber;
        lcm::LCM imu_subscriber;
        lcm::LCM enable_subscriber;
        lcm::LCM controle_type_subscriber;
        lcm::LCM phase_sig_subscriber;

        lcm::LCM servo_cmd_publisher;

        mors_msgs::servo_cmd_msg servoCmdMsg;

        // VectorXd r1_grf, r2_grf, l1_grf, l2_grf;
        // VectorXd r1_pos, ref_l1_pos, r2_pos, l2_pos;
        // VectorXd r1_vel, l1_vel, r2_vel, l2_vel;
        VectorXd imu_orientation_euler;
        VectorXd servo_pos, servo_vel, servo_torq;
        // VectorXd kp_vec, kd_vec;
        LegData leg_cmd;

        unique_ptr<thread> thFootCmd;
        unique_ptr<thread> thGrfCmd;
        unique_ptr<thread> thImu;
        unique_ptr<thread> thServoState;
        unique_ptr<thread> thEnable;
        unique_ptr<thread> thCtrlType;
        unique_ptr<thread> thPhaseSig;
        
        bool enable, reset;
        int control_type;
        double gear_ratio;

        Vector4d phase_signal;
};

#endif //_lcm_data_exchange_lc_hpp_