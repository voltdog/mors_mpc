#ifndef _lcm_data_exchange_ml_hpp_
#define _lcm_data_exchange_ml_hpp_

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
#include "mors_msgs/odometry_msg.hpp"
#include "mors_msgs/robot_state_msg.hpp"
#include "mors_msgs/robot_cmd_msg.hpp"
#include "mors_msgs/phase_signal_msg.hpp"
#include "system_functions.hpp"
#include <yaml-cpp/yaml.h>
#include "structs.hpp"

using namespace std;
using namespace Eigen;
using namespace YAML;

#define LEG_CONTROL   1
#define SERVO_CONTROL 2

#define SERVO_FILTERED_CHANNEL "SERVO_STATE_FILTERED"

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
        void servoCmdHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::servo_cmd_msg* msg);
        void enableHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::enable_msg *msg);
        void ctrlTypeHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::std_int* msg);
        void odometryHandler(const lcm::ReceiveBuffer* rbuf, 
                            const std::string& chan, 
                            const mors_msgs::odometry_msg* msg);
        void servoStateFiltHandler(const lcm::ReceiveBuffer* rbuf, 
                            const std::string& chan, 
                            const mors_msgs::servo_state_msg* msg);
        void robotStateHandler(const lcm::ReceiveBuffer* rbuf, 
                            const std::string& chan, 
                            const mors_msgs::robot_state_msg* msg);
        void robotCmdHandler(const lcm::ReceiveBuffer* rbuf, 
                            const std::string& chan, 
                            const mors_msgs::robot_cmd_msg* msg);
        void phaseSigHandler(const lcm::ReceiveBuffer* rbuf, 
                            const std::string& chan, 
                            const mors_msgs::phase_signal_msg* msg);

        void footCmdThread();
        void grfCmdThread();
        void imuThread();
        void servoStateThread();
        void servoCmdThread();
        void enableThread();
        void ctrlTypeThread();
        void odometryThread();
        void servoStateFiltThread();
        void robotStateThread();
        void robotCmdThread();
        void phaseSigThread();

        LegData getLegCmdData();
        ImuData getImuData();
        ServoData getServoStateData();
        ServoData getServoStateFiltData();
        ServoData getServoCmdData();
        Odometry getOdometry();
        RobotData getBodyState();
        LegData getLegState();
        RobotData getRobotCmd();
        void getPhaseSig(Vector4d& phase, Vector4d& phi);

        void getEnableData(bool &leg_controller_en, bool &leg_controller_reset,
                        bool &locomotion_en, bool &locomotion_reset,
                        bool &action_ctr_en, bool &action_ctr_reset);
        int getCtrlTypeData();


    private:
        string foot_cmd_channel, grf_cmd_channel, servo_state_channel, servo_cmd_channel, imu_channel;
        string enable_channel, control_type_channel, odometry_channel, robot_state_channel;
        string robot_cmd_channel, phase_sig_channel;

        lcm::LCM foot_cmd_subscriber;
        lcm::LCM grf_cmd_subscriber;
        lcm::LCM servo_state_subscriber;
        lcm::LCM servo_cmd_subscriber;
        lcm::LCM imu_subscriber;
        lcm::LCM enable_subscriber;
        lcm::LCM controle_type_subscriber;
        lcm::LCM odometry_subscriber;
        lcm::LCM robot_state_subscriber;
        lcm::LCM robot_cmd_subscriber;
        lcm::LCM servo_state_filt_subscriber;
        lcm::LCM phase_sig_subscriber;

        // VectorXd r1_grf, r2_grf, l1_grf, l2_grf;
        // VectorXd r1_pos, ref_l1_pos, r2_pos, l2_pos;
        // VectorXd r1_vel, l1_vel, r2_vel, l2_vel;
        // VectorXd imu_orientation_euler;
        ImuData imu_data;
        // VectorXd servo_pos, servo_vel, servo_torq;
        // VectorXd servo_pos_cmd, servo_vel_cmd, servo_torq_cmd, servo_kp_cmd, servo_kd_cmd;
        // VectorXd kp_vec, kd_vec;
        LegData leg_cmd;
        ServoData servo_state;
        ServoData servo_cmd;
        Odometry odometry;
        ServoData servo_state_filt;
        RobotData body_state;
        RobotData body_cmd;
        LegData leg_state;
        Vector4d phase;
        Vector4d phi;

        unique_ptr<thread> thFootCmd;
        unique_ptr<thread> thGrfCmd;
        unique_ptr<thread> thImu;
        unique_ptr<thread> thServoState;
        unique_ptr<thread> thServoCmd;
        unique_ptr<thread> thEnable;
        unique_ptr<thread> thCtrlType;
        unique_ptr<thread> thOdometry;
        unique_ptr<thread> thServoStateFilt;
        unique_ptr<thread> thRobotState;
        unique_ptr<thread> thRobotCmd;
        unique_ptr<thread> thPhaseSig;
        
        bool leg_controller_enable, leg_controller_reset;
        bool locomotion_enable, locomotion_reset;
        bool action_ctr_enable, action_ctr_reset;
        int control_type;
        double gear_ratio;
};

#endif //_lcm_data_exchange_ml_hpp_