#ifndef _lcm_data_exchange_hpp_
#define _lcm_data_exchange_hpp_

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <mutex>
#include <thread>
#include <Eigen/Dense>
#include <lcm/lcm-cpp.hpp>
 
#include "mors_msgs/robot_cmd_msg.hpp"
#include "mors_msgs/mpc_cmd_msg.hpp"
#include "mors_msgs/wbc_cmd_msg.hpp"
#include "mors_msgs/wbc_state_msg.hpp"
#include "mors_msgs/robot_state_msg.hpp"
#include "mors_msgs/gait_params_msg.hpp"
#include "mors_msgs/phase_signal_msg.hpp"
#include "mors_msgs/heightmap_msg.hpp"
#include "mors_msgs/footsteps_msg.hpp"
#include "mors_msgs/dcm_com_trajectory_msg.hpp"
#include "mors_msgs/robot_mode_cmd_msg.hpp"
#include "mors_msgs/robot_mode_result_msg.hpp"
#include "mors_msgs/robot_mode_state_msg.hpp"
 

#include "LocomotionData.hpp"

using namespace std;
using namespace Eigen;

// #define LEG_CONTROL   1
// #define SERVO_CONTROL 2

class LCMExchanger
{
    public:
        explicit LCMExchanger(const std::filesystem::path& config_path);
        ~LCMExchanger();

        void start_exchanger();
        void stop_exchanger();

        void robotCmdHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::robot_cmd_msg* msg);
        
        void robotStateHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::robot_state_msg* msg);

        void gaitParamsHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::gait_params_msg* msg);
        void heightmapHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::heightmap_msg* msg);
        void robotModeHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::robot_mode_cmd_msg* msg);

        std::optional<mors::fsm::RobotStateSnapshot> get_robot_state() const;
        mors::fsm::LocomotionSetpoints get_setpoints() const;
        void get_heightmap_data(VisionBasedMap& vision_map);
        void getHeightmapStatus(bool& heightmap_received) const;
        // Передаёт текущую очередь целиком, чтобы ESTOP мог прервать обычные команды.
        std::vector<mors_msgs::robot_mode_cmd_msg> take_mode_commands();

        void sendWbcCmd(RobotData& robot_data, LegData& leg_data);
        void sendPhaseSig(vector<int>& phase, vector<double>& phi, double t);
        void sendMpcCmd(RobotData& robot_cmd, vector<bool> &active_legs);
        void sendFootstepSequence(
            const mors::wbic::FootSequence<PREVIEW_STRIDES_HORIZON>& foot_plan,
            double t);
        void sendDcmComTrajectory(const std::vector<Eigen::Vector3d>& x_com,
                                  const std::vector<Eigen::Vector3d>& d_x_com,
                                  const std::vector<Eigen::Vector3d>& dd_x_com,
                                  double t);
        void sendWbcState(const JointVector12d& grf);
        void sendRobotModeResult(int8_t command, int8_t result, const string& reason);
        void sendRobotModeState(int8_t current_state,
                                int8_t target_state,
                                int8_t cur_algorithm,
                                int8_t cur_action,
                                int8_t cur_params,
                                bool fault,
                                const string& fault_str);

    private:
        void robotCmdThread();
        void robotStateThread();
        void gaitParamsThread();
        void heightmapThread();
        void robotModeThread();

        string robot_cmd_channel, robot_state_channel, gait_params_channel;
        string heightmap_channel;
        string robot_mode_cmd_channel, robot_mode_result_channel, robot_mode_state_channel;
        string wbc_cmd_channel, wbc_state_channel, phase_signal_channel;
        string mpc_cmd_channel;
        string footstep_sequence_channel;
        string dcm_com_trajectory_channel;

        lcm::LCM robot_cmd_subscriber;
        lcm::LCM robot_state_subscriber;
        lcm::LCM gait_params_subscriber;
        lcm::LCM heightmap_subscriber;
        lcm::LCM robot_mode_subscriber;

        unique_ptr<thread> thRobotCmd;
        unique_ptr<thread> thRobotState;
        unique_ptr<thread> thGaitParams;
        unique_ptr<thread> thHeightmap;
        unique_ptr<thread> thRobotMode;

        lcm::LCM wbc_cmd_publisher;
        lcm::LCM wbc_state_publisher;
        lcm::LCM phase_sig_publisher;
        lcm::LCM robot_ref_publisher;
        lcm::LCM footstep_sequence_publisher;
        lcm::LCM dcm_com_trajectory_publisher;
        lcm::LCM robot_mode_publisher;

        mors_msgs::mpc_cmd_msg mpcCmdMsg;
        mors_msgs::wbc_cmd_msg wbcCmdMsg;
        mors_msgs::wbc_state_msg wbcStateMsg;
        mors_msgs::phase_signal_msg phaseSigMsg;
        mors_msgs::footsteps_msg footstepsMsg;
        mors_msgs::dcm_com_trajectory_msg dcmComTrajectoryMsg;

        std::optional<mors::fsm::RobotStateSnapshot> observation;
        mors::fsm::LocomotionSetpoints setpoints;
        std::vector<mors_msgs::robot_mode_cmd_msg> robot_mode_cmd_queue;
        VisionBasedMap vision_map;
        float heightmap_height_min;
        float heightmap_height_resolution;

        std::atomic<bool> stop_requested_{false};
        std::atomic<bool> heightmap_received_{false};

        mutable std::mutex robot_cmd_mutex;
        mutable std::mutex robot_state_mutex;
        mutable std::mutex gait_params_mutex;
        mutable std::mutex heightmap_mutex;
        std::mutex robot_mode_cmd_mutex;
};

#endif //_lcm_data_exchange_hpp_
