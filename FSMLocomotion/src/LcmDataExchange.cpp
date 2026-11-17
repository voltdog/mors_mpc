#include "LcmDataExchange.hpp"

#include "FSM/FSMTypes.hpp"

#include <algorithm>
#include <cstdlib>
#include <Eigen/Geometry>
#include <limits>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace {

std::string get_locomotion_url()
{
    for (const char* name : {"LCM_LOCOMOTION_URL", "LCM_DEFAULT_URL"}) {
        if (const char* value = std::getenv(name); value && *value) {
            return value;
        }
    }
    throw std::runtime_error("LCM_LOCOMOTION_URL or LCM_DEFAULT_URL must be set");
}

constexpr uint16_t kHeightCellValidMask = static_cast<uint16_t>(1u << 15);
constexpr uint16_t kHeightCellClassMask = static_cast<uint16_t>(0x3u << 13);
constexpr uint16_t kHeightCellHeightMask = static_cast<uint16_t>(0x1FFFu);
constexpr int kReceiveTimeoutMs = 50;

} // namespace

LCMExchanger::LCMExchanger(const std::filesystem::path& config_path)
    : robot_cmd_subscriber(get_locomotion_url()),
      robot_state_subscriber(get_locomotion_url()),
      gait_params_subscriber(get_locomotion_url()),
      heightmap_subscriber(get_locomotion_url()),
      robot_mode_subscriber(get_locomotion_url()),
      wbc_cmd_publisher(get_locomotion_url()),
      wbc_state_publisher(get_locomotion_url()),
      phase_sig_publisher(get_locomotion_url()),
      robot_ref_publisher(get_locomotion_url()),
      footstep_sequence_publisher(get_locomotion_url()),
      dcm_com_trajectory_publisher(get_locomotion_url()),
      robot_mode_publisher(get_locomotion_url())
{
    if (!(robot_cmd_subscriber.good() && robot_state_subscriber.good() &&
          gait_params_subscriber.good() && heightmap_subscriber.good() &&
          robot_mode_subscriber.good() && wbc_cmd_publisher.good() &&
          wbc_state_publisher.good() && phase_sig_publisher.good() &&
          robot_ref_publisher.good() && footstep_sequence_publisher.good() &&
          dcm_com_trajectory_publisher.good() && robot_mode_publisher.good())) {
        throw std::runtime_error("LCMExchanger initialization failed");
    }

    const YAML::Node channel_config = YAML::LoadFile((config_path / "channels.yaml").string());
    robot_cmd_channel = channel_config["robot_cmd"].as<string>();
    mpc_cmd_channel = channel_config["mpc_cmd"].as<string>();
    wbc_cmd_channel = channel_config["wbc_cmd"].as<string>();
    wbc_state_channel = channel_config["wbc_state"].as<string>();
    robot_state_channel = channel_config["robot_state"].as<string>();
    gait_params_channel = channel_config["gait_params"].as<string>();
    phase_signal_channel = channel_config["gait_phase"].as<string>();
    footstep_sequence_channel = channel_config["footstep_sequence"].as<string>();
    dcm_com_trajectory_channel = channel_config["dcm_com_trajectory"].as<string>();
    heightmap_channel = channel_config["heightmap"].as<string>();
    robot_mode_cmd_channel = channel_config["robot_mode_cmd"].as<string>();
    robot_mode_result_channel = channel_config["robot_mode_result"].as<string>();
    robot_mode_state_channel = channel_config["robot_mode_state"].as<string>();

    heightmap_height_min = -2.0f;
    heightmap_height_resolution = 0.005f;

    try {
        const auto heightmap_config_path = config_path / "heightmap_builder.yaml";
        const YAML::Node heightmap_config = YAML::LoadFile(heightmap_config_path.string());
        if (heightmap_config["map"]) {
            const YAML::Node map_config = heightmap_config["map"];
            if (map_config["height_min"]) {
                heightmap_height_min = map_config["height_min"].as<float>();
            }
            if (map_config["height_resolution"]) {
                heightmap_height_resolution = map_config["height_resolution"].as<float>();
            }
        }
    } catch (const std::exception&) {
        // Keep default decode parameters.
    }

    for (int i = 0; i < 3; ++i) {
        wbcStateMsg.r1_grf[i] = 0.0f;
        wbcStateMsg.l1_grf[i] = 0.0f;
        wbcStateMsg.r2_grf[i] = 0.0f;
        wbcStateMsg.l2_grf[i] = 0.0f;
    }

    footstepsMsg.t = 0.0;
    for (int stride = 0; stride < PREVIEW_STRIDES_HORIZON; ++stride) {
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            for (int axis = 0; axis < 3; ++axis) {
                footstepsMsg.positions[stride][leg][axis] = 0.0;
            }
        }
    }

    dcmComTrajectoryMsg.t = 0.0;
    dcmComTrajectoryMsg.x_com_count = 0;
    dcmComTrajectoryMsg.d_x_com_count = 0;
    dcmComTrajectoryMsg.dd_x_com_count = 0;
}

void LCMExchanger::start_exchanger()
{
    if (thRobotCmd) {
        return;
    }
    const auto subscribe = [this](lcm::LCM& subscriber, const std::string& channel, auto handler) {
        if (!subscriber.subscribe(channel, handler, this)) {
            throw std::runtime_error("Failed to subscribe to " + channel);
        }
    };
    subscribe(robot_cmd_subscriber, robot_cmd_channel, &LCMExchanger::robotCmdHandler);
    subscribe(robot_state_subscriber, robot_state_channel, &LCMExchanger::robotStateHandler);
    subscribe(gait_params_subscriber, gait_params_channel, &LCMExchanger::gaitParamsHandler);
    subscribe(heightmap_subscriber, heightmap_channel, &LCMExchanger::heightmapHandler);
    subscribe(robot_mode_subscriber, robot_mode_cmd_channel, &LCMExchanger::robotModeHandler);

    thRobotCmd = make_unique<thread>(&LCMExchanger::robotCmdThread, this);
    thRobotState = make_unique<thread>(&LCMExchanger::robotStateThread, this);
    thGaitParams = make_unique<thread>(&LCMExchanger::gaitParamsThread, this);
    thHeightmap = make_unique<thread>(&LCMExchanger::heightmapThread, this);
    thRobotMode = make_unique<thread>(&LCMExchanger::robotModeThread, this);
}

void LCMExchanger::robotCmdHandler(const lcm::ReceiveBuffer*,
                                   const std::string&,
                                   const mors_msgs::robot_cmd_msg* msg)
{
    lock_guard<mutex> lock(robot_cmd_mutex);
    auto& robot_cmd = setpoints.robot_cmd;
    for (int i = 0; i < 3; ++i) {
        robot_cmd.pos(i) = msg->cmd_pose[i];
        robot_cmd.orientation(i) = msg->cmd_pose[i + 3];
        robot_cmd.lin_vel(i) = msg->cmd_vel[i];
        robot_cmd.ang_vel(i) = msg->cmd_vel[i + 3];
    }
    setpoints.active_legs.assign(msg->active_legs, msg->active_legs + NUM_LEGS);
    setpoints.adaptation_type = static_cast<std::uint8_t>(msg->adaptation_type);
    setpoints.robot_cmd_received_at_s = mors::fsm::monotonic_seconds();
}

void LCMExchanger::robotStateHandler(const lcm::ReceiveBuffer*,
                                     const std::string&,
                                     const mors_msgs::robot_state_msg* msg)
{
    mors::fsm::RobotStateSnapshot state;
    state.received_at_s = mors::fsm::monotonic_seconds();
    auto& robot_state = state.body;
    auto& leg_state = state.legs;
    auto& servo_state = state.joints;
    for (int i = 0; i < 3; ++i) {
        leg_state.r1_pos(i) = msg->legs.r1_pos[i];
        leg_state.l1_pos(i) = msg->legs.l1_pos[i];
        leg_state.r2_pos(i) = msg->legs.r2_pos[i];
        leg_state.l2_pos(i) = msg->legs.l2_pos[i];

        leg_state.r1_vel(i) = msg->legs.r1_vel[i];
        leg_state.l1_vel(i) = msg->legs.l1_vel[i];
        leg_state.r2_vel(i) = msg->legs.r2_vel[i];
        leg_state.l2_vel(i) = msg->legs.l2_vel[i];

        leg_state.r1_grf(i) = msg->legs.r1_grf[i];
        leg_state.l1_grf(i) = msg->legs.l1_grf[i];
        leg_state.r2_grf(i) = msg->legs.r2_grf[i];
        leg_state.l2_grf(i) = msg->legs.l2_grf[i];

        leg_state.contacts[i] = msg->legs.contact_states[i];

        robot_state.pos(i) = msg->body.position[i];
        robot_state.orientation(i) = msg->body.orientation_euler[i];
        robot_state.orientation_quaternion(i) = msg->body.orientation_quaternion[i];
        robot_state.lin_vel(i) = msg->body.lin_vel[i];
        robot_state.ang_vel(i) = msg->body.ang_vel[i];
    }

    leg_state.contacts[3] = msg->legs.contact_states[3];
    robot_state.orientation_quaternion(3) = msg->body.orientation_quaternion[3];
    for (Eigen::Index i = 0; i < servo_state.position.size(); ++i) {
        servo_state.position(i) = msg->joints.position[i];
        servo_state.velocity(i) = msg->joints.velocity[i];
        servo_state.torque(i) = msg->joints.torque[i];
    }
    std::copy_n(msg->readiness, state.readiness.size(), state.readiness.begin());

    lock_guard<mutex> lock(robot_state_mutex);
    observation = std::move(state);
}

void LCMExchanger::gaitParamsHandler(const lcm::ReceiveBuffer*,
                                     const std::string&,
                                     const mors_msgs::gait_params_msg* msg)
{
    lock_guard<mutex> lock(gait_params_mutex);
    setpoints.t_sw = msg->t_sw;
    setpoints.t_st = msg->t_st;
    setpoints.stride_height = msg->stride_height;
    setpoints.gait_type.assign(msg->gait_type, msg->gait_type + NUM_LEGS);
}

void LCMExchanger::heightmapHandler(const lcm::ReceiveBuffer*,
                                    const std::string&,
                                    const mors_msgs::heightmap_msg* msg)
{
    lock_guard<mutex> lock(heightmap_mutex);

    if (msg->data_size != mors::wbic::kHeightmapCellCount ||
        msg->data.size() < static_cast<size_t>(mors::wbic::kHeightmapCellCount)) {
        return;
    }

    vision_map.origin_x = msg->origin_x;
    vision_map.origin_y = msg->origin_y;
    vision_map.yaw = msg->yaw;

    for (int row = 0; row < mors::wbic::kHeightmapCellsY; ++row) {
        for (int col = 0; col < mors::wbic::kHeightmapCellsX; ++col) {
            const int idx = row * mors::wbic::kHeightmapCellsX + col;
            const uint16_t packed = static_cast<uint16_t>(msg->data[static_cast<size_t>(idx)]);
            const bool valid = (packed & kHeightCellValidMask) != 0u;
            const uint8_t traversability =
                static_cast<uint8_t>((packed & kHeightCellClassMask) >> 13);

            vision_map.traversability(row, col) = traversability;

            if (valid) {
                const uint16_t height_q = static_cast<uint16_t>(packed & kHeightCellHeightMask);
                vision_map.heightmap(row, col) = heightmap_height_min +
                    static_cast<float>(height_q) * heightmap_height_resolution;
            } else {
                vision_map.heightmap(row, col) = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }

    heightmap_received_.store(true, std::memory_order_release);
}

void LCMExchanger::robotModeHandler(const lcm::ReceiveBuffer*,
                                    const std::string&,
                                    const mors_msgs::robot_mode_cmd_msg* msg)
{
    lock_guard<mutex> lock(robot_mode_cmd_mutex);
    robot_mode_cmd_queue.push_back(*msg);
}

void LCMExchanger::robotCmdThread()
{
    while (!stop_requested_.load(std::memory_order_relaxed)) {
        robot_cmd_subscriber.handleTimeout(kReceiveTimeoutMs);
    }
}

void LCMExchanger::robotStateThread()
{
    while (!stop_requested_.load(std::memory_order_relaxed)) {
        robot_state_subscriber.handleTimeout(kReceiveTimeoutMs);
    }
}

void LCMExchanger::gaitParamsThread()
{
    while (!stop_requested_.load(std::memory_order_relaxed)) {
        gait_params_subscriber.handleTimeout(kReceiveTimeoutMs);
    }
}

void LCMExchanger::heightmapThread()
{
    while (!stop_requested_.load(std::memory_order_relaxed)) {
        heightmap_subscriber.handleTimeout(kReceiveTimeoutMs);
    }
}

void LCMExchanger::robotModeThread()
{
    while (!stop_requested_.load(std::memory_order_relaxed)) {
        robot_mode_subscriber.handleTimeout(kReceiveTimeoutMs);
    }
}

std::optional<mors::fsm::RobotStateSnapshot> LCMExchanger::get_robot_state() const
{
    lock_guard<mutex> lock(robot_state_mutex);
    return observation;
}

mors::fsm::LocomotionSetpoints LCMExchanger::get_setpoints() const
{
    std::scoped_lock lock(robot_cmd_mutex, gait_params_mutex);
    return setpoints;
}

std::vector<mors_msgs::robot_mode_cmd_msg> LCMExchanger::take_mode_commands()
{
    std::vector<mors_msgs::robot_mode_cmd_msg> commands;
    lock_guard<mutex> lock(robot_mode_cmd_mutex);
    commands.swap(robot_mode_cmd_queue);
    return commands;
}

void LCMExchanger::getHeightmapStatus(bool& heightmap_received) const
{
    heightmap_received = heightmap_received_.load(std::memory_order_acquire);
}

void LCMExchanger::get_heightmap_data(VisionBasedMap& vision_map)
{
    lock_guard<mutex> lock(heightmap_mutex);
    vision_map = this->vision_map;
}

void LCMExchanger::sendWbcCmd(RobotData& robot_data, LegData& leg_data)
{
    const Eigen::Quaterniond q_ref =
        Eigen::AngleAxisd(robot_data.orientation[Z], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(robot_data.orientation[Y], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(robot_data.orientation[X], Eigen::Vector3d::UnitX());

    wbcCmdMsg.body.orientation_quaternion[0] = q_ref.x();
    wbcCmdMsg.body.orientation_quaternion[1] = q_ref.y();
    wbcCmdMsg.body.orientation_quaternion[2] = q_ref.z();
    wbcCmdMsg.body.orientation_quaternion[3] = q_ref.w();

    for (int i = 0; i < 3; ++i) {
        wbcCmdMsg.body.position[i] = robot_data.pos(i);
        wbcCmdMsg.body.orientation_euler[i] = robot_data.orientation(i);
        wbcCmdMsg.body.lin_vel[i] = robot_data.lin_vel(i);
        wbcCmdMsg.body.ang_vel[i] = robot_data.ang_vel(i);

        wbcCmdMsg.legs.r1_pos[i] = leg_data.r1_pos(i);
        wbcCmdMsg.legs.l1_pos[i] = leg_data.l1_pos(i);
        wbcCmdMsg.legs.r2_pos[i] = leg_data.r2_pos(i);
        wbcCmdMsg.legs.l2_pos[i] = leg_data.l2_pos(i);

        wbcCmdMsg.legs.r1_vel[i] = leg_data.r1_vel(i);
        wbcCmdMsg.legs.l1_vel[i] = leg_data.l1_vel(i);
        wbcCmdMsg.legs.r2_vel[i] = leg_data.r2_vel(i);
        wbcCmdMsg.legs.l2_vel[i] = leg_data.l2_vel(i);

        wbcCmdMsg.legs.r1_grf[i] = leg_data.r1_grf(i);
        wbcCmdMsg.legs.l1_grf[i] = leg_data.l1_grf(i);
        wbcCmdMsg.legs.r2_grf[i] = leg_data.r2_grf(i);
        wbcCmdMsg.legs.l2_grf[i] = leg_data.l2_grf(i);
    }

    wbc_cmd_publisher.publish(wbc_cmd_channel, &wbcCmdMsg);
}

void LCMExchanger::sendPhaseSig(vector<int>& phase, vector<double>& phi, double t)
{
    for (int i = 0; i < NUM_LEGS; ++i) {
        phaseSigMsg.phase[i] = phase[i];
        phaseSigMsg.phi[i] = phi[i];
    }
    phaseSigMsg.t = t;
    phase_sig_publisher.publish(phase_signal_channel, &phaseSigMsg);
}

void LCMExchanger::sendMpcCmd(RobotData& robot_cmd, vector<bool>& active_legs)
{
    for (int i = 0; i < 3; ++i) {
        mpcCmdMsg.cmd_vel[i] = robot_cmd.lin_vel[i];
        mpcCmdMsg.cmd_vel[i + 3] = robot_cmd.ang_vel[i];
        mpcCmdMsg.cmd_pose[i] = robot_cmd.pos[i];
        mpcCmdMsg.cmd_pose[i + 3] = robot_cmd.orientation[i];
        mpcCmdMsg.active_legs[i] = active_legs[i];
    }
    mpcCmdMsg.active_legs[3] = active_legs[3];

    robot_ref_publisher.publish(mpc_cmd_channel, &mpcCmdMsg);
}

void LCMExchanger::sendFootstepSequence(
    const mors::wbic::FootSequence<PREVIEW_STRIDES_HORIZON>& foot_plan,
    double t)
{
    footstepsMsg.t = t;

    for (int stride = 0; stride < PREVIEW_STRIDES_HORIZON; ++stride) {
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            for (int axis = 0; axis < 3; ++axis) {
                footstepsMsg.positions[stride][leg][axis] =
                    foot_plan[stride][leg][axis];
            }
        }
    }

    footstep_sequence_publisher.publish(footstep_sequence_channel, &footstepsMsg);
}

void LCMExchanger::sendDcmComTrajectory(const std::vector<Eigen::Vector3d>& x_com,
                                        const std::vector<Eigen::Vector3d>& d_x_com,
                                        const std::vector<Eigen::Vector3d>& dd_x_com,
                                        double t)
{
    auto fillTrajectory = [](const std::vector<Eigen::Vector3d>& source,
                             std::vector<std::vector<double>>& destination) {
        destination.resize(source.size());
        for (std::size_t point = 0; point < source.size(); ++point) {
            destination[point].resize(3);
            for (int axis = 0; axis < 3; ++axis) {
                destination[point][axis] = source[point](axis);
            }
        }
    };

    dcmComTrajectoryMsg.t = t;
    dcmComTrajectoryMsg.x_com_count = static_cast<int32_t>(x_com.size());
    dcmComTrajectoryMsg.d_x_com_count = static_cast<int32_t>(d_x_com.size());
    dcmComTrajectoryMsg.dd_x_com_count = static_cast<int32_t>(dd_x_com.size());

    fillTrajectory(x_com, dcmComTrajectoryMsg.x_com);
    fillTrajectory(d_x_com, dcmComTrajectoryMsg.d_x_com);
    fillTrajectory(dd_x_com, dcmComTrajectoryMsg.dd_x_com);

    dcm_com_trajectory_publisher.publish(dcm_com_trajectory_channel, &dcmComTrajectoryMsg);
}

void LCMExchanger::sendWbcState(const JointVector12d& grf)
{
    for (int i = 0; i < 3; ++i) {
        wbcStateMsg.r1_grf[i] = grf(PIN_R1 * 3 + i);
        wbcStateMsg.l1_grf[i] = grf(PIN_L1 * 3 + i);
        wbcStateMsg.r2_grf[i] = grf(PIN_R2 * 3 + i);
        wbcStateMsg.l2_grf[i] = grf(PIN_L2 * 3 + i);
    }

    wbc_state_publisher.publish(wbc_state_channel, &wbcStateMsg);
}

void LCMExchanger::sendRobotModeResult(int8_t command, int8_t result, const string& reason)
{
    mors_msgs::robot_mode_result_msg msg{};
    msg.command = command;
    msg.result = result;
    msg.reason = reason;
    robot_mode_publisher.publish(robot_mode_result_channel, &msg);
}

void LCMExchanger::sendRobotModeState(int8_t current_state,
                                      int8_t target_state,
                                      int8_t cur_algorithm,
                                      int8_t cur_action,
                                      int8_t cur_params,
                                      bool fault,
                                      const string& fault_str)
{
    mors_msgs::robot_mode_state_msg msg{};
    msg.current_state = current_state;
    msg.target_state = target_state;
    msg.cur_algorithm = cur_algorithm;
    msg.cur_action = cur_action;
    msg.cur_params = cur_params;
    msg.fault = fault;
    msg.fault_str = fault_str;
    robot_mode_publisher.publish(robot_mode_state_channel, &msg);
}

LCMExchanger::~LCMExchanger()
{
    stop_exchanger();
}

void LCMExchanger::stop_exchanger()
{
    stop_requested_.store(true, std::memory_order_relaxed);

    for (auto* receiver : {thRobotCmd.get(), thRobotState.get(), thGaitParams.get(),
                           thHeightmap.get(), thRobotMode.get()}) {
        if (receiver && receiver->joinable()) {
            receiver->join();
        }
    }
}
