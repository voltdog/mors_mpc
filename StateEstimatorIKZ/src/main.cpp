// leg_state.hpp includes pinocchio through Robot.hpp, so keep it before any
// legacy X/Y/Z macro definitions that may appear in copied headers.
#include "leg_state.hpp"
#include "z_pos_estimator.hpp"

#include "HeightMapBuilder.hpp"
#include "gm_force_observer.hpp"
#include "low_pass_filtering.hpp"
#include "sensor_fusion.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <librealsense2/rs.hpp>
#include <lcm/lcm-cpp.hpp>
#include <yaml-cpp/yaml.h>

#include "mors_msgs/contact_sensor_msg.hpp"
#include "mors_msgs/imu_lcm_data.hpp"
#include "mors_msgs/robot_state_msg.hpp"
#include "mors_msgs/servo_state_msg.hpp"
#include "system_functions.hpp"

namespace
{

constexpr int kStateHistoryCapacity = 2048;
constexpr double kTorqueScale = 0.73 / 10.0;
constexpr const char* kRobotStateCheckChannel = "ROBOT_STATE_CHECK";
std::atomic_bool g_running{true};

int64_t NowNs()
{
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

std::string GetRequiredEnv(const char* name)
{
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0')
    {
        throw std::runtime_error(std::string("[StateEstimatorHMB] ") + name + " must be set.");
    }
    return value;
}

std::string ConfigPath(const std::string& config_dir, const std::string& file_name)
{
    const std::filesystem::path path = std::filesystem::path(config_dir) / file_name;
    if (!std::filesystem::exists(path))
    {
        throw std::runtime_error("[StateEstimatorHMB] config file not found: " + path.string());
    }
    return path.string();
}

Eigen::Matrix3d RotationFromEuler(double roll, double pitch, double yaw)
{
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    Eigen::Matrix3d r;
    r << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
         sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
         -sp,     cp * sr,                cp * cr;
    return r;
}

void T265QuaternionToEuler(
    float qx,
    float qy,
    float qz,
    float qw,
    float& roll,
    float& pitch,
    float& yaw)
{
    pitch = -std::asin(2.0f * (qx * qz - qw * qy));
    roll = std::atan2(
        2.0f * (qw * qx + qy * qz),
        qw * qw - qx * qx - qy * qy + qz * qz);
    yaw = std::atan2(
        2.0f * (qw * qz + qx * qy),
        qw * qw + qx * qx - qy * qy - qz * qz);
}

RobotPhysicalParams LoadRobotPhysicalParams(const std::string& path)
{
    const YAML::Node robot_config = YAML::LoadFile(path);
    RobotPhysicalParams robot;
    robot.bx = robot_config["bx"].as<double>();
    robot.by = robot_config["by"].as<double>();
    robot.m1 = robot_config["m1"].as<double>();
    robot.m2 = robot_config["m2"].as<double>();
    robot.m3 = robot_config["m3"].as<double>();
    robot.l1 = robot_config["l1"].as<double>();
    robot.l2 = robot_config["l2"].as<double>();
    robot.l3 = robot_config["l3"].as<double>();
    robot.d1 = robot_config["d1"].as<double>();
    robot.d2 = robot_config["d2"].as<double>();
    robot.d3 = robot_config["d3"].as<double>();
    robot.l_cz_2 = robot_config["Pc2"][2].as<double>();
    robot.l_cx_3 = robot_config["Pc3"][0].as<double>();
    robot.g = robot_config["g"].as<double>();
    robot.kt = robot_config["kt"].as<double>();
    robot.gear_ratio = robot_config["gear_ratio"].as<double>();
    return robot;
}

struct ChannelsConfig
{
    std::string imu_data;
    std::string servo_state;
    std::string contact_state;
    std::string robot_state;
};

struct StateEstimatorConfig
{
    double dt{0.002};
    double contact_threshold{1.0};
    Eigen::Vector3d camera_offset{0.0, 0.0, 0.0};
};

struct T265Config
{
    std::string serial;
};

struct D435iConfig
{
    std::string serial;
    int width{424};
    int height{240};
    int fps{30};
    double publish_fps{30.0};
    double min_depth_m{0.105};
    double max_depth_m{3.0};
    bool verbose{false};
};

struct DepthProcessingConfig
{
    double min_depth_m{0.105};
    double max_depth_m{3.0};
    bool enable_downsampling{true};
    int downsample_factor{2};
    bool remove_outliers{true};
    int outlier_window_radius{2};
    double outlier_threshold_m{0.12};
    int min_neighbors_for_outlier{3};
    double max_sync_dt_sec{0.05};
    bool require_recent_robot_state{true};
};

struct SharedInputs
{
    ImuData imu;
    ServoData servo;
    Odometry odometry;
    ZPosEstimator::Contacts contacts{};
    int64_t odometry_timestamp_ns{0};
    bool has_imu{false};
    bool has_servo{false};
    bool has_odometry{false};
    bool has_contacts{false};
};

struct DepthFrameData
{
    int64_t timestamp_ns{0};
    uint64_t frame_index{0};
    std::vector<float> cam_x;
    std::vector<float> cam_y;
    std::vector<float> cam_z;
};

ChannelsConfig LoadChannels(const std::string& path)
{
    const YAML::Node root = YAML::LoadFile(path);
    return ChannelsConfig{
        root["imu_data"].as<std::string>(),
        root["servo_state"].as<std::string>(),
        root["contact_state"].as<std::string>(),
        root["robot_state"].as<std::string>()};
}

StateEstimatorConfig LoadStateEstimatorConfig(
    const std::string& timesteps_path,
    const std::string& state_estimator_path)
{
    const YAML::Node timesteps = YAML::LoadFile(timesteps_path);
    const YAML::Node se = YAML::LoadFile(state_estimator_path);

    StateEstimatorConfig config;
    config.dt = timesteps["state_estimator_dt"].as<double>();
    config.contact_threshold = se["contact_threshold"].as<double>();
    config.camera_offset <<
        se["camera_offset_x"].as<double>(),
        se["camera_offset_y"].as<double>(),
        se["camera_offset_z"].as<double>();
    return config;
}

T265Config LoadT265Config(const std::string& path)
{
    const YAML::Node root = YAML::LoadFile(path);
    T265Config config;
    if (root["stream"] && root["stream"]["serial"])
    {
        config.serial = root["stream"]["serial"].as<std::string>();
    }
    return config;
}

D435iConfig LoadD435iConfig(const std::string& path)
{
    const YAML::Node root = YAML::LoadFile(path);
    D435iConfig config;
    if (const YAML::Node stream = root["stream"])
    {
        if (stream["serial"]) config.serial = stream["serial"].as<std::string>();
        if (stream["width"]) config.width = stream["width"].as<int>();
        if (stream["height"]) config.height = stream["height"].as<int>();
        if (stream["fps"]) config.fps = stream["fps"].as<int>();
        if (stream["publish_fps"]) config.publish_fps = stream["publish_fps"].as<double>();
    }
    if (const YAML::Node depth = root["depth"])
    {
        if (depth["min_depth_m"]) config.min_depth_m = depth["min_depth_m"].as<double>();
        if (depth["max_depth_m"]) config.max_depth_m = depth["max_depth_m"].as<double>();
    }
    if (const YAML::Node runtime = root["runtime"])
    {
        if (runtime["verbose"]) config.verbose = runtime["verbose"].as<bool>();
    }
    return config;
}

DepthProcessingConfig LoadDepthProcessingConfig(const std::string& path)
{
    const YAML::Node root = YAML::LoadFile(path);
    DepthProcessingConfig config;
    if (const YAML::Node depth = root["depth"])
    {
        if (depth["min_depth"]) config.min_depth_m = depth["min_depth"].as<double>();
        if (depth["max_depth"]) config.max_depth_m = depth["max_depth"].as<double>();
        if (depth["enable_downsampling"]) config.enable_downsampling = depth["enable_downsampling"].as<bool>();
        if (depth["downsample_factor"]) config.downsample_factor = std::max(1, depth["downsample_factor"].as<int>());
        if (depth["remove_outliers"]) config.remove_outliers = depth["remove_outliers"].as<bool>();
        if (depth["outlier_window_radius"]) config.outlier_window_radius = std::max(1, depth["outlier_window_radius"].as<int>());
        if (depth["outlier_threshold_m"]) config.outlier_threshold_m = std::max(0.0, depth["outlier_threshold_m"].as<double>());
        if (depth["min_neighbors_for_outlier"]) config.min_neighbors_for_outlier = std::max(1, depth["min_neighbors_for_outlier"].as<int>());
    }
    if (const YAML::Node sync = root["sync"])
    {
        if (sync["max_sync_dt_sec"]) config.max_sync_dt_sec = std::max(0.0, sync["max_sync_dt_sec"].as<double>());
        if (sync["require_recent_robot_state"]) config.require_recent_robot_state = sync["require_recent_robot_state"].as<bool>();
    }
    return config;
}

bool IsOutlierDepth(
    const std::vector<float>& depth_m,
    int width,
    int height,
    int u,
    int v,
    float value_m,
    const DepthProcessingConfig& config)
{
    if (!config.remove_outliers)
    {
        return false;
    }

    const int r = config.outlier_window_radius;
    std::vector<float> neighbors;
    neighbors.reserve(static_cast<size_t>((2 * r + 1) * (2 * r + 1)));

    for (int dv = -r; dv <= r; ++dv)
    {
        const int vv = v + dv;
        if (vv < 0 || vv >= height)
        {
            continue;
        }
        for (int du = -r; du <= r; ++du)
        {
            const int uu = u + du;
            if (uu < 0 || uu >= width || (du == 0 && dv == 0))
            {
                continue;
            }

            const float neighbor = depth_m[static_cast<size_t>(vv) * static_cast<size_t>(width) +
                                           static_cast<size_t>(uu)];
            if (std::isfinite(neighbor) &&
                neighbor >= config.min_depth_m &&
                neighbor <= config.max_depth_m)
            {
                neighbors.push_back(neighbor);
            }
        }
    }

    if (static_cast<int>(neighbors.size()) < config.min_neighbors_for_outlier)
    {
        return false;
    }

    auto middle = neighbors.begin() + static_cast<std::ptrdiff_t>(neighbors.size() / 2u);
    std::nth_element(neighbors.begin(), middle, neighbors.end());
    return std::fabs(value_m - *middle) > config.outlier_threshold_m;
}

class StateEstimatorHMBApp
{
public:
    StateEstimatorHMBApp()
        : config_dir_(GetRequiredEnv("CONFIGPATH")),
          control_lcm_url_(GetRequiredEnv("LCM_CONTROL_URL")),
          servo_lcm_url_(GetRequiredEnv("LCM_SERVO_URL")),
          vision_lcm_url_(GetRequiredEnv("LCM_VISION_URL")),
          channels_(LoadChannels(ConfigPath(config_dir_, "channels.yaml"))),
          se_config_(LoadStateEstimatorConfig(
              ConfigPath(config_dir_, "timesteps.yaml"),
              ConfigPath(config_dir_, "state_estimator.yaml"))),
          robot_params_(LoadRobotPhysicalParams(ConfigPath(config_dir_, "robot_config.yaml"))),
          t265_config_(LoadT265Config(ConfigPath(config_dir_, "realsense_camera.yaml"))),
          d435i_config_(LoadD435iConfig(ConfigPath(config_dir_, "realsense_camera_d435i.yaml"))),
          depth_config_(LoadDepthProcessingConfig(ConfigPath(config_dir_, "heightmap_builder.yaml"))),
          imu_lcm_(std::make_unique<lcm::LCM>(control_lcm_url_)),
          servo_lcm_(std::make_unique<lcm::LCM>(servo_lcm_url_)),
          contact_lcm_(std::make_unique<lcm::LCM>(control_lcm_url_)),
          robot_state_publisher_(std::make_unique<lcm::LCM>(control_lcm_url_)),
          servo_filtered_publisher_(std::make_unique<lcm::LCM>(servo_lcm_url_)),
          heightmap_builder_(std::make_unique<hmb::HeightMapBuilderNode>(
              ConfigPath(config_dir_, "heightmap_builder.yaml"),
              false))
    {
        if (!imu_lcm_->good() || !servo_lcm_->good() || !contact_lcm_->good() ||
            !robot_state_publisher_->good() || !servo_filtered_publisher_->good())
        {
            throw std::runtime_error("[StateEstimatorHMB] failed to initialize one or more LCM endpoints.");
        }

        shared_inputs_.servo.pos = Eigen::VectorXd::Zero(12);
        shared_inputs_.servo.vel = Eigen::VectorXd::Zero(12);
        shared_inputs_.servo.torq = Eigen::VectorXd::Zero(12);

        std::cout << "[StateEstimatorHMB] loaded configs from " << config_dir_ << "\n"
                  << "  state dt: " << se_config_.dt << " sec\n"
                  << "  D435i: " << d435i_config_.width << "x" << d435i_config_.height
                  << "@" << d435i_config_.fps << "\n"
                  << "  depth/state sync max dt: " << depth_config_.max_sync_dt_sec << " sec"
                  << std::endl;
    }

    void Run()
    {
        running_.store(true);
        threads_.emplace_back(&StateEstimatorHMBApp::ImuLcmThread, this);
        threads_.emplace_back(&StateEstimatorHMBApp::ServoLcmThread, this);
        threads_.emplace_back(&StateEstimatorHMBApp::ContactLcmThread, this);
        threads_.emplace_back(&StateEstimatorHMBApp::PoseCameraThread, this);
        threads_.emplace_back(&StateEstimatorHMBApp::StateEstimatorThread, this);
        threads_.emplace_back(&StateEstimatorHMBApp::DepthCameraThread, this);
        threads_.emplace_back(&StateEstimatorHMBApp::HeightMapThread, this);

        while (running_.load() && g_running.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        Stop();
    }

    void Stop()
    {
        const bool was_running = running_.exchange(false);
        depth_cv_.notify_all();
        if (!was_running)
        {
            return;
        }
        for (std::thread& thread : threads_)
        {
            if (thread.joinable())
            {
                thread.join();
            }
        }
    }

private:
    void ImuHandler(
        const lcm::ReceiveBuffer*,
        const std::string&,
        const mors_msgs::imu_lcm_data* msg)
    {
        if (msg == nullptr)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(inputs_mutex_);
        for (int i = 0; i < 3; ++i)
        {
            shared_inputs_.imu.orientation_euler(i) = msg->orientation_euler[i];
            shared_inputs_.imu.orientation_quaternion(i) = msg->orientation_quaternion[i];
            shared_inputs_.imu.ang_vel(i) = msg->angular_velocity[i];
            shared_inputs_.imu.lin_accel(i) = msg->linear_acceleration[i];
        }
        shared_inputs_.imu.orientation_quaternion(3) = msg->orientation_quaternion[3];
        shared_inputs_.has_imu = true;
    }

    void ServoHandler(
        const lcm::ReceiveBuffer*,
        const std::string&,
        const mors_msgs::servo_state_msg* msg)
    {
        if (msg == nullptr)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(inputs_mutex_);
        for (int i = 0; i < 12; ++i)
        {
            shared_inputs_.servo.pos(i) = msg->position[i];
            shared_inputs_.servo.vel(i) = msg->velocity[i];
            shared_inputs_.servo.torq(i) = -msg->torque[i];
        }
        shared_inputs_.has_servo = true;
    }

    void ContactHandler(
        const lcm::ReceiveBuffer*,
        const std::string&,
        const mors_msgs::contact_sensor_msg* msg)
    {
        if (msg == nullptr)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(inputs_mutex_);
        for (int i = 0; i < 4; ++i)
        {
            shared_inputs_.contacts[i] = msg->contact_states[i];
        }
        shared_inputs_.has_contacts = true;
    }

    void ImuLcmThread()
    {
        imu_lcm_->subscribe(channels_.imu_data, &StateEstimatorHMBApp::ImuHandler, this);
        while (running_.load() && g_running.load())
        {
            if (imu_lcm_->handleTimeout(20) < 0)
            {
                running_.store(false);
            }
        }
    }

    void ServoLcmThread()
    {
        servo_lcm_->subscribe(channels_.servo_state, &StateEstimatorHMBApp::ServoHandler, this);
        while (running_.load() && g_running.load())
        {
            if (servo_lcm_->handleTimeout(20) < 0)
            {
                running_.store(false);
            }
        }
    }

    void ContactLcmThread()
    {
        contact_lcm_->subscribe(channels_.contact_state, &StateEstimatorHMBApp::ContactHandler, this);
        while (running_.load() && g_running.load())
        {
            if (contact_lcm_->handleTimeout(20) < 0)
            {
                running_.store(false);
            }
        }
    }

    void PoseCameraThread()
    {
        std::unique_ptr<rs2::pipeline> pipe;
        try
        {
            {
                std::lock_guard<std::mutex> lock(realsense_pipeline_mutex_);
                pipe = std::make_unique<rs2::pipeline>();
                rs2::config cfg;
                if (!t265_config_.serial.empty())
                {
                    cfg.enable_device(t265_config_.serial);
                }
                cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
                pipe->start(cfg);
            }

            std::cout << "[StateEstimatorHMB] T265 pose stream started"
                      << (t265_config_.serial.empty() ? "" : " serial=" + t265_config_.serial)
                      << std::endl;

            while (running_.load() && g_running.load())
            {
                const rs2::frameset frames = pipe->wait_for_frames();
                const rs2::frame pose_frame = frames.first_or_default(RS2_STREAM_POSE);
                if (!pose_frame)
                {
                    continue;
                }

                const auto pose = pose_frame.as<rs2::pose_frame>().get_pose_data();
                float roll = 0.0f;
                float pitch = 0.0f;
                float yaw = 0.0f;
                // cout << pose.tracker_confidence << endl;
                T265QuaternionToEuler(
                    pose.rotation.z,
                    pose.rotation.x,
                    pose.rotation.y,
                    pose.rotation.w,
                    roll,
                    pitch,
                    yaw);

                Odometry odometry;
                odometry.position(0) = pose.translation.z;
                odometry.position(1) = pose.translation.x;
                odometry.position(2) = pose.translation.y;
                odometry.orientation_euler(0) = roll;
                odometry.orientation_euler(1) = pitch;
                odometry.orientation_euler(2) = yaw;
                odometry.orientation_quaternion(0) = pose.rotation.z;
                odometry.orientation_quaternion(1) = pose.rotation.x;
                odometry.orientation_quaternion(2) = pose.rotation.y;
                odometry.orientation_quaternion(3) = pose.rotation.w;
                odometry.lin_vel(0) = pose.velocity.z;
                odometry.lin_vel(1) = pose.velocity.x;
                odometry.lin_vel(2) = pose.velocity.y;
                odometry.ang_vel(0) = pose.angular_velocity.z;
                odometry.ang_vel(1) = pose.angular_velocity.x;
                odometry.ang_vel(2) = pose.angular_velocity.y;

                std::lock_guard<std::mutex> lock(inputs_mutex_);
                shared_inputs_.odometry = odometry;
                shared_inputs_.odometry_timestamp_ns = NowNs();
                shared_inputs_.has_odometry = true;
            }
        }
        catch (const rs2::error& e)
        {
            std::cerr << "[StateEstimatorHMB] T265 RealSense error in "
                      << e.get_failed_function() << "(" << e.get_failed_args()
                      << "): " << e.what() << std::endl;
            running_.store(false);
        }
        catch (const std::exception& e)
        {
            std::cerr << "[StateEstimatorHMB] T265 fatal: " << e.what() << std::endl;
            running_.store(false);
        }
        StopRealSensePipeline(pipe);
    }

    void StateEstimatorThread()
    {
        try
        {
            SensorFusion sensor_fusion;
            RobotData robot_state;
            LegData leg_state;
            LegState leg_state_estimator(robot_params_);
            ZPosEstimator z_pos_estimator;

            Eigen::VectorXd p(3);
            p << 0.032, -0.01, 0.001;
            Eigen::VectorXd initial_theta = Eigen::VectorXd::Zero(12);
            leg_state_estimator.set_grf_observer_params(100.0, se_config_.dt, p, initial_theta);
            leg_state_estimator.set_contact_threshold(se_config_.contact_threshold);

            bool first_pos = true;
            bool first_yaw = true;
            Eigen::Vector3d pos_offset = Eigen::Vector3d::Zero();
            double yaw_offset = 0.0;

            const auto period = std::chrono::duration<double>(se_config_.dt);
            auto next_tick = std::chrono::steady_clock::now();

            std::cout << "[StateEstimatorHMB] state estimator loop started" << std::endl;

            while (running_.load() && g_running.load())
            {
                next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

                SharedInputs inputs;
                {
                    std::lock_guard<std::mutex> lock(inputs_mutex_);
                    inputs = shared_inputs_;
                }

                if (!inputs.has_imu || !inputs.has_servo ||
                    !inputs.has_odometry || !inputs.has_contacts)
                {
                    std::this_thread::sleep_until(next_tick);
                    continue;
                }

                ServoData servo_state = inputs.servo;
                servo_state.torq *= kTorqueScale;

                robot_state.orientation_quaternion = inputs.imu.orientation_quaternion;
                robot_state.orientation = sensor_fusion.update_orientation(
                    inputs.imu.orientation_euler,
                    inputs.imu.orientation_euler);
                if (first_yaw)
                {
                    yaw_offset = robot_state.orientation(2);
                    first_yaw = false;
                }
                robot_state.orientation(2) -= yaw_offset;

                const Eigen::Matrix3d r_body = RotationFromEuler(
                    robot_state.orientation(0),
                    robot_state.orientation(1),
                    robot_state.orientation(2));
                robot_state.ang_vel = inputs.imu.ang_vel;

                const Eigen::Vector3d raw_body_pos =
                    inputs.odometry.position + r_body * se_config_.camera_offset;
                if (first_pos)
                {
                    pos_offset = raw_body_pos - Eigen::Vector3d(0.0, 0.0, 0.038);
                    first_pos = false;
                }
                robot_state.pos = raw_body_pos - pos_offset;

                const Eigen::Vector3d ang_vel_world = r_body * robot_state.ang_vel;
                Eigen::Matrix3d ang_vel_world_cross;
                ang_vel_world_cross << 0.0, -ang_vel_world(2), ang_vel_world(1),
                                       ang_vel_world(2), 0.0, -ang_vel_world(0),
                                      -ang_vel_world(1), ang_vel_world(0), 0.0;
                robot_state.lin_vel =
                    inputs.odometry.lin_vel + ang_vel_world_cross * r_body * se_config_.camera_offset;

                const std::optional<ZPosEstimate> z_estimate = z_pos_estimator.Update(
                    servo_state.pos,
                    r_body,
                    inputs.contacts);
                RobotData robot_state_check = robot_state;
                if (z_estimate.has_value())
                {
                    robot_state_check.pos.z() = z_estimate->position_z;
                }

                leg_state = leg_state_estimator.get_leg_state(
                    robot_state,
                    servo_state.pos,
                    servo_state.vel,
                    servo_state.torq);
                for (std::size_t leg = 0; leg < inputs.contacts.size(); ++leg)
                {
                    leg_state.contacts[leg] = inputs.contacts[leg];
                }

                const int64_t timestamp_ns = NowNs();
                PublishRobotState(
                    channels_.robot_state,
                    timestamp_ns,
                    robot_state,
                    leg_state);
                PublishRobotState(
                    kRobotStateCheckChannel,
                    timestamp_ns,
                    robot_state_check,
                    leg_state);
                PushRobotStateSnapshot(timestamp_ns, robot_state);

                std::this_thread::sleep_until(next_tick);
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "[StateEstimatorHMB] state estimator fatal: " << e.what() << std::endl;
            running_.store(false);
        }
    }

    void DepthCameraThread()
    {
        std::unique_ptr<rs2::pipeline> pipe;
        try
        {
            rs2::pipeline_profile profile;
            {
                std::lock_guard<std::mutex> lock(realsense_pipeline_mutex_);
                pipe = std::make_unique<rs2::pipeline>();
                rs2::config cfg;
                if (!d435i_config_.serial.empty())
                {
                    cfg.enable_device(d435i_config_.serial);
                }
                cfg.enable_stream(
                    RS2_STREAM_DEPTH,
                    d435i_config_.width,
                    d435i_config_.height,
                    RS2_FORMAT_Z16,
                    d435i_config_.fps);
                profile = pipe->start(cfg);
            }

            double depth_scale = 0.001;
            for (const rs2::sensor& sensor : profile.get_device().query_sensors())
            {
                const rs2::depth_sensor depth_sensor = sensor.as<rs2::depth_sensor>();
                if (depth_sensor)
                {
                    depth_scale = depth_sensor.get_depth_scale();
                    break;
                }
            }

            rs2::pointcloud pointcloud;
            uint64_t frame_index = 0;
            auto next_publish_time = std::chrono::steady_clock::time_point{};

            std::cout << "[StateEstimatorHMB] D435i depth stream started"
                      << (d435i_config_.serial.empty() ? "" : " serial=" + d435i_config_.serial)
                      << ", depth_scale=" << depth_scale << std::endl;

            while (running_.load() && g_running.load())
            {
                const rs2::frameset frames = pipe->wait_for_frames();
                const rs2::depth_frame depth_frame = frames.get_depth_frame();
                if (!depth_frame)
                {
                    continue;
                }

                if (d435i_config_.publish_fps > 0.0 &&
                    d435i_config_.publish_fps < static_cast<double>(d435i_config_.fps))
                {
                    const auto now = std::chrono::steady_clock::now();
                    if (next_publish_time.time_since_epoch().count() == 0)
                    {
                        next_publish_time = now;
                    }
                    if (now < next_publish_time)
                    {
                        continue;
                    }
                    const auto publish_period =
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(1.0 / d435i_config_.publish_fps));
                    do
                    {
                        next_publish_time += publish_period;
                    } while (next_publish_time <= now);
                }

                DepthFrameData data = BuildDepthFrameData(depth_frame, pointcloud, depth_scale, ++frame_index);
                {
                    std::lock_guard<std::mutex> lock(depth_mutex_);
                    latest_depth_frame_ = std::move(data);
                    has_depth_frame_ = true;
                }
                depth_cv_.notify_one();
            }
        }
        catch (const rs2::error& e)
        {
            std::cerr << "[StateEstimatorHMB] D435i RealSense error in "
                      << e.get_failed_function() << "(" << e.get_failed_args()
                      << "): " << e.what()
                      << ". Heightmap updates disabled; state estimation continues."
                      << std::endl;
            depth_cv_.notify_all();
        }
        catch (const std::exception& e)
        {
            std::cerr << "[StateEstimatorHMB] D435i fatal: " << e.what()
                      << ". Heightmap updates disabled; state estimation continues."
                      << std::endl;
            depth_cv_.notify_all();
        }
        StopRealSensePipeline(pipe);
    }

    void StopRealSensePipeline(std::unique_ptr<rs2::pipeline>& pipe)
    {
        if (!pipe)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(realsense_pipeline_mutex_);
        try
        {
            pipe->stop();
        }
        catch (const rs2::error& e)
        {
            std::cerr << "[StateEstimatorHMB] RealSense stop error in "
                      << e.get_failed_function() << "(" << e.get_failed_args()
                      << "): " << e.what() << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cerr << "[StateEstimatorHMB] RealSense stop fatal: " << e.what() << std::endl;
        }
        pipe.reset();
    }

    DepthFrameData BuildDepthFrameData(
        const rs2::depth_frame& depth_frame,
        rs2::pointcloud& pointcloud,
        double depth_scale,
        uint64_t frame_index) const
    {
        const int width = depth_frame.get_width();
        const int height = depth_frame.get_height();
        const int downsample =
            depth_config_.enable_downsampling ? std::max(1, depth_config_.downsample_factor) : 1;
        const size_t pixel_count = static_cast<size_t>(width) * static_cast<size_t>(height);

        std::vector<float> depth_m(pixel_count, std::numeric_limits<float>::quiet_NaN());
        const auto* raw_depth = static_cast<const uint16_t*>(depth_frame.get_data());
        for (size_t i = 0; i < pixel_count; ++i)
        {
            if (raw_depth[i] != 0u)
            {
                depth_m[i] = static_cast<float>(static_cast<double>(raw_depth[i]) * depth_scale);
            }
        }

        const rs2::points points = pointcloud.calculate(depth_frame);
        const rs2::vertex* vertices = points.get_vertices();
        if (vertices == nullptr)
        {
            throw std::runtime_error("[StateEstimatorHMB] RealSense pointcloud returned null vertices.");
        }

        DepthFrameData data;
        data.timestamp_ns = NowNs();
        data.frame_index = frame_index;
        data.cam_x.reserve(pixel_count / static_cast<size_t>(downsample * downsample) + 1u);
        data.cam_y.reserve(data.cam_x.capacity());
        data.cam_z.reserve(data.cam_x.capacity());

        const double min_depth = std::max(depth_config_.min_depth_m, d435i_config_.min_depth_m);
        const double max_depth = std::min(depth_config_.max_depth_m, d435i_config_.max_depth_m);

        for (int v = 0; v < height; v += downsample)
        {
            for (int u = 0; u < width; u += downsample)
            {
                const size_t idx = static_cast<size_t>(v) * static_cast<size_t>(width) +
                                   static_cast<size_t>(u);
                const float z = depth_m[idx];
                if (!std::isfinite(z) || z < min_depth || z > max_depth)
                {
                    continue;
                }
                if (IsOutlierDepth(depth_m, width, height, u, v, z, depth_config_))
                {
                    continue;
                }

                const rs2::vertex& vertex = vertices[idx];
                if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y) || !std::isfinite(vertex.z))
                {
                    continue;
                }

                data.cam_x.push_back(vertex.x);
                data.cam_y.push_back(vertex.y);
                data.cam_z.push_back(vertex.z);
            }
        }

        return data;
    }

    void HeightMapThread()
    {
        uint64_t last_frame_index = 0;
        size_t skipped_sync = 0;

        std::cout << "[StateEstimatorHMB] heightmap loop started" << std::endl;

        while (running_.load() && g_running.load())
        {
            DepthFrameData frame;
            {
                std::unique_lock<std::mutex> lock(depth_mutex_);
                depth_cv_.wait(lock, [this, last_frame_index]
                {
                    return !running_.load() || !g_running.load() ||
                           (has_depth_frame_ && latest_depth_frame_.frame_index != last_frame_index);
                });
                if (!running_.load() || !g_running.load())
                {
                    break;
                }
                frame = latest_depth_frame_;
                last_frame_index = frame.frame_index;
            }

            hmb::RobotStateSnapshot snapshot;
            const bool has_snapshot = FindNearestRobotState(frame.timestamp_ns, &snapshot);
            if (!has_snapshot)
            {
                ++skipped_sync;
                continue;
            }

            const double dt_sec =
                std::fabs(static_cast<double>(frame.timestamp_ns - snapshot.state_timestamp_ns)) * 1e-9;
            if (depth_config_.require_recent_robot_state && dt_sec > depth_config_.max_sync_dt_sec)
            {
                ++skipped_sync;
                if (d435i_config_.verbose && skipped_sync % 30u == 0u)
                {
                    std::cerr << "[StateEstimatorHMB] skipped depth frames by sync: "
                              << skipped_sync << ", last dt=" << dt_sec << " sec" << std::endl;
                }
                continue;
            }

            heightmap_builder_->ProcessCameraPointCloudFrame(
                frame.timestamp_ns,
                snapshot,
                frame.cam_x,
                frame.cam_y,
                frame.cam_z);
        }
    }

    void PublishRobotState(
        const std::string& channel,
        int64_t timestamp_ns,
        const RobotData& robot_state,
        const LegData& leg_state)
    {
        mors_msgs::robot_state_msg msg;
        msg.timestamp = timestamp_ns;
        for (int i = 0; i < 3; ++i)
        {
            msg.body.position[i] = robot_state.pos(i);
            msg.body.orientation[i] = robot_state.orientation(i);
            msg.body.orientation_quaternion[i] = robot_state.orientation_quaternion(i);
            msg.body.lin_vel[i] = robot_state.lin_vel(i);
            msg.body.ang_vel[i] = robot_state.ang_vel(i);

            msg.legs.r1_grf[i] = leg_state.r1_grf(i);
            msg.legs.l1_grf[i] = leg_state.l1_grf(i);
            msg.legs.r2_grf[i] = leg_state.r2_grf(i);
            msg.legs.l2_grf[i] = leg_state.l2_grf(i);

            msg.legs.r1_pos[i] = leg_state.r1_pos(i);
            msg.legs.l1_pos[i] = leg_state.l1_pos(i);
            msg.legs.r2_pos[i] = leg_state.r2_pos(i);
            msg.legs.l2_pos[i] = leg_state.l2_pos(i);

            msg.legs.r1_vel[i] = leg_state.r1_vel(i);
            msg.legs.l1_vel[i] = leg_state.l1_vel(i);
            msg.legs.r2_vel[i] = leg_state.r2_vel(i);
            msg.legs.l2_vel[i] = leg_state.l2_vel(i);
        }
        msg.body.orientation_quaternion[3] = robot_state.orientation_quaternion(3);
        for (int i = 0; i < 4; ++i)
        {
            msg.legs.contact_states[i] = leg_state.contacts[i];
        }

        robot_state_publisher_->publish(channel, &msg);
    }

    void PushRobotStateSnapshot(int64_t timestamp_ns, const RobotData& robot_state)
    {
        hmb::RobotStateSnapshot snapshot;
        snapshot.valid = true;
        snapshot.receive_timestamp_ns = timestamp_ns;
        snapshot.state_timestamp_ns = timestamp_ns;
        snapshot.yaw = robot_state.orientation(2);
        for (size_t i = 0; i < 3; ++i)
        {
            snapshot.position[i] = robot_state.pos(static_cast<int>(i));
            snapshot.orientation_rpy[i] = robot_state.orientation(static_cast<int>(i));
        }
        for (size_t i = 0; i < 4; ++i)
        {
            snapshot.orientation_quaternion[i] = robot_state.orientation_quaternion(static_cast<int>(i));
        }

        std::lock_guard<std::mutex> lock(state_history_mutex_);
        state_history_.push_back(snapshot);
        while (state_history_.size() > kStateHistoryCapacity)
        {
            state_history_.pop_front();
        }
    }

    bool FindNearestRobotState(int64_t timestamp_ns, hmb::RobotStateSnapshot* out) const
    {
        if (out == nullptr)
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(state_history_mutex_);
        if (state_history_.empty())
        {
            return false;
        }

        auto best = state_history_.begin();
        int64_t best_dt = std::llabs(best->state_timestamp_ns - timestamp_ns);
        for (auto it = std::next(state_history_.begin()); it != state_history_.end(); ++it)
        {
            const int64_t dt = std::llabs(it->state_timestamp_ns - timestamp_ns);
            if (dt < best_dt)
            {
                best = it;
                best_dt = dt;
            }
        }

        *out = *best;
        return out->valid;
    }

    std::string config_dir_;
    std::string control_lcm_url_;
    std::string servo_lcm_url_;
    std::string vision_lcm_url_;
    ChannelsConfig channels_;
    StateEstimatorConfig se_config_;
    RobotPhysicalParams robot_params_;
    T265Config t265_config_;
    D435iConfig d435i_config_;
    DepthProcessingConfig depth_config_;

    std::unique_ptr<lcm::LCM> imu_lcm_;
    std::unique_ptr<lcm::LCM> servo_lcm_;
    std::unique_ptr<lcm::LCM> contact_lcm_;
    std::unique_ptr<lcm::LCM> robot_state_publisher_;
    std::unique_ptr<lcm::LCM> servo_filtered_publisher_;
    std::unique_ptr<hmb::HeightMapBuilderNode> heightmap_builder_;

    std::atomic_bool running_{false};
    std::vector<std::thread> threads_;

    mutable std::mutex inputs_mutex_;
    SharedInputs shared_inputs_;

    mutable std::mutex state_history_mutex_;
    std::deque<hmb::RobotStateSnapshot> state_history_;

    std::mutex depth_mutex_;
    std::condition_variable depth_cv_;
    DepthFrameData latest_depth_frame_;
    bool has_depth_frame_{false};
    std::mutex realsense_pipeline_mutex_;
};

void SignalHandler(int)
{
    g_running.store(false);
}

}  // namespace

int main()
{
    try
    {
        std::cout << std::unitbuf;
        std::cerr << std::unitbuf;
        std::signal(SIGINT, SignalHandler);
        std::signal(SIGTERM, SignalHandler);

        StateEstimatorHMBApp app;
        app.Run();
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[StateEstimatorHMB] fatal: " << e.what() << std::endl;
        return 1;
    }
}
