// leg_state.hpp includes pinocchio through Robot.hpp, so keep it before any
// legacy X/Y/Z macro definitions that may appear in copied headers.
#include "leg_state.hpp"

#include "StateEstimator1DKF/HeightMapBuilder.hpp"
#include "gm_force_observer.hpp"
#include "low_pass_filtering.hpp"
#include "sensor_fusion.hpp"
#include "vertical_kalman_filter.hpp"

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

#include "mors_msgs/imu_lcm_data.hpp"
#include "mors_msgs/phase_signal_msg.hpp"
#include "mors_msgs/robot_state_msg.hpp"
#include "mors_msgs/servo_state_msg.hpp"
#include "system_functions.hpp"

namespace
{

constexpr int kStateHistoryCapacity = 2048;
constexpr int kLkfStateSize = 9;
constexpr int kLkfMeasurementSize = 12;
constexpr int kLkfMeasurementsPerAxis = 4;
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
        throw std::runtime_error(std::string("[StateEstimator1DKF] ") + name + " must be set.");
    }
    return value;
}

std::string ConfigPath(const std::string& config_dir, const std::string& file_name)
{
    const std::filesystem::path path = std::filesystem::path(config_dir) / file_name;
    if (!std::filesystem::exists(path))
    {
        throw std::runtime_error("[StateEstimator1DKF] config file not found: " + path.string());
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
    std::string gait_phase;
    std::string robot_state;
};

struct KalmanFilterConfig
{
    double sigma_a{1e-1};
    Eigen::Matrix<double, kLkfMeasurementSize, 1> sigma_z =
        Eigen::Matrix<double, kLkfMeasurementSize, 1>::Constant(1e-11);
    double p0{1e-1};
    double initial_z{0.036};
    double imu_accel_clip_mps2{0.0};
};

enum class ContactSource
{
    GaitPhase,
    Grf,
    Both,
};

struct KinematicPositionConfig
{
    bool enabled{true};
    ContactSource contact_source{ContactSource::GaitPhase};
    std::array<bool, 3> use_axis{false, false, true};
    int min_support_legs{1};
    Eigen::Vector3d sigma{0.05, 0.05, 1e-3};
    Eigen::Vector3d max_innovation{0.20, 0.20, 0.08};
};

struct VerticalFilterConfig
{
    state_estimator_1d::VerticalProcessNoise process_noise;
    double initial_position_std{0.10};
    double initial_velocity_std{0.20};
    double initial_acceleration_bias_std{0.50};
    double t265_position_std{0.20};
    double t265_velocity_std{0.20};
    double leg_velocity_std{0.06};
    double foot_constraint_std{0.005};
    double foot_anchor_initialization_std{0.02};
    double t265_position_gate_sigma{5.0};
    double t265_velocity_gate_sigma{5.0};
    double leg_velocity_gate_sigma{4.0};
    double foot_constraint_gate_sigma{4.0};
    double max_imu_age_sec{0.05};
    int contact_on_samples{5};
    int contact_off_samples{5};
    int min_tracker_confidence{1};
    bool use_t265_position{false};
    bool use_t265_velocity{true};
};

struct StateEstimatorConfig
{
    double dt{0.002};
    double contact_threshold{1.0};
    Eigen::Vector3d camera_offset{0.0, 0.0, 0.0};
    KalmanFilterConfig kalman_filter;
    KinematicPositionConfig kinematic_position;
    VerticalFilterConfig vertical_filter;
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
    std::vector<int> gait_phase{STANCE, STANCE, STANCE, STANCE};
    int64_t imu_timestamp_ns{0};
    int64_t servo_timestamp_ns{0};
    int64_t gait_phase_timestamp_ns{0};
    int64_t odometry_timestamp_ns{0};
    uint64_t imu_sequence{0};
    uint64_t servo_sequence{0};
    uint64_t gait_phase_sequence{0};
    uint64_t odometry_sequence{0};
    unsigned int tracker_confidence{0};
    unsigned int mapper_confidence{0};
    bool has_imu{false};
    bool has_servo{false};
    bool has_odometry{false};
    bool has_gait_phase{false};
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
        root["gait_phase"].as<std::string>(),
        root["robot_state"].as<std::string>()};
}

void LoadKalmanMeasurementNoise(const YAML::Node& node, KalmanFilterConfig& config)
{
    if (!node)
    {
        return;
    }

    if (node.IsScalar())
    {
        config.sigma_z.setConstant(node.as<double>());
        return;
    }

    if (!node.IsSequence())
    {
        throw std::runtime_error("[StateEstimator1DKF] kalman_filter.sigma_z must be a scalar or sequence.");
    }

    if (node.size() == kLkfMeasurementsPerAxis)
    {
        for (int axis = 0; axis < 3; ++axis)
        {
            for (int measurement = 0; measurement < kLkfMeasurementsPerAxis; ++measurement)
            {
                config.sigma_z(axis * kLkfMeasurementsPerAxis + measurement) =
                    node[measurement].as<double>();
            }
        }
    }
    else if (node.size() == kLkfMeasurementSize)
    {
        for (int i = 0; i < kLkfMeasurementSize; ++i)
        {
            config.sigma_z(i) = node[i].as<double>();
        }
    }
    else
    {
        throw std::runtime_error(
            "[StateEstimator1DKF] kalman_filter.sigma_z must contain 4 or 12 values.");
    }

    for (int i = 0; i < kLkfMeasurementSize; ++i)
    {
        if (!(config.sigma_z(i) > 0.0))
        {
            throw std::runtime_error("[StateEstimator1DKF] kalman_filter.sigma_z values must be > 0.");
        }
    }
}

Eigen::Vector3d LoadVector3d(
    const YAML::Node& node,
    const Eigen::Vector3d& fallback,
    const char* field_name,
    bool require_positive)
{
    if (!node)
    {
        return fallback;
    }
    if (!node.IsSequence() || node.size() != 3)
    {
        throw std::runtime_error(std::string("[StateEstimator1DKF] ") + field_name +
                                 " must contain exactly 3 values.");
    }

    Eigen::Vector3d value;
    for (int i = 0; i < 3; ++i)
    {
        value(i) = node[i].as<double>();
        if (require_positive && !(value(i) > 0.0))
        {
            throw std::runtime_error(std::string("[StateEstimator1DKF] ") + field_name +
                                     " values must be > 0.");
        }
    }
    return value;
}

ContactSource ParseContactSource(const YAML::Node& node, ContactSource fallback)
{
    if (!node)
    {
        return fallback;
    }

    const std::string value = node.as<std::string>();
    if (value == "gait_phase")
    {
        return ContactSource::GaitPhase;
    }
    if (value == "grf")
    {
        return ContactSource::Grf;
    }
    if (value == "both")
    {
        return ContactSource::Both;
    }

    throw std::runtime_error(
        "[StateEstimator1DKF] kinematic_position.contact_source must be gait_phase, grf or both.");
}

void LoadKinematicPositionConfig(const YAML::Node& node, KinematicPositionConfig& config)
{
    if (!node)
    {
        return;
    }

    if (node["enabled"]) config.enabled = node["enabled"].as<bool>();
    config.contact_source = ParseContactSource(node["contact_source"], config.contact_source);
    if (node["use_x"]) config.use_axis[X] = node["use_x"].as<bool>();
    if (node["use_y"]) config.use_axis[Y] = node["use_y"].as<bool>();
    if (node["use_z"]) config.use_axis[Z] = node["use_z"].as<bool>();
    if (node["min_support_legs"])
    {
        config.min_support_legs = std::clamp(node["min_support_legs"].as<int>(), 1, NUM_LEGS);
    }
    config.sigma = LoadVector3d(node["sigma"], config.sigma, "kinematic_position.sigma", true);
    config.max_innovation = LoadVector3d(
        node["max_innovation"],
        config.max_innovation,
        "kinematic_position.max_innovation",
        true);
}

double LoadPositiveDouble(
    const YAML::Node& node,
    const char* key,
    double fallback,
    bool allow_zero = false)
{
    if (!node || !node[key])
    {
        return fallback;
    }

    const double value = node[key].as<double>();
    const bool valid = std::isfinite(value) && (allow_zero ? value >= 0.0 : value > 0.0);
    if (!valid)
    {
        throw std::runtime_error(
            std::string("[StateEstimator1DKF] kalman_filter_1d.") + key +
            (allow_zero ? " must be >= 0." : " must be > 0."));
    }
    return value;
}

void LoadVerticalFilterConfig(const YAML::Node& node, VerticalFilterConfig& config)
{
    if (!node)
    {
        return;
    }

    config.process_noise.acceleration_std =
        LoadPositiveDouble(node, "acceleration_std", config.process_noise.acceleration_std);
    config.process_noise.acceleration_bias_random_walk_std = LoadPositiveDouble(
        node,
        "acceleration_bias_random_walk_std",
        config.process_noise.acceleration_bias_random_walk_std);
    config.process_noise.stance_foot_height_random_walk_std = LoadPositiveDouble(
        node,
        "stance_foot_height_random_walk_std",
        config.process_noise.stance_foot_height_random_walk_std,
        true);
    config.process_noise.max_prediction_step_sec = LoadPositiveDouble(
        node,
        "max_prediction_step_sec",
        config.process_noise.max_prediction_step_sec);

    config.initial_position_std =
        LoadPositiveDouble(node, "initial_position_std", config.initial_position_std);
    config.initial_velocity_std =
        LoadPositiveDouble(node, "initial_velocity_std", config.initial_velocity_std);
    config.initial_acceleration_bias_std = LoadPositiveDouble(
        node,
        "initial_acceleration_bias_std",
        config.initial_acceleration_bias_std);
    config.t265_position_std =
        LoadPositiveDouble(node, "t265_position_std", config.t265_position_std);
    config.t265_velocity_std =
        LoadPositiveDouble(node, "t265_velocity_std", config.t265_velocity_std);
    config.leg_velocity_std =
        LoadPositiveDouble(node, "leg_velocity_std", config.leg_velocity_std);
    config.foot_constraint_std =
        LoadPositiveDouble(node, "foot_constraint_std", config.foot_constraint_std);
    config.foot_anchor_initialization_std = LoadPositiveDouble(
        node,
        "foot_anchor_initialization_std",
        config.foot_anchor_initialization_std);

    config.t265_position_gate_sigma = LoadPositiveDouble(
        node,
        "t265_position_gate_sigma",
        config.t265_position_gate_sigma,
        true);
    config.t265_velocity_gate_sigma = LoadPositiveDouble(
        node,
        "t265_velocity_gate_sigma",
        config.t265_velocity_gate_sigma,
        true);
    config.leg_velocity_gate_sigma = LoadPositiveDouble(
        node,
        "leg_velocity_gate_sigma",
        config.leg_velocity_gate_sigma,
        true);
    config.foot_constraint_gate_sigma = LoadPositiveDouble(
        node,
        "foot_constraint_gate_sigma",
        config.foot_constraint_gate_sigma,
        true);
    config.max_imu_age_sec =
        LoadPositiveDouble(node, "max_imu_age_sec", config.max_imu_age_sec);

    if (node["contact_on_samples"])
    {
        config.contact_on_samples = std::max(1, node["contact_on_samples"].as<int>());
    }
    if (node["contact_off_samples"])
    {
        config.contact_off_samples = std::max(1, node["contact_off_samples"].as<int>());
    }
    if (node["min_tracker_confidence"])
    {
        config.min_tracker_confidence =
            std::clamp(node["min_tracker_confidence"].as<int>(), 0, 3);
    }
    if (node["use_t265_position"])
    {
        config.use_t265_position = node["use_t265_position"].as<bool>();
    }
    if (node["use_t265_velocity"])
    {
        config.use_t265_velocity = node["use_t265_velocity"].as<bool>();
    }
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

    if (const YAML::Node kalman = se["kalman_filter"])
    {
        if (kalman["sigma_a"]) config.kalman_filter.sigma_a = kalman["sigma_a"].as<double>();
        if (kalman["sigma_z"]) LoadKalmanMeasurementNoise(kalman["sigma_z"], config.kalman_filter);
        if (kalman["p0"]) config.kalman_filter.p0 = kalman["p0"].as<double>();
        if (kalman["initial_z"]) config.kalman_filter.initial_z = kalman["initial_z"].as<double>();
        if (kalman["imu_accel_clip_mps2"])
        {
            config.kalman_filter.imu_accel_clip_mps2 =
                std::max(0.0, kalman["imu_accel_clip_mps2"].as<double>());
        }
    }
    LoadKinematicPositionConfig(se["kinematic_position"], config.kinematic_position);
    LoadVerticalFilterConfig(se["kalman_filter_1d"], config.vertical_filter);
    return config;
}

Eigen::Vector3d ClipImuAcceleration(const Eigen::Vector3d& accel, double clip_mps2)
{
    if (!(clip_mps2 > 0.0))
    {
        return accel;
    }

    Eigen::Vector3d clipped = accel;
    for (int i = 0; i < 3; ++i)
    {
        clipped(i) = std::clamp(clipped(i), -clip_mps2, clip_mps2);
    }
    return clipped;
}

bool IsSupportPhase(int phase)
{
    return phase == STANCE || phase == EARLY_CONTACT;
}

std::vector<bool> BuildSupportMask(const std::vector<int>& gait_phase, bool has_gait_phase)
{
    std::vector<bool> support(NUM_LEGS, false);
    if (!has_gait_phase)
    {
        return support;
    }

    for (int i = 0; i < NUM_LEGS && i < static_cast<int>(gait_phase.size()); ++i)
    {
        support[i] = IsSupportPhase(gait_phase[i]);
    }
    return support;
}

std::vector<bool> BuildContactSupportMask(const std::vector<bool>& contacts)
{
    std::vector<bool> support(NUM_LEGS, false);
    for (int i = 0; i < NUM_LEGS && i < static_cast<int>(contacts.size()); ++i)
    {
        support[i] = contacts[static_cast<size_t>(i)];
    }
    return support;
}

std::vector<bool> SelectKinematicSupportMask(
    const std::vector<bool>& gait_support_mask,
    const std::vector<bool>& contact_support_mask,
    ContactSource contact_source)
{
    std::vector<bool> support(NUM_LEGS, false);
    for (int i = 0; i < NUM_LEGS; ++i)
    {
        const bool gait_support =
            i < static_cast<int>(gait_support_mask.size()) && gait_support_mask[static_cast<size_t>(i)];
        const bool contact_support =
            i < static_cast<int>(contact_support_mask.size()) && contact_support_mask[static_cast<size_t>(i)];

        switch (contact_source)
        {
            case ContactSource::GaitPhase:
                support[i] = gait_support;
                break;
            case ContactSource::Grf:
                support[i] = contact_support;
                break;
            case ContactSource::Both:
                support[i] = gait_support && contact_support;
                break;
        }
    }
    return support;
}

struct VerticalLegKinematicsMeasurement
{
    std::array<double, NUM_LEGS> foot_offset_world_z{};
    std::array<double, NUM_LEGS> body_velocity_world_z{};
    std::array<bool, NUM_LEGS> valid{false, false, false, false};
};

class VerticalLegKinematics
{
public:
    VerticalLegKinematics()
    {
        robot_.BuildPinocchioModel();
        q_ = Eigen::VectorXd::Zero(robot_.nq);
        v_ = Eigen::VectorXd::Zero(robot_.nv);
        q_(6) = 1.0;
    }

    VerticalLegKinematicsMeasurement Evaluate(
        const ServoData& servo_state,
        const Eigen::Matrix3d& world_R_body,
        const Eigen::Vector3d& angular_velocity_body)
    {
        VerticalLegKinematicsMeasurement measurement;
        if (servo_state.pos.size() < 12 || servo_state.vel.size() < 12)
        {
            return measurement;
        }

        q_.setZero();
        v_.setZero();
        q_(6) = 1.0;
        q_.segment(7, 3) = servo_state.pos.segment(3, 3);
        q_.segment(10, 3) = servo_state.pos.segment(9, 3);
        q_.segment(13, 3) = servo_state.pos.segment(0, 3);
        q_.segment(16, 3) = servo_state.pos.segment(6, 3);

        robot_.ComputeForwardKinematics(q_, v_);
        const std::vector<Eigen::Vector3d> pin_positions =
            robot_.GetToePositionsInBaseFrame();
        const std::vector<Eigen::Matrix3d> pin_jacobians = robot_.GetFootJacobian(q_);

        std::array<Eigen::Vector3d, NUM_LEGS> positions;
        std::array<Eigen::Matrix3d, NUM_LEGS> jacobians;
        positions[R1] = pin_positions[PIN_R1];
        positions[L1] = pin_positions[PIN_L1];
        positions[R2] = pin_positions[PIN_R2];
        positions[L2] = pin_positions[PIN_L2];
        jacobians[R1] = pin_jacobians[PIN_R1];
        jacobians[L1] = pin_jacobians[PIN_L1];
        jacobians[R2] = pin_jacobians[PIN_R2];
        jacobians[L2] = pin_jacobians[PIN_L2];

        for (int leg = 0; leg < NUM_LEGS; ++leg)
        {
            const Eigen::Vector3d joint_velocity =
                servo_state.vel.segment(3 * leg, 3);
            const Eigen::Vector3d foot_velocity_body =
                jacobians[static_cast<size_t>(leg)] * joint_velocity;
            const Eigen::Vector3d rotational_velocity_body =
                angular_velocity_body.cross(positions[static_cast<size_t>(leg)]);
            const Eigen::Vector3d body_velocity_world =
                world_R_body * (-(foot_velocity_body + rotational_velocity_body));
            const Eigen::Vector3d foot_offset_world =
                world_R_body * positions[static_cast<size_t>(leg)];

            measurement.foot_offset_world_z[static_cast<size_t>(leg)] =
                foot_offset_world(Z);
            measurement.body_velocity_world_z[static_cast<size_t>(leg)] =
                body_velocity_world(Z);
            measurement.valid[static_cast<size_t>(leg)] =
                std::isfinite(foot_offset_world(Z)) &&
                std::isfinite(body_velocity_world(Z));
        }
        return measurement;
    }

private:
    Robot robot_;
    Eigen::VectorXd q_;
    Eigen::VectorXd v_;
};

class DebouncedSupportEstimator
{
public:
    DebouncedSupportEstimator(int contact_on_samples, int contact_off_samples)
        : contact_on_samples_(std::max(1, contact_on_samples)),
          contact_off_samples_(std::max(1, contact_off_samples))
    {
        support_.assign(NUM_LEGS, false);
        on_count_.fill(0);
        off_count_.fill(0);
    }

    void Update(const std::vector<bool>& requested_support)
    {
        for (int leg = 0; leg < NUM_LEGS; ++leg)
        {
            const bool requested =
                leg < static_cast<int>(requested_support.size()) && requested_support[leg];
            if (requested)
            {
                off_count_[static_cast<size_t>(leg)] = 0;
                on_count_[static_cast<size_t>(leg)] = std::min(
                    contact_on_samples_,
                    on_count_[static_cast<size_t>(leg)] + 1);
                if (on_count_[static_cast<size_t>(leg)] >= contact_on_samples_)
                {
                    support_[static_cast<size_t>(leg)] = true;
                }
            }
            else
            {
                on_count_[static_cast<size_t>(leg)] = 0;
                off_count_[static_cast<size_t>(leg)] = std::min(
                    contact_off_samples_,
                    off_count_[static_cast<size_t>(leg)] + 1);
                if (off_count_[static_cast<size_t>(leg)] >= contact_off_samples_)
                {
                    support_[static_cast<size_t>(leg)] = false;
                }
            }
        }
    }

    [[nodiscard]] const std::vector<bool>& support() const
    {
        return support_;
    }

private:
    int contact_on_samples_;
    int contact_off_samples_;
    std::array<int, NUM_LEGS> on_count_{};
    std::array<int, NUM_LEGS> off_count_{};
    std::vector<bool> support_;
};

bool HasRequiredStateInputs(const SharedInputs& inputs)
{
    return inputs.has_imu &&
           inputs.has_servo &&
           inputs.has_odometry &&
           inputs.servo.pos.size() >= 12 &&
           inputs.servo.vel.size() >= 12 &&
           inputs.servo.torq.size() >= 12;
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

class StateEstimator1DKFApp
{
public:
    StateEstimator1DKFApp()
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
          gait_phase_lcm_(std::make_unique<lcm::LCM>(control_lcm_url_)),
          robot_state_publisher_(std::make_unique<lcm::LCM>(control_lcm_url_)),
          servo_filtered_publisher_(std::make_unique<lcm::LCM>(servo_lcm_url_)),
          heightmap_builder_(std::make_unique<hmb::HeightMapBuilderNode>(
              ConfigPath(config_dir_, "heightmap_builder.yaml"),
              false))
    {
        if (!imu_lcm_->good() || !servo_lcm_->good() || !gait_phase_lcm_->good() ||
            !robot_state_publisher_->good() || !servo_filtered_publisher_->good())
        {
            throw std::runtime_error("[StateEstimator1DKF] failed to initialize one or more LCM endpoints.");
        }

        shared_inputs_.servo.pos = Eigen::VectorXd::Zero(12);
        shared_inputs_.servo.vel = Eigen::VectorXd::Zero(12);
        shared_inputs_.servo.torq = Eigen::VectorXd::Zero(12);

        std::cout << "[StateEstimator1DKF] loaded configs from " << config_dir_ << "\n"
                  << "  state dt: " << se_config_.dt << " sec\n"
                  << "  D435i: " << d435i_config_.width << "x" << d435i_config_.height
                  << "@" << d435i_config_.fps << "\n"
                  << "  1D KF T265 Z updates: "
                  << (se_config_.vertical_filter.use_t265_position ? "enabled" : "disabled")
                  << "\n"
                  << "  1D KF contact debounce: on="
                  << se_config_.vertical_filter.contact_on_samples
                  << ", off=" << se_config_.vertical_filter.contact_off_samples << " samples\n"
                  << "  depth/state sync max dt: " << depth_config_.max_sync_dt_sec << " sec"
                  << std::endl;
    }

    void Run()
    {
        running_.store(true);
        threads_.emplace_back(&StateEstimator1DKFApp::ImuLcmThread, this);
        threads_.emplace_back(&StateEstimator1DKFApp::ServoLcmThread, this);
        threads_.emplace_back(&StateEstimator1DKFApp::GaitPhaseLcmThread, this);
        threads_.emplace_back(&StateEstimator1DKFApp::PoseCameraThread, this);
        threads_.emplace_back(&StateEstimator1DKFApp::StateEstimatorThread, this);
        threads_.emplace_back(&StateEstimator1DKFApp::DepthCameraThread, this);
        threads_.emplace_back(&StateEstimator1DKFApp::HeightMapThread, this);

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
        shared_inputs_.imu_timestamp_ns = NowNs();
        ++shared_inputs_.imu_sequence;
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
        shared_inputs_.servo_timestamp_ns = NowNs();
        ++shared_inputs_.servo_sequence;
        shared_inputs_.has_servo = true;
    }

    void GaitPhaseHandler(
        const lcm::ReceiveBuffer*,
        const std::string&,
        const mors_msgs::phase_signal_msg* msg)
    {
        if (msg == nullptr)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(inputs_mutex_);
        for (int i = 0; i < 4; ++i)
        {
            shared_inputs_.gait_phase[i] = msg->phase[i];
        }
        shared_inputs_.gait_phase_timestamp_ns = NowNs();
        ++shared_inputs_.gait_phase_sequence;
        shared_inputs_.has_gait_phase = true;
    }

    void ImuLcmThread()
    {
        imu_lcm_->subscribe(channels_.imu_data, &StateEstimator1DKFApp::ImuHandler, this);
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
        servo_lcm_->subscribe(channels_.servo_state, &StateEstimator1DKFApp::ServoHandler, this);
        while (running_.load() && g_running.load())
        {
            if (servo_lcm_->handleTimeout(20) < 0)
            {
                running_.store(false);
            }
        }
    }

    void GaitPhaseLcmThread()
    {
        gait_phase_lcm_->subscribe(channels_.gait_phase, &StateEstimator1DKFApp::GaitPhaseHandler, this);
        while (running_.load() && g_running.load())
        {
            if (gait_phase_lcm_->handleTimeout(20) < 0)
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

            std::cout << "[StateEstimator1DKF] T265 pose stream started"
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
                ++shared_inputs_.odometry_sequence;
                shared_inputs_.tracker_confidence = pose.tracker_confidence;
                shared_inputs_.mapper_confidence = pose.mapper_confidence;
                shared_inputs_.has_odometry = true;
            }
        }
        catch (const rs2::error& e)
        {
            std::cerr << "[StateEstimator1DKF] T265 RealSense error in "
                      << e.get_failed_function() << "(" << e.get_failed_args()
                      << "): " << e.what() << std::endl;
            running_.store(false);
        }
        catch (const std::exception& e)
        {
            std::cerr << "[StateEstimator1DKF] T265 fatal: " << e.what() << std::endl;
            running_.store(false);
        }
        StopRealSensePipeline(pipe);
    }

    void StateEstimatorThread()
    {
        try
        {
            SensorFusion sensor_fusion;
            RobotData raw_robot_state;
            LegData raw_leg_state;
            LegState raw_leg_state_estimator(robot_params_);
            LegState filtered_leg_state_estimator(robot_params_);

            Eigen::VectorXd p(3);
            p << 0.032, -0.01, 0.001;
            Eigen::VectorXd initial_theta = Eigen::VectorXd::Zero(12);
            raw_leg_state_estimator.set_grf_observer_params(100.0, se_config_.dt, p, initial_theta);
            raw_leg_state_estimator.set_contact_threshold(se_config_.contact_threshold);
            filtered_leg_state_estimator.set_grf_observer_params(
                100.0,
                se_config_.dt,
                p,
                initial_theta);
            filtered_leg_state_estimator.set_contact_threshold(se_config_.contact_threshold);

            VerticalLegKinematics vertical_leg_kinematics;
            VerticalLegKinematicsMeasurement vertical_leg_measurement;
            DebouncedSupportEstimator support_estimator(
                se_config_.vertical_filter.contact_on_samples,
                se_config_.vertical_filter.contact_off_samples);
            state_estimator_1d::VerticalKalmanFilter vertical_filter(
                se_config_.vertical_filter.process_noise);

            const Eigen::Vector3d gravity(0.0, 0.0, robot_params_.g);
            const Eigen::Vector3d initial_body_position(0.0, 0.0, 0.038);

            bool first_pos = true;
            bool first_yaw = true;
            bool waiting_for_inputs_logged = false;
            Eigen::Vector3d pos_offset = Eigen::Vector3d::Zero();
            double yaw_offset = 0.0;
            uint64_t last_used_odometry_sequence = 0;
            uint64_t last_used_servo_sequence = 0;
            bool prediction_clock_initialized = false;
            auto last_prediction_time = std::chrono::steady_clock::now();

            const auto period = std::chrono::duration<double>(se_config_.dt);
            auto next_tick = std::chrono::steady_clock::now();

            std::cout << "[StateEstimator1DKF] state estimator loop started" << std::endl;

            while (running_.load() && g_running.load())
            {
                next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

                SharedInputs inputs;
                {
                    std::lock_guard<std::mutex> lock(inputs_mutex_);
                    inputs = shared_inputs_;
                }

                if (!HasRequiredStateInputs(inputs))
                {
                    if (!waiting_for_inputs_logged)
                    {
                        std::cout << "[StateEstimator1DKF] waiting for initial IMU, servo and T265 data"
                                  << std::endl;
                        waiting_for_inputs_logged = true;
                    }
                    prediction_clock_initialized = false;
                    std::this_thread::sleep_until(next_tick);
                    continue;
                }

                ServoData servo_state = inputs.servo;
                servo_state.torq *= kTorqueScale;

                raw_robot_state.orientation_quaternion = inputs.imu.orientation_quaternion;
                raw_robot_state.orientation = sensor_fusion.update_orientation(
                    inputs.imu.orientation_euler,
                    inputs.imu.orientation_euler);
                if (first_yaw)
                {
                    yaw_offset = raw_robot_state.orientation(2);
                    first_yaw = false;
                }
                raw_robot_state.orientation(2) -= yaw_offset;

                const Eigen::Matrix3d r_body = RotationFromEuler(
                    raw_robot_state.orientation(0),
                    raw_robot_state.orientation(1),
                    raw_robot_state.orientation(2));
                raw_robot_state.ang_vel = inputs.imu.ang_vel;

                const Eigen::Vector3d raw_body_pos =
                    inputs.odometry.position + r_body * se_config_.camera_offset;
                if (first_pos)
                {
                    pos_offset = raw_body_pos - initial_body_position;
                    first_pos = false;
                }
                raw_robot_state.pos = raw_body_pos - pos_offset;

                const Eigen::Vector3d ang_vel_world = r_body * raw_robot_state.ang_vel;
                Eigen::Matrix3d ang_vel_world_cross;
                ang_vel_world_cross << 0.0, -ang_vel_world(2), ang_vel_world(1),
                                       ang_vel_world(2), 0.0, -ang_vel_world(0),
                                      -ang_vel_world(1), ang_vel_world(0), 0.0;
                raw_robot_state.lin_vel =
                    inputs.odometry.lin_vel + ang_vel_world_cross * r_body * se_config_.camera_offset;

                raw_leg_state = raw_leg_state_estimator.get_leg_state(
                    raw_robot_state,
                    servo_state.pos,
                    servo_state.vel,
                    servo_state.torq);

                const std::vector<bool> gait_support_mask =
                    BuildSupportMask(inputs.gait_phase, inputs.has_gait_phase);
                const std::vector<bool> grf_support_mask =
                    BuildContactSupportMask(raw_leg_state.contacts);
                std::vector<bool> requested_support = SelectKinematicSupportMask(
                    gait_support_mask,
                    grf_support_mask,
                    se_config_.kinematic_position.contact_source);
                if (!se_config_.kinematic_position.enabled ||
                    !se_config_.kinematic_position.use_axis[Z])
                {
                    std::fill(requested_support.begin(), requested_support.end(), false);
                }

                const Eigen::Vector3d imu_accel_corrected =
                    ClipImuAcceleration(
                        r_body * inputs.imu.lin_accel - gravity,
                        se_config_.kalman_filter.imu_accel_clip_mps2);
                const double imu_age_sec = std::fabs(
                    static_cast<double>(NowNs() - inputs.imu_timestamp_ns)) * 1e-9;
                const bool imu_is_recent =
                    imu_age_sec <= se_config_.vertical_filter.max_imu_age_sec;

                const auto prediction_time = std::chrono::steady_clock::now();
                if (!vertical_filter.initialized())
                {
                    vertical_filter.Initialize(
                        raw_robot_state.pos(Z),
                        raw_robot_state.lin_vel(Z),
                        0.0,
                        se_config_.vertical_filter.initial_position_std,
                        se_config_.vertical_filter.initial_velocity_std,
                        se_config_.vertical_filter.initial_acceleration_bias_std);
                    last_used_odometry_sequence = inputs.odometry_sequence;
                    last_prediction_time = prediction_time;
                    prediction_clock_initialized = true;
                }
                else if (prediction_clock_initialized && imu_is_recent)
                {
                    const double elapsed_sec = std::chrono::duration<double>(
                        prediction_time - last_prediction_time).count();
                    // A long scheduling pause must not integrate a stale acceleration
                    // sample for an unbounded interval. New measurements will recover
                    // the omitted interval after the pause.
                    vertical_filter.Predict(
                        imu_accel_corrected(Z),
                        std::min(elapsed_sec, 0.05));
                    last_prediction_time = prediction_time;
                }
                else
                {
                    last_prediction_time = prediction_time;
                    prediction_clock_initialized = true;
                }

                if (inputs.odometry_sequence != last_used_odometry_sequence)
                {
                    const bool tracker_is_valid =
                        static_cast<int>(inputs.tracker_confidence) >=
                        se_config_.vertical_filter.min_tracker_confidence;
                    if (tracker_is_valid && se_config_.vertical_filter.use_t265_position)
                    {
                        static_cast<void>(vertical_filter.UpdatePosition(
                            raw_robot_state.pos(Z),
                            se_config_.vertical_filter.t265_position_std,
                            se_config_.vertical_filter.t265_position_gate_sigma));
                    }
                    if (tracker_is_valid && se_config_.vertical_filter.use_t265_velocity)
                    {
                        static_cast<void>(vertical_filter.UpdateVelocity(
                            raw_robot_state.lin_vel(Z),
                            se_config_.vertical_filter.t265_velocity_std,
                            se_config_.vertical_filter.t265_velocity_gate_sigma));
                    }
                    last_used_odometry_sequence = inputs.odometry_sequence;
                }

                if (inputs.servo_sequence != last_used_servo_sequence)
                {
                    support_estimator.Update(requested_support);
                    vertical_leg_measurement = vertical_leg_kinematics.Evaluate(
                        servo_state,
                        r_body,
                        raw_robot_state.ang_vel);

                    const std::vector<bool>& stable_support = support_estimator.support();
                    const int support_count = static_cast<int>(std::count(
                        stable_support.begin(),
                        stable_support.end(),
                        true));

                    for (int leg = 0; leg < NUM_LEGS; ++leg)
                    {
                        const bool support = stable_support[static_cast<size_t>(leg)];
                        if (!support)
                        {
                            vertical_filter.InvalidateFootAnchor(leg);
                            continue;
                        }
                        if (support_count < se_config_.kinematic_position.min_support_legs ||
                            !vertical_leg_measurement.valid[static_cast<size_t>(leg)])
                        {
                            continue;
                        }

                        const double foot_offset_z =
                            vertical_leg_measurement.foot_offset_world_z[static_cast<size_t>(leg)];
                        if (!vertical_filter.foot_anchor_valid(leg))
                        {
                            // Covariance is augmented from z here. No position
                            // correction is performed on this same sample.
                            vertical_filter.InitializeFootAnchor(
                                leg,
                                foot_offset_z,
                                se_config_.vertical_filter.foot_anchor_initialization_std);
                        }
                        else
                        {
                            static_cast<void>(vertical_filter.UpdateFootConstraint(
                                leg,
                                foot_offset_z,
                                se_config_.vertical_filter.foot_constraint_std,
                                se_config_.vertical_filter.foot_constraint_gate_sigma));
                        }

                        static_cast<void>(vertical_filter.UpdateVelocity(
                            vertical_leg_measurement.body_velocity_world_z[
                                static_cast<size_t>(leg)],
                            se_config_.vertical_filter.leg_velocity_std,
                            se_config_.vertical_filter.leg_velocity_gate_sigma));
                    }
                    last_used_servo_sequence = inputs.servo_sequence;
                }

                RobotData robot_state_to_publish = raw_robot_state;
                robot_state_to_publish.pos(Z) = vertical_filter.position();
                robot_state_to_publish.lin_vel(Z) = vertical_filter.velocity();
                LegData leg_state_to_publish = filtered_leg_state_estimator.get_leg_state(
                    robot_state_to_publish,
                    servo_state.pos,
                    servo_state.vel,
                    servo_state.torq);

                const int64_t timestamp_ns = NowNs();
                PublishRobotState(channels_.robot_state, timestamp_ns, robot_state_to_publish, leg_state_to_publish);
                // Keep an unfiltered reference channel for direct T265/1DKF comparison.
                PublishRobotState(kRobotStateCheckChannel, timestamp_ns, raw_robot_state, raw_leg_state);
                PushRobotStateSnapshot(timestamp_ns, robot_state_to_publish);

                std::this_thread::sleep_until(next_tick);
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "[StateEstimator1DKF] state estimator fatal: " << e.what() << std::endl;
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

            std::cout << "[StateEstimator1DKF] D435i depth stream started"
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
            std::cerr << "[StateEstimator1DKF] D435i RealSense error in "
                      << e.get_failed_function() << "(" << e.get_failed_args()
                      << "): " << e.what()
                      << ". Heightmap updates disabled; state estimation continues."
                      << std::endl;
            depth_cv_.notify_all();
        }
        catch (const std::exception& e)
        {
            std::cerr << "[StateEstimator1DKF] D435i fatal: " << e.what()
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
            std::cerr << "[StateEstimator1DKF] RealSense stop error in "
                      << e.get_failed_function() << "(" << e.get_failed_args()
                      << "): " << e.what() << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cerr << "[StateEstimator1DKF] RealSense stop fatal: " << e.what() << std::endl;
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
            throw std::runtime_error("[StateEstimator1DKF] RealSense pointcloud returned null vertices.");
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

        std::cout << "[StateEstimator1DKF] heightmap loop started" << std::endl;

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
                    std::cerr << "[StateEstimator1DKF] skipped depth frames by sync: "
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
    std::unique_ptr<lcm::LCM> gait_phase_lcm_;
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

        StateEstimator1DKFApp app;
        app.Run();
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[StateEstimator1DKF] fatal: " << e.what() << std::endl;
        return 1;
    }
}
