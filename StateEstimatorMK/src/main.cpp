// leg_state.hpp includes pinocchio through Robot.hpp, so keep it before any
// legacy X/Y/Z macro definitions that may appear in copied headers.
#include "leg_state.hpp"

#include "StateEstimatorMK/HeightMapBuilder.hpp"
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

#include "mors_msgs/imu_lcm_data.hpp"
#include "mors_msgs/phase_signal_msg.hpp"
#include "mors_msgs/robot_state_msg.hpp"
#include "mors_msgs/servo_state_msg.hpp"
#include "system_functions.hpp"

namespace
{

constexpr int kStateHistoryCapacity = 2048;
constexpr int kFootSupportHistoryCapacity = 2048;
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
        throw std::runtime_error(std::string("[StateEstimatorMK] ") + name + " must be set.");
    }
    return value;
}

std::string ConfigPath(const std::string& config_dir, const std::string& file_name)
{
    const std::filesystem::path path = std::filesystem::path(config_dir) / file_name;
    if (!std::filesystem::exists(path))
    {
        throw std::runtime_error("[StateEstimatorMK] config file not found: " + path.string());
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

struct HeightMapZCorrectionConfig
{
    bool enabled{true};
    double sigma{5e-3};
    double max_innovation{0.05};
    double max_age_sec{0.15};
    int min_support_legs{1};
    int map_radius_cells{2};
};

struct StateEstimatorConfig
{
    double dt{0.002};
    double contact_threshold{1.0};
    Eigen::Vector3d camera_offset{0.0, 0.0, 0.0};
    KalmanFilterConfig kalman_filter;
    KinematicPositionConfig kinematic_position;
    HeightMapZCorrectionConfig heightmap_z_correction;
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
    int64_t odometry_timestamp_ns{0};
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

struct FootSupportSnapshot
{
    bool valid{false};
    int64_t timestamp_ns{0};
    double body_z{0.0};
    std::array<std::array<double, 3>, NUM_LEGS> foot_position_world{};
    std::array<bool, NUM_LEGS> support{};
};

struct HeightMapZCorrection
{
    bool valid{false};
    int64_t timestamp_ns{0};
    double body_z_measurement{0.0};
    int support_count{0};
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
        throw std::runtime_error("[StateEstimatorMK] kalman_filter.sigma_z must be a scalar or sequence.");
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
            "[StateEstimatorMK] kalman_filter.sigma_z must contain 4 or 12 values.");
    }

    for (int i = 0; i < kLkfMeasurementSize; ++i)
    {
        if (!(config.sigma_z(i) > 0.0))
        {
            throw std::runtime_error("[StateEstimatorMK] kalman_filter.sigma_z values must be > 0.");
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
        throw std::runtime_error(std::string("[StateEstimatorMK] ") + field_name +
                                 " must contain exactly 3 values.");
    }

    Eigen::Vector3d value;
    for (int i = 0; i < 3; ++i)
    {
        value(i) = node[i].as<double>();
        if (require_positive && !(value(i) > 0.0))
        {
            throw std::runtime_error(std::string("[StateEstimatorMK] ") + field_name +
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
        "[StateEstimatorMK] kinematic_position.contact_source must be gait_phase, grf or both.");
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

void LoadHeightMapZCorrectionConfig(const YAML::Node& node, HeightMapZCorrectionConfig& config)
{
    if (!node)
    {
        return;
    }

    if (node["enabled"]) config.enabled = node["enabled"].as<bool>();
    if (node["sigma"])
    {
        config.sigma = node["sigma"].as<double>();
        if (!(config.sigma > 0.0))
        {
            throw std::runtime_error("[StateEstimatorMK] heightmap_z_correction.sigma must be > 0.");
        }
    }
    if (node["max_innovation"])
    {
        config.max_innovation = node["max_innovation"].as<double>();
        if (!(config.max_innovation > 0.0))
        {
            throw std::runtime_error("[StateEstimatorMK] heightmap_z_correction.max_innovation must be > 0.");
        }
    }
    if (node["max_age_sec"])
    {
        config.max_age_sec = node["max_age_sec"].as<double>();
        if (!(config.max_age_sec > 0.0))
        {
            throw std::runtime_error("[StateEstimatorMK] heightmap_z_correction.max_age_sec must be > 0.");
        }
    }
    if (node["min_support_legs"])
    {
        config.min_support_legs = std::clamp(node["min_support_legs"].as<int>(), 1, NUM_LEGS);
    }
    if (node["map_radius_cells"])
    {
        config.map_radius_cells = std::max(0, node["map_radius_cells"].as<int>());
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
    LoadHeightMapZCorrectionConfig(se["heightmap_z_correction"], config.heightmap_z_correction);
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

void InitLkfMatrices(
    double dt,
    const KalmanFilterConfig& config,
    Eigen::MatrixXd& f,
    Eigen::MatrixXd& q,
    Eigen::MatrixXd& p0)
{
    f.setIdentity();
    f(0, 1) = dt;
    f(1, 2) = dt;
    f(3, 4) = dt;
    f(4, 5) = dt;
    f(6, 7) = dt;
    f(7, 8) = dt;
    f(0, 2) = 0.5 * dt * dt;
    f(3, 5) = 0.5 * dt * dt;
    f(6, 8) = 0.5 * dt * dt;

    Eigen::MatrixXd qm(3, 3);
    qm << std::pow(dt, 4) / 4.0, std::pow(dt, 3) / 2.0, std::pow(dt, 2) / 2.0,
          std::pow(dt, 3) / 2.0, std::pow(dt, 2),       dt,
          std::pow(dt, 2) / 2.0, dt,                    1.0;
    q.setIdentity();
    q.block(0, 0, 3, 3) = config.sigma_a * qm;
    q.block(3, 3, 3, 3) = config.sigma_a * qm;
    q.block(6, 6, 3, 3) = config.sigma_a * qm;

    p0.setIdentity();
    p0 *= config.p0;
}

class PositionKalmanFilter
{
public:
    PositionKalmanFilter(const Eigen::MatrixXd& f, const Eigen::MatrixXd& q)
        : f_(f),
          q_(q),
          identity_(Eigen::MatrixXd::Identity(f.rows(), f.cols())),
          x_(Eigen::VectorXd::Zero(f.rows())),
          p_(Eigen::MatrixXd::Zero(f.rows(), f.cols())),
          initialized_(false)
    {
    }

    void SetInitialEstimate(const Eigen::VectorXd& x0, const Eigen::MatrixXd& p0)
    {
        x_ = x0;
        p_ = p0;
        initialized_ = true;
    }

    bool initialized() const
    {
        return initialized_;
    }

    void Predict()
    {
        if (!initialized_)
        {
            throw std::runtime_error("[StateEstimatorMK] position Kalman filter is not initialized.");
        }
        x_ = f_ * x_;
        p_ = f_ * p_ * f_.transpose() + q_;
    }

    void Update(const Eigen::MatrixXd& h, const Eigen::VectorXd& z, const Eigen::VectorXd& r_diag)
    {
        if (!initialized_ || z.size() == 0)
        {
            return;
        }

        const Eigen::MatrixXd r = r_diag.asDiagonal();
        const Eigen::MatrixXd s = h * p_ * h.transpose() + r;
        const Eigen::MatrixXd k = p_ * h.transpose() * s.inverse();
        x_ += k * (z - h * x_);
        p_ = (identity_ - k * h) * p_ * (identity_ - k * h).transpose() + k * r * k.transpose();
    }

    const Eigen::VectorXd& state() const
    {
        return x_;
    }

    Eigen::Vector3d position() const
    {
        return Eigen::Vector3d{x_(0), x_(3), x_(6)};
    }

private:
    Eigen::MatrixXd f_;
    Eigen::MatrixXd q_;
    Eigen::MatrixXd identity_;
    Eigen::VectorXd x_;
    Eigen::MatrixXd p_;
    bool initialized_;
};

struct KinematicPositionMeasurement
{
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    std::array<bool, 3> axis_valid{false, false, false};
    int support_count{0};
};

struct LinearMeasurement
{
    Eigen::RowVectorXd h;
    double z{0.0};
    double r{1.0};
};

void AddStateMeasurement(
    std::vector<LinearMeasurement>& measurements,
    int state_index,
    double value,
    double noise)
{
    LinearMeasurement measurement;
    measurement.h = Eigen::RowVectorXd::Zero(kLkfStateSize);
    measurement.h(state_index) = 1.0;
    measurement.z = value;
    measurement.r = noise;
    measurements.push_back(std::move(measurement));
}

void BuildMeasurementMatrices(
    const std::vector<LinearMeasurement>& measurements,
    Eigen::MatrixXd& h,
    Eigen::VectorXd& z,
    Eigen::VectorXd& r_diag)
{
    h.resize(static_cast<int>(measurements.size()), kLkfStateSize);
    z.resize(static_cast<int>(measurements.size()));
    r_diag.resize(static_cast<int>(measurements.size()));

    for (int i = 0; i < static_cast<int>(measurements.size()); ++i)
    {
        h.row(i) = measurements[static_cast<size_t>(i)].h;
        z(i) = measurements[static_cast<size_t>(i)].z;
        r_diag(i) = measurements[static_cast<size_t>(i)].r;
    }
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

Eigen::Vector3d EstimateLegOdometryVelocity(
    Robot& robot,
    const RobotData& body_state,
    const ServoData& servo_state,
    const Eigen::Matrix3d& world_R_body,
    const std::vector<bool>& support_mask,
    bool has_gait_phase,
    const Eigen::Vector3d& fallback_velocity)
{
    if (!has_gait_phase)
    {
        return fallback_velocity;
    }

    Eigen::VectorXd q = Eigen::VectorXd::Zero(19);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(18);
    q.segment(0, 3) = body_state.pos;
    q.segment(3, 4) = body_state.orientation_quaternion;
    q.segment(7, 3) = servo_state.pos.segment(3, 3);
    q.segment(10, 3) = servo_state.pos.segment(9, 3);
    q.segment(13, 3) = servo_state.pos.segment(0, 3);
    q.segment(16, 3) = servo_state.pos.segment(6, 3);

    v.segment(0, 3) = world_R_body.transpose() * body_state.lin_vel;
    v.segment(3, 3) = body_state.ang_vel;
    v.segment(6, 3) = servo_state.vel.segment(3, 3);
    v.segment(9, 3) = servo_state.vel.segment(9, 3);
    v.segment(12, 3) = servo_state.vel.segment(0, 3);
    v.segment(15, 3) = servo_state.vel.segment(6, 3);

    robot.ComputeForwardKinematics(q, v);
    const std::vector<Eigen::Vector3d> pin_leg_positions = robot.GetToePositionsInBaseFrame();
    const std::vector<Eigen::Matrix3d> pin_leg_jacobians = robot.GetFootJacobian(q);

    std::vector<Eigen::Vector3d> leg_positions(NUM_LEGS);
    std::vector<Eigen::Matrix3d> leg_jacobians(NUM_LEGS);
    leg_positions[R1] = pin_leg_positions[PIN_R1];
    leg_positions[L1] = pin_leg_positions[PIN_L1];
    leg_positions[R2] = pin_leg_positions[PIN_R2];
    leg_positions[L2] = pin_leg_positions[PIN_L2];
    leg_jacobians[R1] = pin_leg_jacobians[PIN_R1];
    leg_jacobians[L1] = pin_leg_jacobians[PIN_L1];
    leg_jacobians[R2] = pin_leg_jacobians[PIN_R2];
    leg_jacobians[L2] = pin_leg_jacobians[PIN_L2];

    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    int support_count = 0;
    for (int i = 0; i < NUM_LEGS; ++i)
    {
        if (!support_mask[i])
        {
            continue;
        }

        const Eigen::Vector3d joint_velocity = servo_state.vel.segment(3 * i, 3);
        const Eigen::Vector3d foot_velocity_body = leg_jacobians[i] * joint_velocity;
        const Eigen::Vector3d angular_velocity_body = body_state.ang_vel.cross(leg_positions[i]);
        velocity += world_R_body * (-(foot_velocity_body + angular_velocity_body));
        ++support_count;
    }

    if (support_count == 0)
    {
        return fallback_velocity;
    }
    return velocity / static_cast<double>(support_count);
}

class KinematicPositionEstimator
{
public:
    KinematicPositionEstimator()
    {
        robot_.BuildPinocchioModel();
        q_ = Eigen::VectorXd::Zero(robot_.nq);
        v_ = Eigen::VectorXd::Zero(robot_.nv);
        q_(6) = 1.0;
        foot_anchors_.assign(NUM_LEGS, Eigen::Vector3d::Zero());
        anchor_valid_.assign(NUM_LEGS, false);
    }

    KinematicPositionMeasurement Estimate(
        const Eigen::Vector3d& reference_body_position,
        const Eigen::Vector3d& predicted_body_position,
        const Eigen::Matrix3d& world_R_body,
        const ServoData& servo_state,
        const std::vector<bool>& support_mask,
        const KinematicPositionConfig& config)
    {
        KinematicPositionMeasurement measurement;
        if (!config.enabled || servo_state.pos.size() < 12)
        {
            ResetLostContacts(support_mask);
            return measurement;
        }

        const std::vector<Eigen::Vector3d> foot_positions_base =
            GetFootPositionsInBaseFrame(servo_state);

        Eigen::Vector3d summed_position = Eigen::Vector3d::Zero();
        int support_count = 0;

        for (int leg = 0; leg < NUM_LEGS; ++leg)
        {
            const bool support = leg < static_cast<int>(support_mask.size()) && support_mask[leg];
            if (!support)
            {
                anchor_valid_[leg] = false;
                continue;
            }

            const Eigen::Vector3d foot_world_offset = world_R_body * foot_positions_base[leg];
            if (!anchor_valid_[leg])
            {
                foot_anchors_[leg] = reference_body_position + foot_world_offset;
                anchor_valid_[leg] = true;
            }

            summed_position += foot_anchors_[leg] - foot_world_offset;
            ++support_count;
        }

        measurement.support_count = support_count;
        if (support_count < config.min_support_legs)
        {
            return measurement;
        }

        measurement.position = summed_position / static_cast<double>(support_count);
        for (int axis = 0; axis < 3; ++axis)
        {
            if (!config.use_axis[axis])
            {
                continue;
            }

            const double innovation =
                std::fabs(measurement.position(axis) - predicted_body_position(axis));
            measurement.axis_valid[axis] =
                std::isfinite(measurement.position(axis)) &&
                innovation <= config.max_innovation(axis);
        }
        return measurement;
    }

private:
    void ResetLostContacts(const std::vector<bool>& support_mask)
    {
        for (int leg = 0; leg < NUM_LEGS; ++leg)
        {
            const bool support = leg < static_cast<int>(support_mask.size()) && support_mask[leg];
            if (!support)
            {
                anchor_valid_[leg] = false;
            }
        }
    }

    std::vector<Eigen::Vector3d> GetFootPositionsInBaseFrame(const ServoData& servo_state)
    {
        q_.setZero();
        v_.setZero();
        q_(6) = 1.0;
        q_.segment(7, 3) = servo_state.pos.segment(3, 3);
        q_.segment(10, 3) = servo_state.pos.segment(9, 3);
        q_.segment(13, 3) = servo_state.pos.segment(0, 3);
        q_.segment(16, 3) = servo_state.pos.segment(6, 3);

        robot_.ComputeForwardKinematics(q_, v_);
        const std::vector<Eigen::Vector3d> pin_leg_positions = robot_.GetToePositionsInBaseFrame();

        std::vector<Eigen::Vector3d> leg_positions(NUM_LEGS);
        leg_positions[R1] = pin_leg_positions[PIN_R1];
        leg_positions[L1] = pin_leg_positions[PIN_L1];
        leg_positions[R2] = pin_leg_positions[PIN_R2];
        leg_positions[L2] = pin_leg_positions[PIN_L2];
        return leg_positions;
    }

    Robot robot_;
    Eigen::VectorXd q_;
    Eigen::VectorXd v_;
    std::vector<Eigen::Vector3d> foot_anchors_;
    std::vector<bool> anchor_valid_;
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

Eigen::Vector3d GetLegPositionWorld(const LegData& leg_state, int leg)
{
    switch (leg)
    {
        case R1:
            return leg_state.r1_pos;
        case L1:
            return leg_state.l1_pos;
        case R2:
            return leg_state.r2_pos;
        case L2:
            return leg_state.l2_pos;
        default:
            return Eigen::Vector3d::Zero();
    }
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

class StateEstimatorMKApp
{
public:
    StateEstimatorMKApp()
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
            throw std::runtime_error("[StateEstimatorMK] failed to initialize one or more LCM endpoints.");
        }

        shared_inputs_.servo.pos = Eigen::VectorXd::Zero(12);
        shared_inputs_.servo.vel = Eigen::VectorXd::Zero(12);
        shared_inputs_.servo.torq = Eigen::VectorXd::Zero(12);

        std::cout << "[StateEstimatorMK] loaded configs from " << config_dir_ << "\n"
                  << "  state dt: " << se_config_.dt << " sec\n"
                  << "  D435i: " << d435i_config_.width << "x" << d435i_config_.height
                  << "@" << d435i_config_.fps << "\n"
                  << "  depth/state sync max dt: " << depth_config_.max_sync_dt_sec << " sec"
                  << std::endl;
    }

    void Run()
    {
        running_.store(true);
        threads_.emplace_back(&StateEstimatorMKApp::ImuLcmThread, this);
        threads_.emplace_back(&StateEstimatorMKApp::ServoLcmThread, this);
        threads_.emplace_back(&StateEstimatorMKApp::GaitPhaseLcmThread, this);
        threads_.emplace_back(&StateEstimatorMKApp::PoseCameraThread, this);
        threads_.emplace_back(&StateEstimatorMKApp::StateEstimatorThread, this);
        threads_.emplace_back(&StateEstimatorMKApp::DepthCameraThread, this);
        threads_.emplace_back(&StateEstimatorMKApp::HeightMapThread, this);

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
        shared_inputs_.has_gait_phase = true;
    }

    void ImuLcmThread()
    {
        imu_lcm_->subscribe(channels_.imu_data, &StateEstimatorMKApp::ImuHandler, this);
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
        servo_lcm_->subscribe(channels_.servo_state, &StateEstimatorMKApp::ServoHandler, this);
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
        gait_phase_lcm_->subscribe(channels_.gait_phase, &StateEstimatorMKApp::GaitPhaseHandler, this);
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

            std::cout << "[StateEstimatorMK] T265 pose stream started"
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
                shared_inputs_.has_odometry = true;
            }
        }
        catch (const rs2::error& e)
        {
            std::cerr << "[StateEstimatorMK] T265 RealSense error in "
                      << e.get_failed_function() << "(" << e.get_failed_args()
                      << "): " << e.what() << std::endl;
            running_.store(false);
        }
        catch (const std::exception& e)
        {
            std::cerr << "[StateEstimatorMK] T265 fatal: " << e.what() << std::endl;
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
            RobotData kalman_robot_state;
            LegData raw_leg_state;
            LegData kalman_leg_state;
            LegState raw_leg_state_estimator(robot_params_);
            LegState kalman_leg_state_estimator(robot_params_);

            Eigen::VectorXd p(3);
            p << 0.032, -0.01, 0.001;
            Eigen::VectorXd initial_theta = Eigen::VectorXd::Zero(12);
            raw_leg_state_estimator.set_grf_observer_params(100.0, se_config_.dt, p, initial_theta);
            raw_leg_state_estimator.set_contact_threshold(se_config_.contact_threshold);
            kalman_leg_state_estimator.set_grf_observer_params(100.0, se_config_.dt, p, initial_theta);
            kalman_leg_state_estimator.set_contact_threshold(se_config_.contact_threshold);

            Robot leg_odom_robot;
            leg_odom_robot.BuildPinocchioModel();
            KinematicPositionEstimator kinematic_position_estimator;

            Eigen::MatrixXd f_lkf(kLkfStateSize, kLkfStateSize);
            Eigen::MatrixXd q_lkf(kLkfStateSize, kLkfStateSize);
            Eigen::MatrixXd p0_lkf(kLkfStateSize, kLkfStateSize);
            InitLkfMatrices(
                se_config_.dt,
                se_config_.kalman_filter,
                f_lkf,
                q_lkf,
                p0_lkf);

            Eigen::VectorXd x_lkf = Eigen::VectorXd::Zero(kLkfStateSize);
            x_lkf(6) = se_config_.kalman_filter.initial_z;
            PositionKalmanFilter position_filter(f_lkf, q_lkf);

            const Eigen::Vector3d gravity(0.0, 0.0, robot_params_.g);
            const Eigen::Vector3d initial_body_position(0.0, 0.0, 0.038);

            bool first_pos = true;
            bool first_yaw = true;
            bool waiting_for_inputs_logged = false;
            Eigen::Vector3d pos_offset = Eigen::Vector3d::Zero();
            double yaw_offset = 0.0;
            int64_t last_used_heightmap_z_timestamp_ns = 0;

            const auto period = std::chrono::duration<double>(se_config_.dt);
            auto next_tick = std::chrono::steady_clock::now();

            std::cout << "[StateEstimatorMK] state estimator loop started" << std::endl;

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
                        std::cout << "[StateEstimatorMK] waiting for initial IMU, servo and T265 data"
                                  << std::endl;
                        waiting_for_inputs_logged = true;
                    }
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
                const bool has_support =
                    std::any_of(gait_support_mask.begin(), gait_support_mask.end(), [](bool support)
                    {
                        return support;
                    });
                const Eigen::Vector3d leg_odom_velocity = EstimateLegOdometryVelocity(
                    leg_odom_robot,
                    raw_robot_state,
                    servo_state,
                    r_body,
                    gait_support_mask,
                    inputs.has_gait_phase,
                    raw_robot_state.lin_vel);
                const Eigen::Vector3d imu_accel_corrected =
                    ClipImuAcceleration(
                        r_body * inputs.imu.lin_accel - gravity,
                        se_config_.kalman_filter.imu_accel_clip_mps2);

                if (!position_filter.initialized() && has_support)
                {
                    x_lkf << raw_robot_state.pos(0), raw_robot_state.lin_vel(0), imu_accel_corrected(0),
                             raw_robot_state.pos(1), raw_robot_state.lin_vel(1), imu_accel_corrected(1),
                             raw_robot_state.pos(2), raw_robot_state.lin_vel(2), imu_accel_corrected(2);
                    position_filter.SetInitialEstimate(x_lkf, p0_lkf);
                }

                Eigen::Vector3d predicted_position = raw_robot_state.pos;
                if (position_filter.initialized())
                {
                    position_filter.Predict();
                    predicted_position = position_filter.position();
                }

                bool use_heightmap_z_measurement = false;
                HeightMapZCorrection heightmap_z_correction;
                if (position_filter.initialized() && se_config_.heightmap_z_correction.enabled)
                {
                    heightmap_z_correction = GetLatestHeightMapZCorrection();
                    const double correction_age_sec =
                        std::fabs(static_cast<double>(NowNs() - heightmap_z_correction.timestamp_ns)) * 1e-9;
                    const double innovation =
                        std::fabs(heightmap_z_correction.body_z_measurement - predicted_position(Z));
                    use_heightmap_z_measurement =
                        heightmap_z_correction.valid &&
                        heightmap_z_correction.timestamp_ns > last_used_heightmap_z_timestamp_ns &&
                        correction_age_sec <= se_config_.heightmap_z_correction.max_age_sec &&
                        heightmap_z_correction.support_count >=
                            se_config_.heightmap_z_correction.min_support_legs &&
                        innovation <= se_config_.heightmap_z_correction.max_innovation;
                }

                const std::vector<bool> contact_support_mask =
                    BuildContactSupportMask(raw_leg_state.contacts);
                const std::vector<bool> kinematic_support_mask =
                    SelectKinematicSupportMask(
                        gait_support_mask,
                        contact_support_mask,
                        se_config_.kinematic_position.contact_source);
                const KinematicPositionMeasurement kinematic_position =
                    kinematic_position_estimator.Estimate(
                        predicted_position,
                        predicted_position,
                        r_body,
                        servo_state,
                        kinematic_support_mask,
                        se_config_.kinematic_position);

                if (position_filter.initialized())
                {
                    std::vector<LinearMeasurement> measurements;
                    measurements.reserve(kLkfMeasurementSize + 3);
                    for (int axis = 0; axis < 3; ++axis)
                    {
                        const int state_offset = 3 * axis;
                        const int measurement_offset = kLkfMeasurementsPerAxis * axis;
                        AddStateMeasurement(
                            measurements,
                            state_offset,
                            raw_robot_state.pos(axis),
                            se_config_.kalman_filter.sigma_z(measurement_offset));
                        AddStateMeasurement(
                            measurements,
                            state_offset + 1,
                            raw_robot_state.lin_vel(axis),
                            se_config_.kalman_filter.sigma_z(measurement_offset + 1));
                        AddStateMeasurement(
                            measurements,
                            state_offset + 1,
                            leg_odom_velocity(axis),
                            se_config_.kalman_filter.sigma_z(measurement_offset + 2));
                        AddStateMeasurement(
                            measurements,
                            state_offset + 2,
                            imu_accel_corrected(axis),
                            se_config_.kalman_filter.sigma_z(measurement_offset + 3));

                        if (kinematic_position.axis_valid[axis])
                        {
                            AddStateMeasurement(
                                measurements,
                                state_offset,
                                kinematic_position.position(axis),
                                se_config_.kinematic_position.sigma(axis));
                        }
                        if (axis == Z && use_heightmap_z_measurement)
                        {
                            AddStateMeasurement(
                                measurements,
                                state_offset,
                                heightmap_z_correction.body_z_measurement,
                                se_config_.heightmap_z_correction.sigma);
                        }
                    }

                    Eigen::MatrixXd h_lkf;
                    Eigen::VectorXd z_lkf;
                    Eigen::VectorXd r_lkf_diag;
                    BuildMeasurementMatrices(measurements, h_lkf, z_lkf, r_lkf_diag);
                    position_filter.Update(h_lkf, z_lkf, r_lkf_diag);
                    x_lkf = position_filter.state();
                    if (use_heightmap_z_measurement)
                    {
                        last_used_heightmap_z_timestamp_ns = heightmap_z_correction.timestamp_ns;
                    }
                }

                kalman_robot_state = raw_robot_state;
                if (position_filter.initialized())
                {
                    kalman_robot_state.pos << x_lkf(0), x_lkf(3), x_lkf(6);
                    kalman_robot_state.lin_vel << x_lkf(1), x_lkf(4), x_lkf(7);
                }

                kalman_leg_state = kalman_leg_state_estimator.get_leg_state(
                    kalman_robot_state,
                    servo_state.pos,
                    servo_state.vel,
                    servo_state.torq);

                RobotData robot_state_to_publish = raw_robot_state;
                LegData leg_state_to_publish = raw_leg_state;
                if (position_filter.initialized())
                {
                    robot_state_to_publish.pos(Z) = kalman_robot_state.pos(Z);
                    robot_state_to_publish.lin_vel(Z) = kalman_robot_state.lin_vel(Z);
                    leg_state_to_publish = kalman_leg_state_estimator.get_leg_state(
                        robot_state_to_publish,
                        servo_state.pos,
                        servo_state.vel,
                        servo_state.torq);
                }

                const int64_t timestamp_ns = NowNs();
                PublishRobotState(channels_.robot_state, timestamp_ns, robot_state_to_publish, leg_state_to_publish);
                PublishRobotState(kRobotStateCheckChannel, timestamp_ns, kalman_robot_state, kalman_leg_state);
                PushFootSupportSnapshot(
                    timestamp_ns,
                    robot_state_to_publish,
                    leg_state_to_publish,
                    kinematic_support_mask);
                PushRobotStateSnapshot(timestamp_ns, robot_state_to_publish);

                std::this_thread::sleep_until(next_tick);
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "[StateEstimatorMK] state estimator fatal: " << e.what() << std::endl;
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

            std::cout << "[StateEstimatorMK] D435i depth stream started"
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
            std::cerr << "[StateEstimatorMK] D435i RealSense error in "
                      << e.get_failed_function() << "(" << e.get_failed_args()
                      << "): " << e.what()
                      << ". Heightmap updates disabled; state estimation continues."
                      << std::endl;
            depth_cv_.notify_all();
        }
        catch (const std::exception& e)
        {
            std::cerr << "[StateEstimatorMK] D435i fatal: " << e.what()
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
            std::cerr << "[StateEstimatorMK] RealSense stop error in "
                      << e.get_failed_function() << "(" << e.get_failed_args()
                      << "): " << e.what() << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cerr << "[StateEstimatorMK] RealSense stop fatal: " << e.what() << std::endl;
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
            throw std::runtime_error("[StateEstimatorMK] RealSense pointcloud returned null vertices.");
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

        std::cout << "[StateEstimatorMK] heightmap loop started" << std::endl;

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
                    std::cerr << "[StateEstimatorMK] skipped depth frames by sync: "
                              << skipped_sync << ", last dt=" << dt_sec << " sec" << std::endl;
                }
                continue;
            }

            FootSupportSnapshot foot_snapshot;
            const bool has_foot_snapshot =
                FindNearestFootSupportSnapshot(frame.timestamp_ns, &foot_snapshot);
            if (has_foot_snapshot)
            {
                const double foot_dt_sec =
                    std::fabs(static_cast<double>(frame.timestamp_ns - foot_snapshot.timestamp_ns)) * 1e-9;
                if (foot_dt_sec <= depth_config_.max_sync_dt_sec)
                {
                    UpdateHeightMapZCorrectionFromSupport(frame.timestamp_ns, foot_snapshot);
                }
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

    void PushFootSupportSnapshot(
        int64_t timestamp_ns,
        const RobotData& robot_state,
        const LegData& leg_state,
        const std::vector<bool>& support_mask)
    {
        FootSupportSnapshot snapshot;
        snapshot.valid = true;
        snapshot.timestamp_ns = timestamp_ns;
        snapshot.body_z = robot_state.pos(Z);
        for (int leg = 0; leg < NUM_LEGS; ++leg)
        {
            const Eigen::Vector3d foot_position = GetLegPositionWorld(leg_state, leg);
            snapshot.foot_position_world[static_cast<size_t>(leg)] = {
                foot_position(X),
                foot_position(Y),
                foot_position(Z)};
            snapshot.support[static_cast<size_t>(leg)] =
                leg < static_cast<int>(support_mask.size()) && support_mask[static_cast<size_t>(leg)];
        }

        std::lock_guard<std::mutex> lock(foot_support_history_mutex_);
        foot_support_history_.push_back(snapshot);
        while (foot_support_history_.size() > kFootSupportHistoryCapacity)
        {
            foot_support_history_.pop_front();
        }
    }

    bool FindNearestFootSupportSnapshot(int64_t timestamp_ns, FootSupportSnapshot* out) const
    {
        if (out == nullptr)
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(foot_support_history_mutex_);
        if (foot_support_history_.empty())
        {
            return false;
        }

        auto best = foot_support_history_.begin();
        int64_t best_dt = std::llabs(best->timestamp_ns - timestamp_ns);
        for (auto it = std::next(foot_support_history_.begin()); it != foot_support_history_.end(); ++it)
        {
            const int64_t dt = std::llabs(it->timestamp_ns - timestamp_ns);
            if (dt < best_dt)
            {
                best = it;
                best_dt = dt;
            }
        }

        *out = *best;
        return out->valid;
    }

    void UpdateHeightMapZCorrectionFromSupport(
        int64_t timestamp_ns,
        const FootSupportSnapshot& support_snapshot)
    {
        if (!se_config_.heightmap_z_correction.enabled || !heightmap_builder_)
        {
            return;
        }

        double sum_body_z_measurement = 0.0;
        int support_count = 0;
        for (int leg = 0; leg < NUM_LEGS; ++leg)
        {
            if (!support_snapshot.support[static_cast<size_t>(leg)])
            {
                continue;
            }

            const std::array<double, 3>& foot =
                support_snapshot.foot_position_world[static_cast<size_t>(leg)];
            double map_height = 0.0;
            if (!heightmap_builder_->EstimateHeightAtWorldXY(
                    foot[X],
                    foot[Y],
                    se_config_.heightmap_z_correction.map_radius_cells,
                    &map_height))
            {
                continue;
            }

            const double body_z_measurement = support_snapshot.body_z + (map_height - foot[Z]);
            if (!std::isfinite(body_z_measurement))
            {
                continue;
            }

            sum_body_z_measurement += body_z_measurement;
            ++support_count;
        }

        if (support_count < se_config_.heightmap_z_correction.min_support_legs)
        {
            return;
        }

        HeightMapZCorrection correction;
        correction.valid = true;
        correction.timestamp_ns = timestamp_ns;
        correction.body_z_measurement =
            sum_body_z_measurement / static_cast<double>(support_count);
        correction.support_count = support_count;

        {
            std::lock_guard<std::mutex> lock(heightmap_z_correction_mutex_);
            latest_heightmap_z_correction_ = correction;
        }

        if (d435i_config_.verbose)
        {
            std::cout << "[StateEstimatorMK] heightmap Z correction: z="
                      << correction.body_z_measurement
                      << " support_count=" << correction.support_count << std::endl;
        }
    }

    HeightMapZCorrection GetLatestHeightMapZCorrection() const
    {
        std::lock_guard<std::mutex> lock(heightmap_z_correction_mutex_);
        return latest_heightmap_z_correction_;
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

    mutable std::mutex foot_support_history_mutex_;
    std::deque<FootSupportSnapshot> foot_support_history_;

    mutable std::mutex heightmap_z_correction_mutex_;
    HeightMapZCorrection latest_heightmap_z_correction_;

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

        StateEstimatorMKApp app;
        app.Run();
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[StateEstimatorMK] fatal: " << e.what() << std::endl;
        return 1;
    }
}
