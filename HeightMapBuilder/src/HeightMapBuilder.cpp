#include "HeightMapBuilder/HeightMapBuilder.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>

#include <yaml-cpp/yaml.h>
#include <zlib.h>

namespace
{

constexpr double kMillimetersToMeters = 1e-3;
constexpr double kSecondsToNanoseconds = 1e9;

}  // namespace

namespace hmb
{

HeightMapBuilderNode::HeightMapBuilderNode(const std::string& config_path)
{
    if (!lcm_.good())
    {
        throw std::runtime_error("[HeightMapBuilder] LCM initialization failed.");
    }

    if (!LoadConfig(config_path))
    {
        throw std::runtime_error("[HeightMapBuilder] Failed to load config: " + config_path);
    }

    lcm_.subscribe(config_.channels.depth_image, &HeightMapBuilderNode::OnDepthImage, this);
    lcm_.subscribe(config_.channels.robot_state, &HeightMapBuilderNode::OnRobotState, this);

    std::cout << "[HeightMapBuilder] started\n"
              << "  depth channel: " << config_.channels.depth_image << "\n"
              << "  robot_state channel: " << config_.channels.robot_state << "\n"
              << "  pointcloud channel: " << config_.channels.pointcloud << std::endl;
}

int HeightMapBuilderNode::Run()
{
    while (true)
    {
        const int status = lcm_.handleTimeout(100);
        if (status < 0)
        {
            std::cerr << "[HeightMapBuilder] LCM handleTimeout failed." << std::endl;
            return 1;
        }
    }
}

int64_t HeightMapBuilderNode::NowNs()
{
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

double HeightMapBuilderNode::ComputeYawFromQuaternion(double qx, double qy, double qz, double qw)
{
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

std::array<double, 9> HeightMapBuilderNode::RotationFromEuler(
    double roll, double pitch, double yaw)
{
    std::array<double, 9> r{};

    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    // R_world_body (active rotation, xyz / roll-pitch-yaw)
    r[0] = cy * cp;
    r[1] = cy * sp * sr - sy * cr;
    r[2] = cy * sp * cr + sy * sr;

    r[3] = sy * cp;
    r[4] = sy * sp * sr + cy * cr;
    r[5] = sy * sp * cr - cy * sr;

    r[6] = -sp;
    r[7] = cp * sr;
    r[8] = cp * cr;

    return r;
}

std::array<double, 9> HeightMapBuilderNode::RotationFromQuaternion(
    double qx, double qy, double qz, double qw)
{
    const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (!(norm > 1e-12) || !std::isfinite(norm))
    {
        return std::array<double, 9>{
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };
    }

    const double x = qx / norm;
    const double y = qy / norm;
    const double z = qz / norm;
    const double w = qw / norm;

    // Matches project-wide convention in common/src/system_functions.cpp (quat2mat)
    return std::array<double, 9>{
        -1.0 + 2.0 * (w * w) + 2.0 * (x * x), 2.0 * (x * y + w * z), 2.0 * (x * z - w * y),
        2.0 * (x * y - w * z), -1.0 + 2.0 * (w * w) + 2.0 * (y * y), 2.0 * (y * z + w * x),
        2.0 * (x * z + w * y), 2.0 * (y * z - w * x), -1.0 + 2.0 * (w * w) + 2.0 * (z * z)
    };
}

std::array<double, 3> HeightMapBuilderNode::Mat3MulVec3(
    const std::array<double, 9>& m,
    const std::array<double, 3>& v)
{
    return std::array<double, 3>{
        m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
        m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
        m[6] * v[0] + m[7] * v[1] + m[8] * v[2]
    };
}

std::array<double, 9> HeightMapBuilderNode::Mat3MulMat3(
    const std::array<double, 9>& a,
    const std::array<double, 9>& b)
{
    std::array<double, 9> c{};
    for (int r = 0; r < 3; ++r)
    {
        for (int col = 0; col < 3; ++col)
        {
            c[static_cast<size_t>(r * 3 + col)] =
                a[static_cast<size_t>(r * 3 + 0)] * b[static_cast<size_t>(0 * 3 + col)] +
                a[static_cast<size_t>(r * 3 + 1)] * b[static_cast<size_t>(1 * 3 + col)] +
                a[static_cast<size_t>(r * 3 + 2)] * b[static_cast<size_t>(2 * 3 + col)];
        }
    }
    return c;
}

double HeightMapBuilderNode::DegToRad(double degrees)
{
    return degrees * std::numbers::pi_v<double> / 180.0;
}

bool HeightMapBuilderNode::LoadConfig(const std::string& config_path)
{
    YAML::Node root;
    try
    {
        root = YAML::LoadFile(config_path);
    }
    catch (const std::exception& e)
    {
        std::cerr << "[HeightMapBuilder] Cannot read config '" << config_path
                  << "': " << e.what() << std::endl;
        return false;
    }

    if (const YAML::Node channels = root["channels"])
    {
        if (channels["depth_image"])
        {
            config_.channels.depth_image = channels["depth_image"].as<std::string>();
        }
        if (channels["robot_state"])
        {
            config_.channels.robot_state = channels["robot_state"].as<std::string>();
        }
        if (channels["pointcloud"])
        {
            config_.channels.pointcloud = channels["pointcloud"].as<std::string>();
        }
    }

    if (const YAML::Node camera = root["camera"])
    {
        if (camera["width"])
        {
            config_.camera.width = std::max(1, camera["width"].as<int>());
        }
        if (camera["height"])
        {
            config_.camera.height = std::max(1, camera["height"].as<int>());
        }
        if (camera["fx"])
        {
            config_.camera.fx = camera["fx"].as<double>();
        }
        if (camera["fy"])
        {
            config_.camera.fy = camera["fy"].as<double>();
        }
        if (camera["cx"])
        {
            config_.camera.cx = camera["cx"].as<double>();
        }
        if (camera["cy"])
        {
            config_.camera.cy = camera["cy"].as<double>();
        }
        if (camera["fovy_deg"])
        {
            config_.camera.fovy_deg = camera["fovy_deg"].as<double>();
        }
    }

    if (const YAML::Node depth = root["depth"])
    {
        if (depth["min_depth"])
        {
            config_.depth.min_depth = depth["min_depth"].as<double>();
        }
        if (depth["max_depth"])
        {
            config_.depth.max_depth = depth["max_depth"].as<double>();
        }
        if (depth["enable_downsampling"])
        {
            config_.depth.enable_downsampling = depth["enable_downsampling"].as<bool>();
        }
        if (depth["downsample_factor"])
        {
            config_.depth.downsample_factor = std::max(1, depth["downsample_factor"].as<int>());
        }
        if (depth["remove_outliers"])
        {
            config_.depth.remove_outliers = depth["remove_outliers"].as<bool>();
        }
        if (depth["outlier_window_radius"])
        {
            config_.depth.outlier_window_radius = std::max(1, depth["outlier_window_radius"].as<int>());
        }
        if (depth["outlier_threshold_m"])
        {
            config_.depth.outlier_threshold_m = std::max(0.0, depth["outlier_threshold_m"].as<double>());
        }
        if (depth["min_neighbors_for_outlier"])
        {
            config_.depth.min_neighbors_for_outlier = std::max(1, depth["min_neighbors_for_outlier"].as<int>());
        }
    }

    if (const YAML::Node sync = root["sync"])
    {
        if (sync["max_sync_dt_sec"])
        {
            config_.sync.max_sync_dt_sec = std::max(0.0, sync["max_sync_dt_sec"].as<double>());
        }
        if (sync["require_recent_robot_state"])
        {
            config_.sync.require_recent_robot_state = sync["require_recent_robot_state"].as<bool>();
        }
    }

    if (const YAML::Node runtime = root["runtime"])
    {
        if (runtime["verbose"])
        {
            config_.runtime.verbose = runtime["verbose"].as<bool>();
        }
    }

    if (const YAML::Node transforms = root["transforms"])
    {
        if (transforms["use_camera_pose_params"])
        {
            config_.use_camera_pose_params = transforms["use_camera_pose_params"].as<bool>();
        }

        const auto parse_vec3 = [](const YAML::Node& node, std::array<double, 3>* out) -> bool
        {
            if (!node || !node.IsSequence() || node.size() != 3 || out == nullptr)
            {
                return false;
            }
            for (size_t i = 0; i < 3; ++i)
            {
                (*out)[i] = node[i].as<double>();
            }
            return true;
        };

        if (const YAML::Node camera_frame = transforms["camera_frame"])
        {
            if (camera_frame["position_body_m"] &&
                !parse_vec3(camera_frame["position_body_m"], &config_.camera_frame_position_body_m))
            {
                std::cerr << "[HeightMapBuilder] transforms.camera_frame.position_body_m must contain 3 values."
                          << std::endl;
            }
            if (camera_frame["euler_deg"] &&
                !parse_vec3(camera_frame["euler_deg"], &config_.camera_frame_euler_deg))
            {
                std::cerr << "[HeightMapBuilder] transforms.camera_frame.euler_deg must contain 3 values."
                          << std::endl;
            }
        }

        if (const YAML::Node depth_camera = transforms["depth_camera"])
        {
            if (depth_camera["position_camera_frame_m"] &&
                !parse_vec3(depth_camera["position_camera_frame_m"],
                            &config_.depth_camera_position_camera_frame_m))
            {
                std::cerr
                    << "[HeightMapBuilder] transforms.depth_camera.position_camera_frame_m must contain 3 values."
                    << std::endl;
            }
            if (depth_camera["euler_deg"] &&
                !parse_vec3(depth_camera["euler_deg"], &config_.depth_camera_euler_deg))
            {
                std::cerr << "[HeightMapBuilder] transforms.depth_camera.euler_deg must contain 3 values."
                          << std::endl;
            }
        }

        if (const YAML::Node t_body_camera = transforms["T_body_camera"])
        {
            if (!t_body_camera.IsSequence() || t_body_camera.size() != 16)
            {
                std::cerr << "[HeightMapBuilder] transforms.T_body_camera must contain 16 values. "
                             "Identity will be used."
                          << std::endl;
            }
            else
            {
                for (size_t i = 0; i < 16; ++i)
                {
                    config_.t_body_camera[i] = t_body_camera[i].as<double>();
                }
            }
        }
    }

    if (config_.use_camera_pose_params)
    {
        const std::array<double, 9> r_body_camera_frame = RotationFromEuler(
            DegToRad(config_.camera_frame_euler_deg[0]),
            DegToRad(config_.camera_frame_euler_deg[1]),
            DegToRad(config_.camera_frame_euler_deg[2]));
        const std::array<double, 9> r_camera_frame_depth = RotationFromEuler(
            DegToRad(config_.depth_camera_euler_deg[0]),
            DegToRad(config_.depth_camera_euler_deg[1]),
            DegToRad(config_.depth_camera_euler_deg[2]));

        const std::array<double, 9> r_body_camera =
            Mat3MulMat3(r_body_camera_frame, r_camera_frame_depth);
        const std::array<double, 3> rotated_offset = Mat3MulVec3(
            r_body_camera_frame, config_.depth_camera_position_camera_frame_m);
        const std::array<double, 3> t_body_camera{
            config_.camera_frame_position_body_m[0] + rotated_offset[0],
            config_.camera_frame_position_body_m[1] + rotated_offset[1],
            config_.camera_frame_position_body_m[2] + rotated_offset[2]
        };

        config_.t_body_camera = {
            r_body_camera[0], r_body_camera[1], r_body_camera[2], t_body_camera[0],
            r_body_camera[3], r_body_camera[4], r_body_camera[5], t_body_camera[1],
            r_body_camera[6], r_body_camera[7], r_body_camera[8], t_body_camera[2],
            0.0,             0.0,             0.0,             1.0
        };
    }

    if (config_.depth.min_depth >= config_.depth.max_depth)
    {
        std::cerr << "[HeightMapBuilder] Invalid depth limits: min_depth >= max_depth." << std::endl;
        return false;
    }

    ApplyIntrinsicsFallback();
    return true;
}

void HeightMapBuilderNode::ApplyIntrinsicsFallback()
{
    if (config_.camera.fx <= 0.0 || config_.camera.fy <= 0.0)
    {
        const double fovy_rad = config_.camera.fovy_deg * std::numbers::pi_v<double> / 180.0;
        const double fy = static_cast<double>(config_.camera.height) / (2.0 * std::tan(0.5 * fovy_rad));
        config_.camera.fy = fy;
        config_.camera.fx = fy;
    }

    if (config_.camera.cx <= 0.0)
    {
        config_.camera.cx = (static_cast<double>(config_.camera.width) - 1.0) * 0.5;
    }
    if (config_.camera.cy <= 0.0)
    {
        config_.camera.cy = (static_cast<double>(config_.camera.height) - 1.0) * 0.5;
    }
}

bool HeightMapBuilderNode::DecodeDepthImage(
    const mors_msgs::depth_image_msg& msg, std::vector<uint16_t>* depth_mm) const
{
    if (depth_mm == nullptr)
    {
        return false;
    }
    if (msg.width <= 0 || msg.height <= 0)
    {
        std::cerr << "[HeightMapBuilder] Invalid depth dimensions: "
                  << msg.width << "x" << msg.height << std::endl;
        return false;
    }

    const uint64_t expected_bytes_u64 =
        static_cast<uint64_t>(msg.width) * static_cast<uint64_t>(msg.height) * sizeof(uint16_t);
    if (expected_bytes_u64 > static_cast<uint64_t>(std::numeric_limits<size_t>::max()))
    {
        std::cerr << "[HeightMapBuilder] Depth frame is too large to allocate." << std::endl;
        return false;
    }
    const size_t expected_bytes = static_cast<size_t>(expected_bytes_u64);

    if (msg.compression == 0)
    {
        if (msg.data.size() != expected_bytes)
        {
            std::cerr << "[HeightMapBuilder] Raw depth payload size mismatch. expected="
                      << expected_bytes << " got=" << msg.data.size() << std::endl;
            return false;
        }
        depth_mm->resize(static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height));
        std::memcpy(depth_mm->data(), msg.data.data(), expected_bytes);
        return true;
    }

    if (msg.compression == 1)
    {
        std::vector<uint8_t> raw_bytes(expected_bytes);
        uLongf decoded_size = static_cast<uLongf>(raw_bytes.size());
        const int zret = ::uncompress(
            raw_bytes.data(),
            &decoded_size,
            reinterpret_cast<const Bytef*>(msg.data.data()),
            static_cast<uLongf>(msg.data.size()));
        if (zret != Z_OK)
        {
            std::cerr << "[HeightMapBuilder] zlib uncompress failed. code=" << zret << std::endl;
            return false;
        }
        if (static_cast<size_t>(decoded_size) != expected_bytes)
        {
            std::cerr << "[HeightMapBuilder] Decompressed depth payload size mismatch. expected="
                      << expected_bytes << " got=" << decoded_size << std::endl;
            return false;
        }
        depth_mm->resize(static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height));
        std::memcpy(depth_mm->data(), raw_bytes.data(), expected_bytes);
        return true;
    }

    std::cerr << "[HeightMapBuilder] Unsupported depth compression type: "
              << static_cast<int>(msg.compression) << std::endl;
    return false;
}

bool HeightMapBuilderNode::IsValidDepth(double depth_m) const
{
    return std::isfinite(depth_m) &&
           depth_m >= config_.depth.min_depth &&
           depth_m <= config_.depth.max_depth;
}

bool HeightMapBuilderNode::IsOutlier(
    const std::vector<uint16_t>& depth_mm,
    int width,
    int height,
    int u,
    int v,
    double depth_m) const
{
    if (!config_.depth.remove_outliers)
    {
        return false;
    }

    const int r = config_.depth.outlier_window_radius;
    std::vector<double> neighbors;
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
            const uint16_t raw_depth = depth_mm[static_cast<size_t>(vv) * static_cast<size_t>(width) + static_cast<size_t>(uu)];
            if (raw_depth == 0u)
            {
                continue;
            }

            const double neighbor_depth_m = static_cast<double>(raw_depth) * kMillimetersToMeters;
            if (!IsValidDepth(neighbor_depth_m))
            {
                continue;
            }
            neighbors.push_back(neighbor_depth_m);
        }
    }

    if (static_cast<int>(neighbors.size()) < config_.depth.min_neighbors_for_outlier)
    {
        return false;
    }

    auto middle_it = neighbors.begin() + (neighbors.size() / 2u);
    std::nth_element(neighbors.begin(), middle_it, neighbors.end());
    const double median = *middle_it;
    return std::fabs(depth_m - median) > config_.depth.outlier_threshold_m;
}

void HeightMapBuilderNode::BuildPointCloudCameraFrame(
    const std::vector<uint16_t>& depth_mm,
    int width,
    int height,
    std::vector<float>* out_x,
    std::vector<float>* out_y,
    std::vector<float>* out_z) const
{
    if (out_x == nullptr || out_y == nullptr || out_z == nullptr)
    {
        return;
    }

    out_x->clear();
    out_y->clear();
    out_z->clear();

    const int downsample =
        config_.depth.enable_downsampling ? std::max(1, config_.depth.downsample_factor) : 1;

    const size_t approx_points =
        (static_cast<size_t>(width) / static_cast<size_t>(downsample) + 1u) *
        (static_cast<size_t>(height) / static_cast<size_t>(downsample) + 1u);
    out_x->reserve(approx_points);
    out_y->reserve(approx_points);
    out_z->reserve(approx_points);

    for (int v = 0; v < height; v += downsample)
    {
        for (int u = 0; u < width; u += downsample)
        {
            const uint16_t raw_depth = depth_mm[static_cast<size_t>(v) * static_cast<size_t>(width) + static_cast<size_t>(u)];
            if (raw_depth == 0u)
            {
                continue;
            }

            const double depth_m = static_cast<double>(raw_depth) * kMillimetersToMeters;
            if (!IsValidDepth(depth_m))
            {
                continue;
            }
            if (IsOutlier(depth_mm, width, height, u, v, depth_m))
            {
                continue;
            }

            const double x = (static_cast<double>(u) - config_.camera.cx) * depth_m / config_.camera.fx;
            const double y = (static_cast<double>(v) - config_.camera.cy) * depth_m / config_.camera.fy;

            out_x->push_back(static_cast<float>(x));
            out_y->push_back(static_cast<float>(y));
            out_z->push_back(static_cast<float>(depth_m));
        }
    }
}

void HeightMapBuilderNode::TransformPointCloudCameraToWorld(
    const std::vector<float>& cam_x,
    const std::vector<float>& cam_y,
    const std::vector<float>& cam_z,
    std::vector<float>* world_x,
    std::vector<float>* world_y,
    std::vector<float>* world_z) const
{
    if (world_x == nullptr || world_y == nullptr || world_z == nullptr)
    {
        return;
    }
    if (cam_x.size() != cam_y.size() || cam_x.size() != cam_z.size())
    {
        world_x->clear();
        world_y->clear();
        world_z->clear();
        return;
    }

    const std::array<double, 9> r_body_camera{
        config_.t_body_camera[0], config_.t_body_camera[1], config_.t_body_camera[2],
        config_.t_body_camera[4], config_.t_body_camera[5], config_.t_body_camera[6],
        config_.t_body_camera[8], config_.t_body_camera[9], config_.t_body_camera[10]
    };
    const std::array<double, 3> t_body_camera{
        config_.t_body_camera[3], config_.t_body_camera[7], config_.t_body_camera[11]
    };

    std::array<double, 9> r_world_body{};
    const bool euler_valid =
        std::isfinite(latest_robot_state_.orientation_rpy[0]) &&
        std::isfinite(latest_robot_state_.orientation_rpy[1]) &&
        std::isfinite(latest_robot_state_.orientation_rpy[2]);
    if (euler_valid)
    {
        r_world_body = RotationFromEuler(
            latest_robot_state_.orientation_rpy[0],
            latest_robot_state_.orientation_rpy[1],
            latest_robot_state_.orientation_rpy[2]);
    }
    else
    {
        r_world_body = RotationFromQuaternion(
            latest_robot_state_.orientation_quaternion[0],
            latest_robot_state_.orientation_quaternion[1],
            latest_robot_state_.orientation_quaternion[2],
            latest_robot_state_.orientation_quaternion[3]);
    }

    const std::array<double, 3> t_world_body{
        latest_robot_state_.position[0],
        latest_robot_state_.position[1],
        latest_robot_state_.position[2]
    };

    world_x->resize(cam_x.size());
    world_y->resize(cam_x.size());
    world_z->resize(cam_x.size());

    for (size_t i = 0; i < cam_x.size(); ++i)
    {
        const std::array<double, 3> p_camera{
            static_cast<double>(cam_x[i]),
            static_cast<double>(cam_y[i]),
            static_cast<double>(cam_z[i])
        };
        std::array<double, 3> p_body = Mat3MulVec3(r_body_camera, p_camera);
        p_body[0] += t_body_camera[0];
        p_body[1] += t_body_camera[1];
        p_body[2] += t_body_camera[2];

        std::array<double, 3> p_world = Mat3MulVec3(r_world_body, p_body);
        p_world[0] += t_world_body[0];
        p_world[1] += t_world_body[1];
        p_world[2] += t_world_body[2];

        (*world_x)[i] = static_cast<float>(p_world[0]);
        (*world_y)[i] = static_cast<float>(p_world[1]);
        (*world_z)[i] = static_cast<float>(p_world[2]);
    }
}

void HeightMapBuilderNode::PublishPointCloud(
    const mors_msgs::depth_image_msg& depth_msg,
    const std::vector<float>& x,
    const std::vector<float>& y,
    const std::vector<float>& z)
{
    if (x.size() != y.size() || x.size() != z.size())
    {
        std::cerr << "[HeightMapBuilder] Internal pointcloud vector size mismatch." << std::endl;
        return;
    }

    mors_msgs::pointcloud_msg pointcloud_msg;
    pointcloud_msg.timestamp = NowNs();
    pointcloud_msg.depth_timestamp = depth_msg.timestamp;
    pointcloud_msg.robot_state_rx_timestamp = latest_robot_state_.receive_timestamp_ns;

    for (size_t i = 0; i < 3; ++i)
    {
        pointcloud_msg.robot_position[i] = static_cast<float>(latest_robot_state_.position[i]);
    }
    for (size_t i = 0; i < 4; ++i)
    {
        pointcloud_msg.robot_orientation_quaternion[i] =
            static_cast<float>(latest_robot_state_.orientation_quaternion[i]);
    }
    pointcloud_msg.robot_yaw = static_cast<float>(latest_robot_state_.yaw);

    pointcloud_msg.points_count = static_cast<int32_t>(x.size());
    pointcloud_msg.x = x;
    pointcloud_msg.y = y;
    pointcloud_msg.z = z;

    lcm_.publish(config_.channels.pointcloud, &pointcloud_msg);
}

void HeightMapBuilderNode::OnDepthImage(
    const lcm::ReceiveBuffer* /*rbuf*/,
    const std::string& /*chan*/,
    const mors_msgs::depth_image_msg* msg)
{
    if (msg == nullptr)
    {
        return;
    }

    if (config_.sync.require_recent_robot_state && !latest_robot_state_.valid)
    {
        return;
    }

    const int64_t depth_ts = (msg->timestamp > 0) ? msg->timestamp : NowNs();
    if (latest_robot_state_.valid)
    {
        const int64_t dt_ns = std::llabs(depth_ts - latest_robot_state_.receive_timestamp_ns);
        const double dt_sec = static_cast<double>(dt_ns) / kSecondsToNanoseconds;
        if (dt_sec > config_.sync.max_sync_dt_sec && config_.sync.require_recent_robot_state)
        {
            return;
        }
    }

    std::vector<uint16_t> depth_mm;
    if (!DecodeDepthImage(*msg, &depth_mm))
    {
        return;
    }

    std::vector<float> points_x;
    std::vector<float> points_y;
    std::vector<float> points_z;
    BuildPointCloudCameraFrame(depth_mm, msg->width, msg->height, &points_x, &points_y, &points_z);

    std::vector<float> points_world_x;
    std::vector<float> points_world_y;
    std::vector<float> points_world_z;
    TransformPointCloudCameraToWorld(
        points_x,
        points_y,
        points_z,
        &points_world_x,
        &points_world_y,
        &points_world_z);
    PublishPointCloud(*msg, points_world_x, points_world_y, points_world_z);

    ++frame_counter_;
    if (config_.runtime.verbose && frame_counter_ % 30u == 0u)
    {
        std::cout << "[HeightMapBuilder] Frame " << frame_counter_
                  << ", points: " << points_x.size() << std::endl;
    }
}

void HeightMapBuilderNode::OnRobotState(
    const lcm::ReceiveBuffer* /*rbuf*/,
    const std::string& /*chan*/,
    const mors_msgs::robot_state_msg* msg)
{
    if (msg == nullptr)
    {
        return;
    }

    latest_robot_state_.receive_timestamp_ns = NowNs();

    for (size_t i = 0; i < 3; ++i)
    {
        latest_robot_state_.position[i] = msg->body.position[i];
        latest_robot_state_.orientation_rpy[i] = msg->body.orientation[i];
    }
    for (size_t i = 0; i < 4; ++i)
    {
        latest_robot_state_.orientation_quaternion[i] = msg->body.orientation_quaternion[i];
    }

    const double yaw_from_euler = msg->body.orientation[2];
    if (std::isfinite(yaw_from_euler))
    {
        latest_robot_state_.yaw = yaw_from_euler;
    }
    else
    {
        latest_robot_state_.yaw = ComputeYawFromQuaternion(
            latest_robot_state_.orientation_quaternion[0],
            latest_robot_state_.orientation_quaternion[1],
            latest_robot_state_.orientation_quaternion[2],
            latest_robot_state_.orientation_quaternion[3]);
    }

    latest_robot_state_.valid = true;
}

}  // namespace hmb
