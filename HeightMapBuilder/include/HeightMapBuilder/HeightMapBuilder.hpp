#ifndef HEIGHT_MAP_BUILDER_HEIGHT_MAP_BUILDER_HPP_
#define HEIGHT_MAP_BUILDER_HEIGHT_MAP_BUILDER_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <lcm/lcm-cpp.hpp>

#include "mors_msgs/depth_image_msg.hpp"
#include "mors_msgs/pointcloud_msg.hpp"
#include "mors_msgs/robot_state_msg.hpp"

namespace hmb
{

struct ChannelsConfig
{
    std::string depth_image{"DEPTH_IMAGE"};
    std::string robot_state{"ROBOT_STATE"};
    std::string pointcloud{"POINCLOUD"};
};

struct CameraConfig
{
    int width{424};
    int height{240};
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    double fovy_deg{58.0};
};

struct DepthConfig
{
    double min_depth{0.10};
    double max_depth{3.00};
    bool enable_downsampling{true};
    int downsample_factor{2};
    bool remove_outliers{true};
    int outlier_window_radius{1};
    double outlier_threshold_m{0.12};
    int min_neighbors_for_outlier{3};
};

struct SyncConfig
{
    double max_sync_dt_sec{0.05};
    bool require_recent_robot_state{true};
};

struct RuntimeConfig
{
    bool verbose{false};
};

struct HeightMapBuilderConfig
{
    ChannelsConfig channels;
    CameraConfig camera;
    DepthConfig depth;
    SyncConfig sync;
    RuntimeConfig runtime;
    bool use_camera_pose_params{true};
    std::array<double, 3> camera_frame_position_body_m{0.2565, 0.0, 0.0};
    std::array<double, 3> camera_frame_euler_deg{0.0, 25.0, 90.0};
    std::array<double, 3> depth_camera_position_camera_frame_m{0.0125, 0.0, 0.0};
    std::array<double, 3> depth_camera_euler_deg{-90.0, 0.0, 180.0};
    std::array<double, 16> t_body_camera{
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };
};

struct RobotStateSnapshot
{
    bool valid{false};
    int64_t receive_timestamp_ns{0};
    std::array<double, 3> position{0.0, 0.0, 0.0};
    std::array<double, 3> orientation_rpy{0.0, 0.0, 0.0}; // roll pitch yaw
    std::array<double, 4> orientation_quaternion{0.0, 0.0, 0.0, 1.0}; // x y z w
    double yaw{0.0};
};

class HeightMapBuilderNode
{
public:
    explicit HeightMapBuilderNode(const std::string& config_path);
    int Run();

private:
    static int64_t NowNs();
    static double ComputeYawFromQuaternion(double qx, double qy, double qz, double qw);
    static std::array<double, 9> RotationFromEuler(double roll, double pitch, double yaw);
    static std::array<double, 9> RotationFromQuaternion(double qx, double qy, double qz, double qw);
    static std::array<double, 3> Mat3MulVec3(
        const std::array<double, 9>& m,
        const std::array<double, 3>& v);
    static std::array<double, 9> Mat3MulMat3(
        const std::array<double, 9>& a,
        const std::array<double, 9>& b);
    static double DegToRad(double degrees);

    bool LoadConfig(const std::string& config_path);
    void ApplyIntrinsicsFallback();
    bool DecodeDepthImage(const mors_msgs::depth_image_msg& msg, std::vector<uint16_t>* depth_mm) const;
    bool IsValidDepth(double depth_m) const;
    bool IsOutlier(
        const std::vector<uint16_t>& depth_mm,
        int width,
        int height,
        int u,
        int v,
        double depth_m) const;
    void BuildPointCloudCameraFrame(
        const std::vector<uint16_t>& depth_mm,
        int width,
        int height,
        std::vector<float>* out_x,
        std::vector<float>* out_y,
        std::vector<float>* out_z) const;
    void TransformPointCloudCameraToWorld(
        const std::vector<float>& cam_x,
        const std::vector<float>& cam_y,
        const std::vector<float>& cam_z,
        std::vector<float>* world_x,
        std::vector<float>* world_y,
        std::vector<float>* world_z) const;
    void PublishPointCloud(
        const mors_msgs::depth_image_msg& depth_msg,
        const std::vector<float>& x,
        const std::vector<float>& y,
        const std::vector<float>& z);

    void OnDepthImage(
        const lcm::ReceiveBuffer* rbuf,
        const std::string& chan,
        const mors_msgs::depth_image_msg* msg);
    void OnRobotState(
        const lcm::ReceiveBuffer* rbuf,
        const std::string& chan,
        const mors_msgs::robot_state_msg* msg);

    HeightMapBuilderConfig config_;
    RobotStateSnapshot latest_robot_state_;
    lcm::LCM lcm_;
    uint64_t frame_counter_{0};
};

}  // namespace hmb

#endif  // HEIGHT_MAP_BUILDER_HEIGHT_MAP_BUILDER_HPP_
