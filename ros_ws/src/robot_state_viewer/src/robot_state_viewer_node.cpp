#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <lcm/lcm-cpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>
#include <zlib.h>

#include "mors_msgs/depth_image_msg.hpp"
#include "mors_msgs/heightmap_msg.hpp"
#include "mors_msgs/pointcloud_msg.hpp"
#include "mors_msgs/robot_state_msg.hpp"
#include "mors_msgs/servo_state_msg.hpp"

namespace robot_state_viewer
{

class RobotStateViewerNode : public rclcpp::Node
{
public:
  RobotStateViewerNode()
  : Node("robot_state_viewer"), running_(true)
  {
    world_frame_id_ = this->declare_parameter<std::string>("world_frame_id", "map");
    base_link_frame_id_ = this->declare_parameter<std::string>("base_link_frame_id", "base_link");

    depth_lcm_channel_ = this->declare_parameter<std::string>("lcm_channel", "DEPTH_IMAGE");
    depth_ros_topic_ = this->declare_parameter<std::string>("ros_topic", "/camera/depth_image");
    pointcloud_lcm_channel_ =
      this->declare_parameter<std::string>("pointcloud_lcm_channel", "POINCLOUD");
    pointcloud_ros_topic_ =
      this->declare_parameter<std::string>("pointcloud_ros_topic", "/camera/poincloud");
    robot_state_lcm_channel_ =
      this->declare_parameter<std::string>("robot_state_lcm_channel", "ROBOT_STATE");
    servo_state_lcm_channel_ =
      this->declare_parameter<std::string>("servo_state_lcm_channel", "SERVO_STATE");

    frame_id_ = this->declare_parameter<std::string>("frame_id", "camera_depth_optical_frame");
    pointcloud_frame_id_ =
      this->declare_parameter<std::string>("pointcloud_frame_id", world_frame_id_);
    joint_states_topic_ = this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
    heightmap_ros_topic_ =
      this->declare_parameter<std::string>("heightmap_ros_topic", "/heightmap/pointcloud");
    heightmap_config_path_ =
      this->declare_parameter<std::string>("heightmap_config_path", "");
    loadHeightmapDecodeConfig(heightmap_config_path_);

    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      depth_ros_topic_, rclcpp::QoS(10).reliable());
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      pointcloud_ros_topic_, rclcpp::QoS(10).reliable());
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::QoS(20).reliable());
    heightmap_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      heightmap_ros_topic_, rclcpp::QoS(10).reliable());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    if (!lcm_.good()) {
      throw std::runtime_error("Failed to initialize LCM");
    }

    lcm_.subscribe(depth_lcm_channel_, &RobotStateViewerNode::depthImageHandler, this);
    lcm_.subscribe(pointcloud_lcm_channel_, &RobotStateViewerNode::pointCloudHandler, this);
    lcm_.subscribe(robot_state_lcm_channel_, &RobotStateViewerNode::robotStateHandler, this);
    lcm_.subscribe(servo_state_lcm_channel_, &RobotStateViewerNode::servoStateHandler, this);
    lcm_.subscribe(heightmap_lcm_channel_, &RobotStateViewerNode::heightmapHandler, this);
    lcm_thread_ = std::thread(&RobotStateViewerNode::lcmLoop, this);

    joint_names_ = {
      "abad_joint_R1", "hip_joint_R1", "knee_joint_R1",
      "abad_joint_L1", "hip_joint_L1", "knee_joint_L1",
      "abad_joint_R2", "hip_joint_R2", "knee_joint_R2",
      "abad_joint_L2", "hip_joint_L2", "knee_joint_L2"
    };

    RCLCPP_INFO(
      this->get_logger(),
      "robot_state_viewer started:"
      " depth LCM='%s' -> ROS='%s', pointcloud LCM='%s' -> ROS='%s',"
      " heightmap LCM='%s' -> ROS='%s', robot_state LCM='%s', servo_state LCM='%s',"
      " tf('%s'->'%s'), cloud_frame='%s', joint_states='%s',"
      " heightmap_cfg='%s', heightmap_grid=%dx%d cell=%.4f hmin=%.3f hres=%.4f",
      depth_lcm_channel_.c_str(),
      depth_ros_topic_.c_str(),
      pointcloud_lcm_channel_.c_str(),
      pointcloud_ros_topic_.c_str(),
      heightmap_lcm_channel_.c_str(),
      heightmap_ros_topic_.c_str(),
      robot_state_lcm_channel_.c_str(),
      servo_state_lcm_channel_.c_str(),
      world_frame_id_.c_str(),
      base_link_frame_id_.c_str(),
      pointcloud_frame_id_.c_str(),
      joint_states_topic_.c_str(),
      heightmap_config_resolved_path_.c_str(),
      heightmap_window_cells_x_,
      heightmap_window_cells_y_,
      heightmap_cell_size_,
      heightmap_height_min_,
      heightmap_height_resolution_);
  }

  ~RobotStateViewerNode() override
  {
    running_.store(false);
    if (lcm_thread_.joinable()) {
      lcm_thread_.join();
    }
  }

private:
  static bool quaternionValid(double x, double y, double z, double w)
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(w)) {
      return false;
    }
    const double n = std::sqrt(x * x + y * y + z * z + w * w);
    return n > 1e-9;
  }

  static std::array<double, 4> eulerToQuaternion(double roll, double pitch, double yaw)
  {
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);

    const double qw = cr * cp * cy + sr * sp * sy;
    const double qx = sr * cp * cy - cr * sp * sy;
    const double qy = cr * sp * cy + sr * cp * sy;
    const double qz = cr * cp * sy - sr * sp * cy;
    return {qx, qy, qz, qw};
  }

  static std::string resolveHeightmapConfigPath(const std::string & configured_path)
  {
    namespace fs = std::filesystem;

    if (!configured_path.empty()) {
      if (fs::exists(configured_path)) {
        return configured_path;
      }
      throw std::runtime_error(
              "heightmap_config_path does not exist: " + configured_path);
    }

    std::vector<std::string> candidates;
    const char * config_dir = std::getenv("CONFIGPATH");
    if (config_dir != nullptr && config_dir[0] != '\0') {
      candidates.emplace_back(std::string(config_dir) + "/heightmap_builder.yaml");
    }
    candidates.emplace_back("config/heightmap_builder.yaml");
    candidates.emplace_back("../config/heightmap_builder.yaml");
    candidates.emplace_back("../../config/heightmap_builder.yaml");
    candidates.emplace_back("../../../config/heightmap_builder.yaml");
    candidates.emplace_back("../../../../config/heightmap_builder.yaml");

    for (const auto & path : candidates) {
      if (fs::exists(path)) {
        return path;
      }
    }

    throw std::runtime_error(
            "Cannot locate heightmap_builder.yaml. "
            "Set parameter 'heightmap_config_path' or environment CONFIGPATH.");
  }

  void loadHeightmapDecodeConfig(const std::string & configured_path)
  {
    const std::string resolved_path = resolveHeightmapConfigPath(configured_path);

    YAML::Node root;
    try {
      root = YAML::LoadFile(resolved_path);
    } catch (const std::exception & e) {
      throw std::runtime_error(
              "Failed to load heightmap config '" + resolved_path + "': " + e.what());
    }

    const YAML::Node channels = root["channels"];
    if (!channels || !channels["heightmap"]) {
      throw std::runtime_error(
              "heightmap config '" + resolved_path +
              "' missing required key: channels.heightmap");
    }
    heightmap_lcm_channel_ = channels["heightmap"].as<std::string>();
    if (heightmap_lcm_channel_.empty()) {
      throw std::runtime_error(
              "heightmap config '" + resolved_path +
              "': channels.heightmap must be non-empty");
    }

    const YAML::Node map = root["map"];
    if (!map) {
      throw std::runtime_error(
              "heightmap config '" + resolved_path + "' missing required key: map");
    }
    if (!map["local_window_cells_x"] || !map["local_window_cells_y"] ||
      !map["cell_size"] || !map["height_min"] || !map["height_resolution"])
    {
      throw std::runtime_error(
              "heightmap config '" + resolved_path +
              "' missing one of required keys: map.local_window_cells_x, "
              "map.local_window_cells_y, map.cell_size, map.height_min, map.height_resolution");
    }

    heightmap_window_cells_x_ = map["local_window_cells_x"].as<int>();
    heightmap_window_cells_y_ = map["local_window_cells_y"].as<int>();
    heightmap_cell_size_ = map["cell_size"].as<double>();
    heightmap_height_min_ = map["height_min"].as<double>();
    heightmap_height_resolution_ = map["height_resolution"].as<double>();

    if (heightmap_window_cells_x_ <= 0 || heightmap_window_cells_y_ <= 0) {
      throw std::runtime_error(
              "Invalid map.local_window_cells_x/y in '" + resolved_path + "'");
    }
    if (!(heightmap_cell_size_ > 0.0)) {
      throw std::runtime_error("Invalid map.cell_size in '" + resolved_path + "'");
    }
    if (!(heightmap_height_resolution_ > 0.0)) {
      throw std::runtime_error(
              "Invalid map.height_resolution in '" + resolved_path + "'");
    }

    heightmap_config_resolved_path_ = resolved_path;
  }

  void lcmLoop()
  {
    while (rclcpp::ok() && running_.load()) {
      lcm_.handleTimeout(100);
    }
  }

  void depthImageHandler(
    const lcm::ReceiveBuffer * /*rbuf*/,
    const std::string & /*channel*/,
    const mors_msgs::depth_image_msg * msg)
  {
    if (msg == nullptr) {
      return;
    }

    if (msg->width <= 0 || msg->height <= 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Invalid depth frame size: width=%d height=%d", msg->width, msg->height);
      return;
    }

    const uint64_t expected_size_u64 = static_cast<uint64_t>(msg->width) *
      static_cast<uint64_t>(msg->height) * sizeof(uint16_t);
    if (expected_size_u64 > static_cast<uint64_t>(std::numeric_limits<size_t>::max())) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Depth frame is too large to allocate: width=%d height=%d", msg->width, msg->height);
      return;
    }
    const auto expected_size = static_cast<size_t>(expected_size_u64);

    const auto & payload = msg->data;
    std::vector<uint8_t> raw_bytes;

    if (msg->compression == 0) {
      if (payload.size() != expected_size) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Raw depth payload size mismatch: expected=%zu got=%zu",
          expected_size, payload.size());
        return;
      }
      raw_bytes = payload;
    } else if (msg->compression == 1) {
      raw_bytes.resize(expected_size);
      uLongf dst_len = static_cast<uLongf>(raw_bytes.size());
      const int zret = ::uncompress(
        raw_bytes.data(), &dst_len,
        payload.data(), static_cast<uLongf>(payload.size()));
      if (zret != Z_OK) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Failed to decompress depth payload, zlib error code=%d", zret);
        return;
      }
      if (static_cast<size_t>(dst_len) != expected_size) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Decompressed depth size mismatch: expected=%zu got=%zu",
          expected_size, static_cast<size_t>(dst_len));
        return;
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Unsupported depth compression type=%d", msg->compression);
      return;
    }

    sensor_msgs::msg::Image image_msg;
    image_msg.header.frame_id = frame_id_;
    image_msg.header.stamp = this->now();

    if (msg->timestamp > 0) {
      const int64_t sec64 = msg->timestamp / 1000000000LL;
      const int64_t nsec64 = msg->timestamp % 1000000000LL;
      if (sec64 >= std::numeric_limits<int32_t>::min() &&
        sec64 <= std::numeric_limits<int32_t>::max())
      {
        image_msg.header.stamp.sec = static_cast<int32_t>(sec64);
        image_msg.header.stamp.nanosec = static_cast<uint32_t>(nsec64);
      }
    }

    image_msg.height = static_cast<uint32_t>(msg->height);
    image_msg.width = static_cast<uint32_t>(msg->width);
    image_msg.encoding = "16UC1";
    image_msg.is_bigendian = 0;
    image_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(
      static_cast<uint32_t>(msg->width * static_cast<int32_t>(sizeof(uint16_t))));
    image_msg.data = std::move(raw_bytes);

    depth_pub_->publish(std::move(image_msg));
  }

  void pointCloudHandler(
    const lcm::ReceiveBuffer * /*rbuf*/,
    const std::string & /*channel*/,
    const mors_msgs::pointcloud_msg * msg)
  {
    if (msg == nullptr) {
      return;
    }

    if (msg->points_count < 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Invalid points_count=%d", msg->points_count);
      return;
    }

    const size_t expected_points = static_cast<size_t>(msg->points_count);
    if (msg->x.size() < expected_points ||
      msg->y.size() < expected_points ||
      msg->z.size() < expected_points)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Pointcloud arrays smaller than points_count (count=%zu, x=%zu, y=%zu, z=%zu)",
        expected_points, msg->x.size(), msg->y.size(), msg->z.size());
      return;
    }

    const uint64_t points_count_u64 = static_cast<uint64_t>(expected_points);
    const uint64_t data_size_u64 = points_count_u64 * 3ULL * sizeof(float);
    if (data_size_u64 > static_cast<uint64_t>(std::numeric_limits<size_t>::max())) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Pointcloud is too large to allocate: points=%zu", expected_points);
      return;
    }
    const size_t data_size = static_cast<size_t>(data_size_u64);

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = pointcloud_frame_id_;
    cloud_msg.header.stamp = this->now();

    if (msg->timestamp > 0) {
      const int64_t sec64 = msg->timestamp / 1000000000LL;
      const int64_t nsec64 = msg->timestamp % 1000000000LL;
      if (sec64 >= std::numeric_limits<int32_t>::min() &&
        sec64 <= std::numeric_limits<int32_t>::max())
      {
        cloud_msg.header.stamp.sec = static_cast<int32_t>(sec64);
        cloud_msg.header.stamp.nanosec = static_cast<uint32_t>(nsec64);
      }
    }

    cloud_msg.height = 1;
    cloud_msg.width = static_cast<uint32_t>(expected_points);
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = true;
    cloud_msg.point_step = static_cast<uint32_t>(3 * sizeof(float));
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;

    cloud_msg.fields.resize(3);
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[0].count = 1;

    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[1].count = 1;

    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[2].count = 1;

    cloud_msg.data.resize(data_size);
    uint8_t * dst = cloud_msg.data.data();
    for (size_t i = 0; i < expected_points; ++i) {
      std::memcpy(dst + i * cloud_msg.point_step + 0, &msg->x[i], sizeof(float));
      std::memcpy(dst + i * cloud_msg.point_step + 4, &msg->y[i], sizeof(float));
      std::memcpy(dst + i * cloud_msg.point_step + 8, &msg->z[i], sizeof(float));
    }

    pointcloud_pub_->publish(std::move(cloud_msg));
  }

  void heightmapHandler(
    const lcm::ReceiveBuffer * /*rbuf*/,
    const std::string & /*channel*/,
    const mors_msgs::heightmap_msg * msg)
  {
    if (msg == nullptr) {
      return;
    }

    const int64_t expected_size_i64 =
      static_cast<int64_t>(heightmap_window_cells_x_) * static_cast<int64_t>(heightmap_window_cells_y_);
    if (expected_size_i64 <= 0 || expected_size_i64 > static_cast<int64_t>(std::numeric_limits<int32_t>::max())) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Invalid configured heightmap window size: %d x %d",
        heightmap_window_cells_x_, heightmap_window_cells_y_);
      return;
    }

    const int32_t expected_size = static_cast<int32_t>(expected_size_i64);
    if (msg->data_size != expected_size) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "HEIGHTMAP data_size mismatch: expected=%d got=%d",
        expected_size, msg->data_size);
      return;
    }

    if (msg->data.size() < static_cast<size_t>(expected_size)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "HEIGHTMAP payload too small: expected=%d got=%zu",
        expected_size, msg->data.size());
      return;
    }

    size_t valid_points = 0;
    for (int i = 0; i < expected_size; ++i) {
      const uint16_t packed = static_cast<uint16_t>(msg->data[static_cast<size_t>(i)]);
      const bool valid = ((packed >> 15) & 0x1u) != 0u;
      if (valid) {
        ++valid_points;
      }
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = world_frame_id_;
    cloud_msg.header.stamp = this->now();
    cloud_msg.height = 1;
    cloud_msg.width = static_cast<uint32_t>(valid_points);
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = true;
    cloud_msg.point_step = static_cast<uint32_t>(4 * sizeof(float));  // x,y,z,intensity
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;

    cloud_msg.fields.resize(4);
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[0].count = 1;

    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[1].count = 1;

    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[2].count = 1;

    cloud_msg.fields[3].name = "intensity";
    cloud_msg.fields[3].offset = 12;
    cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[3].count = 1;

    cloud_msg.data.resize(static_cast<size_t>(cloud_msg.row_step));
    uint8_t * dst = cloud_msg.data.data();

    const double origin_x = static_cast<double>(msg->origin_x);
    const double origin_y = static_cast<double>(msg->origin_y);
    const double start_x =
      origin_x - 0.5 * static_cast<double>(heightmap_window_cells_x_) * heightmap_cell_size_;
    const double start_y =
      origin_y - 0.5 * static_cast<double>(heightmap_window_cells_y_) * heightmap_cell_size_;

    size_t out_idx = 0;
    for (int ly = 0; ly < heightmap_window_cells_y_; ++ly) {
      for (int lx = 0; lx < heightmap_window_cells_x_; ++lx) {
        const size_t in_idx = static_cast<size_t>(ly) * static_cast<size_t>(heightmap_window_cells_x_) +
          static_cast<size_t>(lx);
        const uint16_t packed = static_cast<uint16_t>(msg->data[in_idx]);
        const bool valid = ((packed >> 15) & 0x1u) != 0u;
        if (!valid) {
          continue;
        }

        const uint8_t traversability_class = static_cast<uint8_t>((packed >> 13) & 0x3u);
        const uint16_t height_q = static_cast<uint16_t>(packed & 0x1FFFu);

        const float x = static_cast<float>(
          start_x + (static_cast<double>(lx) + 0.5) * heightmap_cell_size_);
        const float y = static_cast<float>(
          start_y + (static_cast<double>(ly) + 0.5) * heightmap_cell_size_);
        const float z = static_cast<float>(
          heightmap_height_min_ + static_cast<double>(height_q) * heightmap_height_resolution_);
        const float intensity = static_cast<float>(traversability_class);

        std::memcpy(dst + out_idx * cloud_msg.point_step + 0, &x, sizeof(float));
        std::memcpy(dst + out_idx * cloud_msg.point_step + 4, &y, sizeof(float));
        std::memcpy(dst + out_idx * cloud_msg.point_step + 8, &z, sizeof(float));
        std::memcpy(dst + out_idx * cloud_msg.point_step + 12, &intensity, sizeof(float));
        ++out_idx;
      }
    }

    heightmap_pub_->publish(std::move(cloud_msg));
  }

  void servoStateHandler(
    const lcm::ReceiveBuffer * /*rbuf*/,
    const std::string & /*channel*/,
    const mors_msgs::servo_state_msg * msg)
  {
    if (msg == nullptr) {
      return;
    }

    for (size_t i = 0; i < joint_positions_.size(); ++i) {
      joint_positions_[i] = static_cast<double>(msg->position[i]);
    }
    have_servo_state_ = true;
  }

  void robotStateHandler(
    const lcm::ReceiveBuffer * /*rbuf*/,
    const std::string & /*channel*/,
    const mors_msgs::robot_state_msg * msg)
  {
    if (msg == nullptr) {
      return;
    }

    const rclcpp::Time stamp = this->now();

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = world_frame_id_;
    tf_msg.child_frame_id = base_link_frame_id_;
    tf_msg.transform.translation.x = msg->body.position[0];
    tf_msg.transform.translation.y = msg->body.position[1];
    tf_msg.transform.translation.z = msg->body.position[2];

    double qx = msg->body.orientation_quaternion[0];
    double qy = msg->body.orientation_quaternion[1];
    double qz = msg->body.orientation_quaternion[2];
    double qw = msg->body.orientation_quaternion[3];

    if (!quaternionValid(qx, qy, qz, qw)) {
      const auto q = eulerToQuaternion(
        msg->body.orientation[0],
        msg->body.orientation[1],
        msg->body.orientation[2]);
      qx = q[0];
      qy = q[1];
      qz = q[2];
      qw = q[3];
    } else {
      const double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
      qx /= n;
      qy /= n;
      qz /= n;
      qw /= n;
    }

    tf_msg.transform.rotation.x = qx;
    tf_msg.transform.rotation.y = qy;
    tf_msg.transform.rotation.z = qz;
    tf_msg.transform.rotation.w = qw;
    tf_broadcaster_->sendTransform(tf_msg);

    if (!have_servo_state_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "No SERVO_STATE received yet, publishing zero joint positions.");
    }

    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = stamp;
    joint_msg.name = joint_names_;
    joint_msg.position.assign(joint_positions_.begin(), joint_positions_.end());
    joint_state_pub_->publish(std::move(joint_msg));
  }

  std::string depth_lcm_channel_;
  std::string depth_ros_topic_;
  std::string pointcloud_lcm_channel_;
  std::string pointcloud_ros_topic_;
  std::string heightmap_lcm_channel_;
  std::string heightmap_ros_topic_;
  std::string heightmap_config_path_;
  std::string heightmap_config_resolved_path_;
  std::string robot_state_lcm_channel_;
  std::string servo_state_lcm_channel_;
  std::string frame_id_;
  std::string pointcloud_frame_id_;
  std::string world_frame_id_;
  std::string base_link_frame_id_;
  std::string joint_states_topic_;
  int heightmap_window_cells_x_{0};
  int heightmap_window_cells_y_{0};
  double heightmap_cell_size_{0.0};
  double heightmap_height_min_{0.0};
  double heightmap_height_resolution_{0.0};

  std::array<double, 12> joint_positions_{};
  std::vector<std::string> joint_names_;
  bool have_servo_state_{false};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr heightmap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  lcm::LCM lcm_;
  std::atomic<bool> running_;
  std::thread lcm_thread_;
};

}  // namespace robot_state_viewer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<robot_state_viewer::RobotStateViewerNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "robot_state_viewer fatal error: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
