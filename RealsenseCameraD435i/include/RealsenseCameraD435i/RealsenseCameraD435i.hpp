#ifndef REALSENSE_CAMERA_D435I_REALSENSE_CAMERA_D435I_HPP_
#define REALSENSE_CAMERA_D435I_REALSENSE_CAMERA_D435I_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <librealsense2/rs.hpp>
#include <lcm/lcm-cpp.hpp>

#include "mors_msgs/depth_image_msg.hpp"

namespace rscam
{

struct ChannelsConfig
{
    std::string depth_image{"DEPTH_IMAGE"};
};

struct StreamConfig
{
    std::string serial;
    int width{424};
    int height{240};
    int fps{30};
    double publish_fps{0.0};
};

struct DepthConfig
{
    double min_depth_m{0.105};
    double max_depth_m{3.0};
};

struct CompressionConfig
{
    bool enabled{true};
    int zlib_level{1};
};

struct RuntimeConfig
{
    bool verbose{false};
};

struct RealsenseCameraD435iConfig
{
    ChannelsConfig channels;
    StreamConfig stream;
    DepthConfig depth;
    CompressionConfig compression;
    RuntimeConfig runtime;
};

class RealsenseCameraD435iNode
{
public:
    explicit RealsenseCameraD435iNode(const std::string& config_path);
    int Run();

private:
    static int64_t NowNs();

    bool LoadConfig(const std::string& config_path);
    void ValidateConfig() const;
    void StartCamera();
    void PublishDepthFrame(const rs2::depth_frame& frame);
    std::vector<uint8_t> BuildDepthPayload(const rs2::depth_frame& frame) const;
    std::vector<uint8_t> CompressPayload(const std::vector<uint8_t>& raw_payload) const;

    RealsenseCameraD435iConfig config_;
    std::string lcm_url_;
    lcm::LCM lcm_;
    rs2::pipeline pipeline_;
    rs2::pipeline_profile pipeline_profile_;
    double depth_scale_m_{0.001};
    uint64_t frame_counter_{0};
    std::chrono::steady_clock::time_point next_publish_time_{};
};

}  // namespace rscam

#endif  // REALSENSE_CAMERA_D435I_REALSENSE_CAMERA_D435I_HPP_
