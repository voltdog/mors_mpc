#include "RealsenseCameraD435i/RealsenseCameraD435i.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <zlib.h>

namespace
{

constexpr double kMetersToMillimeters = 1000.0;
constexpr int kRawDepthBytesPerPixel = 2;
constexpr int8_t kCompressionRawU16Mm = 0;
constexpr int8_t kCompressionZlibU16Mm = 1;
constexpr double kDisabledPublishThrottle = 0.0;
constexpr const char* kVisionLcmUrlEnv = "LCM_VISION_URL";

std::string GetRequiredEnv(const char* name)
{
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0')
    {
        throw std::runtime_error(std::string("[RealsenseCameraD435i] ") + name + " must be set.");
    }
    return value;
}

int ClampZlibLevel(int level)
{
    return std::clamp(level, Z_NO_COMPRESSION, Z_BEST_COMPRESSION);
}

std::string CameraInfoOrUnknown(const rs2::device& device, rs2_camera_info info)
{
    if (!device.supports(info))
    {
        return "unknown";
    }
    return device.get_info(info);
}

}  // namespace

namespace rscam
{

RealsenseCameraD435iNode::RealsenseCameraD435iNode(const std::string& config_path)
    : lcm_url_(GetRequiredEnv(kVisionLcmUrlEnv)),
      lcm_(lcm_url_)
{
    if (!lcm_.good())
    {
        throw std::runtime_error("[RealsenseCameraD435i] LCM initialization failed: " + lcm_url_);
    }

    std::cout << "[RealsenseCameraD435i] LCM vision URL: " << lcm_url_ << std::endl;

    if (!LoadConfig(config_path))
    {
        throw std::runtime_error("[RealsenseCameraD435i] Failed to load config: " + config_path);
    }

    ValidateConfig();

    std::cout << "[RealsenseCameraD435i] loaded config: serial="
              << (config_.stream.serial.empty() ? std::string("<auto>") : config_.stream.serial)
              << ", stream=" << config_.stream.width << "x" << config_.stream.height
              << "@" << config_.stream.fps
              << ", publish_fps=" << config_.stream.publish_fps
              << std::endl;

    StartCamera();

    std::cout << "[RealsenseCameraD435i] started\n"
              << "  depth channel: " << config_.channels.depth_image << "\n"
              << "  stream: " << config_.stream.width << "x" << config_.stream.height
              << "@" << config_.stream.fps << " Z16\n"
              << "  depth range: [" << config_.depth.min_depth_m << ", "
              << config_.depth.max_depth_m << "] m\n"
              << "  depth scale: " << depth_scale_m_ << " m/unit\n"
              << "  publish fps: "
              << (config_.stream.publish_fps > kDisabledPublishThrottle
                      ? std::to_string(config_.stream.publish_fps)
                      : std::string("camera fps"))
              << "\n"
              << "  compression: "
              << (config_.compression.enabled ? "zlib if smaller" : "disabled")
              << std::endl;
}

int RealsenseCameraD435iNode::Run()
{
    while (true)
    {
        const rs2::frameset frames = pipeline_.wait_for_frames();
        const rs2::depth_frame depth_frame = frames.get_depth_frame();
        if (!depth_frame)
        {
            continue;
        }

        if (config_.stream.publish_fps > kDisabledPublishThrottle &&
            config_.stream.publish_fps < static_cast<double>(config_.stream.fps))
        {
            const auto now = std::chrono::steady_clock::now();
            if (next_publish_time_.time_since_epoch().count() == 0)
            {
                next_publish_time_ = now;
            }
            if (now < next_publish_time_)
            {
                continue;
            }

            const auto publish_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / config_.stream.publish_fps));
            do
            {
                next_publish_time_ += publish_period;
            } while (next_publish_time_ <= now);
        }

        PublishDepthFrame(depth_frame);
    }
}

int64_t RealsenseCameraD435iNode::NowNs()
{
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

bool RealsenseCameraD435iNode::LoadConfig(const std::string& config_path)
{
    try
    {
        const YAML::Node root = YAML::LoadFile(config_path);

        if (const YAML::Node channels = root["channels"])
        {
            if (channels["depth_image"])
            {
                config_.channels.depth_image = channels["depth_image"].as<std::string>();
            }
        }

        if (const YAML::Node stream = root["stream"])
        {
            if (stream["serial"])
            {
                config_.stream.serial = stream["serial"].as<std::string>();
            }
            if (stream["width"])
            {
                config_.stream.width = stream["width"].as<int>();
            }
            if (stream["height"])
            {
                config_.stream.height = stream["height"].as<int>();
            }
            if (stream["fps"])
            {
                config_.stream.fps = stream["fps"].as<int>();
            }
            if (stream["publish_fps"])
            {
                config_.stream.publish_fps = stream["publish_fps"].as<double>();
            }
        }

        if (const YAML::Node depth = root["depth"])
        {
            if (depth["min_depth_m"])
            {
                config_.depth.min_depth_m = depth["min_depth_m"].as<double>();
            }
            if (depth["max_depth_m"])
            {
                config_.depth.max_depth_m = depth["max_depth_m"].as<double>();
            }
        }

        if (const YAML::Node compression = root["compression"])
        {
            if (compression["enabled"])
            {
                config_.compression.enabled = compression["enabled"].as<bool>();
            }
            if (compression["zlib_level"])
            {
                config_.compression.zlib_level = compression["zlib_level"].as<int>();
            }
        }

        if (const YAML::Node runtime = root["runtime"])
        {
            if (runtime["verbose"])
            {
                config_.runtime.verbose = runtime["verbose"].as<bool>();
            }
        }
    }
    catch (const YAML::Exception& e)
    {
        std::cerr << "[RealsenseCameraD435i] YAML error: " << e.what() << std::endl;
        return false;
    }

    return true;
}

void RealsenseCameraD435iNode::ValidateConfig() const
{
    if (config_.channels.depth_image.empty())
    {
        throw std::runtime_error("[RealsenseCameraD435i] channels.depth_image must not be empty.");
    }
    if (config_.stream.width <= 0 || config_.stream.height <= 0 || config_.stream.fps <= 0)
    {
        throw std::runtime_error("[RealsenseCameraD435i] stream width, height and fps must be positive.");
    }
    if (!std::isfinite(config_.stream.publish_fps) || config_.stream.publish_fps < 0.0)
    {
        throw std::runtime_error("[RealsenseCameraD435i] stream.publish_fps must be >= 0.");
    }
    if (!std::isfinite(config_.depth.min_depth_m) ||
        !std::isfinite(config_.depth.max_depth_m) ||
        config_.depth.min_depth_m < 0.0 ||
        config_.depth.min_depth_m >= config_.depth.max_depth_m)
    {
        throw std::runtime_error("[RealsenseCameraD435i] invalid depth range.");
    }
}

void RealsenseCameraD435iNode::StartCamera()
{
    rs2::config cfg;
    if (!config_.stream.serial.empty())
    {
        cfg.enable_device(config_.stream.serial);
    }

    cfg.enable_stream(
        RS2_STREAM_DEPTH,
        config_.stream.width,
        config_.stream.height,
        RS2_FORMAT_Z16,
        config_.stream.fps);

    std::cout << "[RealsenseCameraD435i] starting RealSense pipeline..." << std::endl;

    // When T265 is already opened by RealsenseCamera, enumerating all devices
    // and querying their sensors may fail on the busy T265 USB interface. Let
    // librealsense resolve a device that can satisfy the depth stream instead.
    pipeline_profile_ = pipeline_.start(cfg);
    std::cout << "[RealsenseCameraD435i] RealSense pipeline started" << std::endl;

    const rs2::device device = pipeline_profile_.get_device();
    std::cout << "[RealsenseCameraD435i] selected depth device: "
              << CameraInfoOrUnknown(device, RS2_CAMERA_INFO_NAME)
              << ", serial=" << CameraInfoOrUnknown(device, RS2_CAMERA_INFO_SERIAL_NUMBER)
              << std::endl;

    for (const rs2::sensor& sensor : device.query_sensors())
    {
        const rs2::depth_sensor depth_sensor = sensor.as<rs2::depth_sensor>();
        if (depth_sensor)
        {
            depth_scale_m_ = static_cast<double>(depth_sensor.get_depth_scale());
            break;
        }
    }

    if (!(depth_scale_m_ > 0.0) || !std::isfinite(depth_scale_m_))
    {
        throw std::runtime_error("[RealsenseCameraD435i] invalid depth scale from camera.");
    }
}

void RealsenseCameraD435iNode::PublishDepthFrame(const rs2::depth_frame& frame)
{
    const std::vector<uint8_t> raw_payload = BuildDepthPayload(frame);

    mors_msgs::depth_image_msg msg;
    msg.timestamp = NowNs();
    msg.width = frame.get_width();
    msg.height = frame.get_height();
    msg.compression = kCompressionRawU16Mm;
    msg.data = raw_payload;

    if (config_.compression.enabled && !raw_payload.empty())
    {
        const std::vector<uint8_t> compressed_payload = CompressPayload(raw_payload);
        if (!compressed_payload.empty() && compressed_payload.size() < raw_payload.size())
        {
            msg.compression = kCompressionZlibU16Mm;
            msg.data = compressed_payload;
        }
    }

    msg.data_size = static_cast<int32_t>(msg.data.size());
    lcm_.publish(config_.channels.depth_image, &msg);

    ++frame_counter_;
    if (config_.runtime.verbose && frame_counter_ % 30u == 0u)
    {
        std::cout << "[RealsenseCameraD435i] published frame " << frame_counter_
                  << ", payload=" << msg.data_size
                  << ", compression=" << static_cast<int>(msg.compression)
                  << std::endl;
    }
}

std::vector<uint8_t> RealsenseCameraD435iNode::BuildDepthPayload(const rs2::depth_frame& frame) const
{
    const int width = frame.get_width();
    const int height = frame.get_height();
    const size_t pixel_count = static_cast<size_t>(width) * static_cast<size_t>(height);

    std::vector<uint8_t> payload(pixel_count * kRawDepthBytesPerPixel, 0u);
    const auto* depth_units = static_cast<const uint16_t*>(frame.get_data());
    if (depth_units == nullptr)
    {
        return payload;
    }

    for (size_t i = 0; i < pixel_count; ++i)
    {
        const uint16_t raw_depth = depth_units[i];
        if (raw_depth == 0u)
        {
            continue;
        }

        const double depth_m = static_cast<double>(raw_depth) * depth_scale_m_;
        if (!std::isfinite(depth_m) ||
            depth_m < config_.depth.min_depth_m ||
            depth_m > config_.depth.max_depth_m)
        {
            continue;
        }

        const auto depth_mm_u32 = static_cast<uint32_t>(
            std::lround(depth_m * kMetersToMillimeters));
        if (depth_mm_u32 == 0u || depth_mm_u32 > std::numeric_limits<uint16_t>::max())
        {
            continue;
        }

        // The LCM contract matches Simulator: little-endian uint16 millimeters,
        // row-major, with zero reserved for invalid depth.
        const auto depth_mm = static_cast<uint16_t>(depth_mm_u32);
        payload[2u * i] = static_cast<uint8_t>(depth_mm & 0xFFu);
        payload[2u * i + 1u] = static_cast<uint8_t>((depth_mm >> 8u) & 0xFFu);
    }

    return payload;
}

std::vector<uint8_t> RealsenseCameraD435iNode::CompressPayload(
    const std::vector<uint8_t>& raw_payload) const
{
    if (raw_payload.empty())
    {
        return {};
    }

    uLongf compressed_size = compressBound(static_cast<uLong>(raw_payload.size()));
    std::vector<uint8_t> compressed_payload(compressed_size);

    const int status = compress2(
        compressed_payload.data(),
        &compressed_size,
        raw_payload.data(),
        static_cast<uLong>(raw_payload.size()),
        ClampZlibLevel(config_.compression.zlib_level));
    if (status != Z_OK)
    {
        return {};
    }

    compressed_payload.resize(static_cast<size_t>(compressed_size));
    return compressed_payload;
}

}  // namespace rscam
