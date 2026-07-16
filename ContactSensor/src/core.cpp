#include "contact_sensor/core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>

#include <yaml-cpp/yaml.h>

namespace contact_sensor {
namespace {

constexpr std::array<std::string_view, kLegCount> kThresholdNames{
    "r1", "l1", "r2", "l2"};

std::size_t HeaderPrefixSuffixLength(
    std::span<const std::uint8_t> bytes) noexcept {
    const std::size_t maximum_length =
        std::min(bytes.size(), kFrameHeader.size() - 1);
    for (std::size_t length = maximum_length; length > 0; --length) {
        if (std::equal(bytes.end() - static_cast<std::ptrdiff_t>(length),
                       bytes.end(), kFrameHeader.begin())) {
            return length;
        }
    }
    return 0;
}

RawData DecodePayload(
    std::span<const std::uint8_t, kPayloadSize> payload) noexcept {
    RawData raw_data{};
    for (std::size_t leg = 0; leg < kLegCount; ++leg) {
        const std::size_t offset = leg * sizeof(std::int16_t);
        const std::uint16_t encoded =
            static_cast<std::uint16_t>(payload[offset]) |
            (static_cast<std::uint16_t>(payload[offset + 1]) << 8U);

        raw_data[leg] = (encoded & 0x8000U) != 0U
                            ? static_cast<std::int32_t>(encoded) - 0x10000
                            : static_cast<std::int32_t>(encoded);
    }
    return raw_data;
}

[[noreturn]] void ThrowConfigError(const std::string& path,
                                   const std::string& detail) {
    throw std::runtime_error("invalid contact sensor config '" + path +
                             "': " + detail);
}

float ParseFiniteFloat(const YAML::Node& value,
                       const std::string& path,
                       const std::string& field) {
    if (!value || !value.IsScalar()) {
        ThrowConfigError(path, field + " must be a finite number");
    }

    double parsed_value{};
    try {
        parsed_value = value.as<double>();
    } catch (const YAML::Exception&) {
        ThrowConfigError(path, field + " must be a finite number");
    }

    constexpr double kMaximum =
        static_cast<double>(std::numeric_limits<float>::max());
    if (!std::isfinite(parsed_value) ||
        parsed_value < -kMaximum || parsed_value > kMaximum) {
        ThrowConfigError(path, field + " must be a finite number");
    }

    return static_cast<float>(parsed_value);
}

float ParseThreshold(const YAML::Node& thresholds,
                     const std::string& path,
                     std::string_view name) {
    const std::string key{name};
    return ParseFiniteFloat(
        thresholds[key], path, "thresholds." + key);
}

std::size_t ParseCalibrationSamples(const YAML::Node& root,
                                    const std::string& path) {
    const YAML::Node value = root["calibration_samples"];
    if (!value || !value.IsScalar()) {
        ThrowConfigError(path, "calibration_samples must be a positive integer");
    }

    long long parsed_value{};
    try {
        parsed_value = value.as<long long>();
    } catch (const YAML::Exception&) {
        ThrowConfigError(path, "calibration_samples must be a positive integer");
    }

    if (parsed_value <= 0 ||
        static_cast<std::uint64_t>(parsed_value) >
            static_cast<std::uint64_t>(
                std::numeric_limits<std::size_t>::max())) {
        ThrowConfigError(path, "calibration_samples is outside the valid range");
    }

    return static_cast<std::size_t>(parsed_value);
}

}  // namespace

Config LoadConfig(const std::string& path) {
    if (path.empty()) {
        throw std::runtime_error(
            "contact sensor config path must not be empty");
    }

    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& error) {
        throw std::runtime_error("failed to load contact sensor config '" +
                                 path + "': " + error.what());
    }

    if (!root || !root.IsMap()) {
        ThrowConfigError(path, "root must be a map");
    }

    const YAML::Node serial_port = root["serial_port"];
    if (!serial_port || !serial_port.IsScalar()) {
        ThrowConfigError(path, "serial_port must be a non-empty string");
    }

    Config config;
    try {
        config.serial_port = serial_port.as<std::string>();
    } catch (const YAML::Exception&) {
        ThrowConfigError(path, "serial_port must be a non-empty string");
    }
    if (config.serial_port.empty()) {
        ThrowConfigError(path, "serial_port must be a non-empty string");
    }

    config.raw_data_divisor =
        ParseFiniteFloat(root["raw_data_divisor"], path, "raw_data_divisor");
    if (config.raw_data_divisor <= 0.0F) {
        ThrowConfigError(path, "raw_data_divisor must be greater than zero");
    }

    config.calibration_samples = ParseCalibrationSamples(root, path);

    const YAML::Node thresholds = root["thresholds"];
    if (!thresholds || !thresholds.IsMap()) {
        ThrowConfigError(path, "thresholds must be a map");
    }

    for (std::size_t leg = 0; leg < kLegCount; ++leg) {
        config.thresholds[leg] =
            ParseThreshold(thresholds, path, kThresholdNames[leg]);
    }

    return config;
}

std::uint16_t ComputeCrc16CcittFalse(
    std::span<const std::uint8_t> bytes) noexcept {
    std::uint16_t crc = 0xFFFFU;
    for (const std::uint8_t byte : bytes) {
        crc ^= static_cast<std::uint16_t>(byte) << 8U;
        for (std::uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U) != 0U
                      ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
                      : static_cast<std::uint16_t>(crc << 1U);
        }
    }
    return crc;
}

std::optional<RawData> DecodeFrame(
    std::span<const std::uint8_t, kFrameSize> frame) noexcept {
    if (!std::equal(kFrameHeader.begin(), kFrameHeader.end(), frame.begin()) ||
        !std::equal(kFrameTerminator.begin(), kFrameTerminator.end(),
                    frame.begin() +
                        static_cast<std::ptrdiff_t>(kTerminatorOffset))) {
        return std::nullopt;
    }

    const std::uint16_t encoded_crc =
        (static_cast<std::uint16_t>(frame[kCrcOffset]) << 8U) |
        static_cast<std::uint16_t>(frame[kCrcOffset + 1]);
    const std::uint16_t computed_crc =
        ComputeCrc16CcittFalse(frame.first(kCrcOffset));
    if (encoded_crc != computed_crc) {
        return std::nullopt;
    }

    return DecodePayload(
        std::span<const std::uint8_t, kPayloadSize>{
            frame.data() + kPayloadOffset, kPayloadSize});
}

ContactStates DetectContacts(const ScaledRawData& raw_data,
                             const Thresholds& thresholds) noexcept {
    ContactStates contacts{};
    for (std::size_t leg = 0; leg < kLegCount; ++leg) {
        contacts[leg] = raw_data[leg] >= thresholds[leg];
    }
    return contacts;
}

ScaledRawData ScaleRawData(const RawData& raw_data, float divisor) noexcept {
    ScaledRawData scaled_data{};
    for (std::size_t leg = 0; leg < kLegCount; ++leg) {
        scaled_data[leg] = static_cast<float>(raw_data[leg]) / divisor;
    }
    return scaled_data;
}

ScaledRawData AbsoluteRawData(const ScaledRawData& raw_data) noexcept {
    ScaledRawData absolute_data{};
    for (std::size_t leg = 0; leg < kLegCount; ++leg) {
        absolute_data[leg] = std::abs(raw_data[leg]);
    }
    return absolute_data;
}

ZeroOffsetCalibrator::ZeroOffsetCalibrator(std::size_t sample_count)
    : sample_count_(sample_count) {
    if (sample_count == 0) {
        throw std::invalid_argument(
            "calibration sample count must be greater than zero");
    }
}

std::optional<ScaledRawData> ZeroOffsetCalibrator::Process(
    const ScaledRawData& raw_data) noexcept {
    if (!IsCalibrated()) {
        for (std::size_t leg = 0; leg < kLegCount; ++leg) {
            sums_[leg] += raw_data[leg];
        }
        ++samples_collected_;

        if (IsCalibrated()) {
            for (std::size_t leg = 0; leg < kLegCount; ++leg) {
                offset_[leg] = static_cast<float>(
                    sums_[leg] / static_cast<double>(sample_count_));
            }
        }
        return std::nullopt;
    }

    ScaledRawData corrected{};
    for (std::size_t leg = 0; leg < kLegCount; ++leg) {
        corrected[leg] = raw_data[leg] - offset_[leg];
    }
    return corrected;
}

void ZeroOffsetCalibrator::Reset() noexcept {
    samples_collected_ = 0;
    sums_.fill(0);
    offset_.fill(0);
}

bool ZeroOffsetCalibrator::IsCalibrated() const noexcept {
    return samples_collected_ == sample_count_;
}

std::size_t ZeroOffsetCalibrator::samples_collected() const noexcept {
    return samples_collected_;
}

const Offsets& ZeroOffsetCalibrator::offset() const noexcept {
    return offset_;
}

std::vector<RawData> FrameAccumulator::Append(
    std::span<const std::uint8_t> bytes) {
    buffer_.insert(buffer_.end(), bytes.begin(), bytes.end());

    std::vector<RawData> frames;
    frames.reserve(buffer_.size() / kFrameSize);

    std::size_t search_offset = 0;
    while (search_offset < buffer_.size()) {
        const auto header = std::search(
            buffer_.begin() + static_cast<std::ptrdiff_t>(search_offset),
            buffer_.end(), kFrameHeader.begin(), kFrameHeader.end());

        if (header == buffer_.end()) {
            const auto unprocessed = std::span<const std::uint8_t>{
                buffer_.data() + search_offset,
                buffer_.size() - search_offset};
            const std::size_t bytes_to_keep =
                HeaderPrefixSuffixLength(unprocessed);
            if (bytes_to_keep == 0) {
                buffer_.clear();
            } else {
                buffer_.erase(
                    buffer_.begin(),
                    buffer_.end() -
                        static_cast<std::ptrdiff_t>(bytes_to_keep));
            }
            return frames;
        }

        const std::size_t frame_offset =
            static_cast<std::size_t>(std::distance(buffer_.begin(), header));
        if (buffer_.size() - frame_offset < kFrameSize) {
            buffer_.erase(
                buffer_.begin(),
                buffer_.begin() + static_cast<std::ptrdiff_t>(frame_offset));
            return frames;
        }

        const auto frame = std::span<const std::uint8_t, kFrameSize>{
            buffer_.data() + frame_offset, kFrameSize};
        if (const std::optional<RawData> decoded = DecodeFrame(frame)) {
            frames.push_back(*decoded);
            search_offset = frame_offset + kFrameSize;
        } else {
            // The candidate may contain the beginning of the next valid frame.
            search_offset = frame_offset + 1;
        }
    }

    buffer_.clear();
    return frames;
}

void FrameAccumulator::Reset() noexcept {
    buffer_.clear();
}

std::size_t FrameAccumulator::buffered_size() const noexcept {
    return buffer_.size();
}

FallbackScheduler::FallbackScheduler(TimePoint start) noexcept
    : next_deadline_(start + kSilenceTimeout) {}

void FallbackScheduler::OnFrame(TimePoint now) noexcept {
    next_deadline_ = now + kSilenceTimeout;
}

void FallbackScheduler::OnIoError(TimePoint now) noexcept {
    next_deadline_ = now;
}

bool FallbackScheduler::IsDue(TimePoint now) const noexcept {
    return now >= next_deadline_;
}

void FallbackScheduler::MarkPublished(TimePoint now) noexcept {
    next_deadline_ = now + kRepeatInterval;
}

FallbackScheduler::TimePoint FallbackScheduler::NextDeadline() const noexcept {
    return next_deadline_;
}

}  // namespace contact_sensor
