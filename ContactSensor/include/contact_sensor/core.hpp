#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace contact_sensor {

inline constexpr std::size_t kLegCount = 4;

// Wire format (15 bytes):
// "#01" + R1,L1,R2,L2 (int16 LE) + CRC-16/CCITT-FALSE (BE) + "\r\n".
// The CRC covers the header and payload.
inline constexpr std::array<std::uint8_t, 3> kFrameHeader{'#', '0', '1'};
inline constexpr std::array<std::uint8_t, 2> kFrameTerminator{'\r', '\n'};
inline constexpr std::size_t kPayloadSize =
    kLegCount * sizeof(std::int16_t);
inline constexpr std::size_t kCrcSize = sizeof(std::uint16_t);
inline constexpr std::size_t kPayloadOffset = kFrameHeader.size();
inline constexpr std::size_t kCrcOffset = kPayloadOffset + kPayloadSize;
inline constexpr std::size_t kTerminatorOffset = kCrcOffset + kCrcSize;
inline constexpr std::size_t kFrameSize =
    kTerminatorOffset + kFrameTerminator.size();
static_assert(kFrameSize == 15);

using RawData = std::array<std::int32_t, kLegCount>;
using ScaledRawData = std::array<float, kLegCount>;
using Offsets = std::array<float, kLegCount>;
using Thresholds = std::array<float, kLegCount>;
using ContactStates = std::array<bool, kLegCount>;

struct Config {
    std::string serial_port;
    float raw_data_divisor{};
    std::size_t calibration_samples{};
    Thresholds thresholds{};
};

Config LoadConfig(const std::string& path);

std::uint16_t ComputeCrc16CcittFalse(
    std::span<const std::uint8_t> bytes) noexcept;

std::optional<RawData> DecodeFrame(
    std::span<const std::uint8_t, kFrameSize> frame) noexcept;

ContactStates DetectContacts(const ScaledRawData& raw_data,
                             const Thresholds& thresholds) noexcept;

ScaledRawData ScaleRawData(const RawData& raw_data, float divisor) noexcept;

ScaledRawData AbsoluteRawData(const ScaledRawData& raw_data) noexcept;

class ZeroOffsetCalibrator {
public:
    explicit ZeroOffsetCalibrator(std::size_t sample_count);

    // Calibration samples are consumed but not returned. Once calibration is
    // complete, every subsequent sample is corrected by the measured offset.
    [[nodiscard]] std::optional<ScaledRawData> Process(
        const ScaledRawData& raw_data) noexcept;

    void Reset() noexcept;

    [[nodiscard]] bool IsCalibrated() const noexcept;
    [[nodiscard]] std::size_t samples_collected() const noexcept;
    [[nodiscard]] const Offsets& offset() const noexcept;

private:
    std::size_t sample_count_;
    std::size_t samples_collected_{};
    std::array<double, kLegCount> sums_{};
    Offsets offset_{};
};

class FrameAccumulator {
public:
    std::vector<RawData> Append(std::span<const std::uint8_t> bytes);

    void Reset() noexcept;

    [[nodiscard]] std::size_t buffered_size() const noexcept;

private:
    std::vector<std::uint8_t> buffer_;
};

class FallbackScheduler {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    static constexpr auto kSilenceTimeout = std::chrono::milliseconds{100};
    static constexpr auto kRepeatInterval = std::chrono::milliseconds{100};

    explicit FallbackScheduler(TimePoint start) noexcept;

    void OnFrame(TimePoint now) noexcept;
    void OnIoError(TimePoint now) noexcept;

    [[nodiscard]] bool IsDue(TimePoint now) const noexcept;
    void MarkPublished(TimePoint now) noexcept;

    [[nodiscard]] TimePoint NextDeadline() const noexcept;

private:
    TimePoint next_deadline_;
};

}  // namespace contact_sensor
