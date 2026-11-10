#include "contact_sensor/core.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace {

using contact_sensor::ContactStates;
using contact_sensor::ChannelReadiness;
using contact_sensor::DecodedFrame;
using contact_sensor::Offsets;
using contact_sensor::RawData;
using contact_sensor::ScaledRawData;
using contact_sensor::Thresholds;

constexpr ChannelReadiness kAllReady{true, true, true, true};

DecodedFrame Decoded(
    RawData raw_data,
    ChannelReadiness readiness = kAllReady) {
    return DecodedFrame{raw_data, readiness};
}

class TestFailure : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

void Check(bool condition, std::string_view message) {
    if (!condition) {
        throw TestFailure(std::string{message});
    }
}

template <typename Actual, typename Expected>
void CheckEqual(const Actual& actual, const Expected& expected,
                std::string_view message) {
    if (!(actual == expected)) {
        throw TestFailure(std::string{message});
    }
}

class TemporaryConfig {
public:
    explicit TemporaryConfig(std::string_view contents)
        : path_{MakePath()} {
        std::ofstream output{path_};
        if (!output) {
            throw TestFailure("could not create temporary config");
        }
        output << contents;
        if (!output) {
            throw TestFailure("could not write temporary config");
        }
    }

    TemporaryConfig(const TemporaryConfig&) = delete;
    TemporaryConfig& operator=(const TemporaryConfig&) = delete;

    ~TemporaryConfig() {
        std::error_code error;
        std::filesystem::remove(path_, error);
    }

    [[nodiscard]] std::string path() const { return path_.string(); }

private:
    static std::filesystem::path MakePath() {
        static std::uint64_t sequence = 0;
        return std::filesystem::temp_directory_path() /
               ("contact_sensor_core_test_" +
                std::to_string(++sequence) + ".yaml");
    }

    std::filesystem::path path_;
};

template <typename Function>
void ExpectRuntimeError(Function&& function, std::string_view expected_text) {
    try {
        std::forward<Function>(function)();
    } catch (const std::runtime_error& error) {
        Check(std::string_view{error.what()}.find(expected_text) !=
                  std::string_view::npos,
              "runtime_error did not contain the expected diagnostic");
        return;
    }
    throw TestFailure("expected std::runtime_error");
}

std::array<std::uint8_t, contact_sensor::kFrameSize> MakeFrame(
    const RawData& raw_data,
    const ChannelReadiness& readiness = kAllReady) {
    std::array<std::uint8_t, contact_sensor::kFrameSize> frame{};
    std::copy(contact_sensor::kFrameHeader.begin(),
              contact_sensor::kFrameHeader.end(), frame.begin());

    for (std::size_t leg = 0; leg < contact_sensor::kLegCount; ++leg) {
        const std::uint16_t encoded =
            static_cast<std::uint16_t>(raw_data[leg]);
        const std::size_t offset =
            contact_sensor::kPayloadOffset + leg * sizeof(std::int16_t);
        frame[offset] = static_cast<std::uint8_t>(encoded & 0x00FFU);
        frame[offset + 1] = static_cast<std::uint8_t>(encoded >> 8U);
        if (readiness[leg]) {
            frame[contact_sensor::kReadinessOffset] |=
                static_cast<std::uint8_t>(1U << leg);
        }
    }

    const std::uint16_t crc = contact_sensor::ComputeCrc16CcittFalse(
        std::span<const std::uint8_t>{frame}.first(
            contact_sensor::kCrcOffset));
    frame[contact_sensor::kCrcOffset] =
        static_cast<std::uint8_t>(crc >> 8U);
    frame[contact_sensor::kCrcOffset + 1] =
        static_cast<std::uint8_t>(crc & 0x00FFU);
    std::copy(contact_sensor::kFrameTerminator.begin(),
              contact_sensor::kFrameTerminator.end(),
              frame.begin() +
                  static_cast<std::ptrdiff_t>(
                      contact_sensor::kTerminatorOffset));
    return frame;
}

void RefreshCrc(
    std::array<std::uint8_t, contact_sensor::kFrameSize>& frame) {
    const std::uint16_t crc = contact_sensor::ComputeCrc16CcittFalse(
        std::span<const std::uint8_t>{frame}.first(
            contact_sensor::kCrcOffset));
    frame[contact_sensor::kCrcOffset] =
        static_cast<std::uint8_t>(crc >> 8U);
    frame[contact_sensor::kCrcOffset + 1] =
        static_cast<std::uint8_t>(crc & 0x00FFU);
}

void TestCrcGoldenVectors() {
    constexpr std::array<std::uint8_t, 9> standard_input{
        '1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CheckEqual(contact_sensor::ComputeCrc16CcittFalse(standard_input),
               std::uint16_t{0x29B1},
               "CRC-16/CCITT-FALSE standard check value is wrong");

    constexpr std::array<std::uint8_t, contact_sensor::kFrameSize>
        expected_frame{
            '#', '0', '2',
            0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x04, 0x00,
            0x0F,
            0x2B, 0xAD,
            '\r', '\n',
        };
    CheckEqual(MakeFrame(RawData{1, 2, 3, 4}), expected_frame,
               "wire-format golden frame is wrong");
}

void TestDecodeSignedLittleEndian() {
    const auto frame = MakeFrame(RawData{-32768, 32767, -2, 0x1234});
    const std::optional<DecodedFrame> decoded =
        contact_sensor::DecodeFrame(frame);
    Check(decoded.has_value(), "valid frame was rejected");
    CheckEqual(*decoded, Decoded(RawData{-32768, 32767, -2, 0x1234}),
               "little-endian signed decoding failed");
}

void TestDecodeRejectsMalformedFrames() {
    const auto valid = MakeFrame(RawData{1, 2, 3, 4});

    auto bad_header = valid;
    bad_header[2] = '1';
    Check(!contact_sensor::DecodeFrame(bad_header).has_value(),
          "legacy #01 frame was accepted");

    auto bad_payload = valid;
    bad_payload[contact_sensor::kPayloadOffset] ^= 0x01U;
    Check(!contact_sensor::DecodeFrame(bad_payload).has_value(),
          "frame with a bad CRC was accepted");

    auto bad_terminator = valid;
    bad_terminator[contact_sensor::kTerminatorOffset] = '\n';
    Check(!contact_sensor::DecodeFrame(bad_terminator).has_value(),
          "frame with a bad terminator was accepted");

    auto changed_readiness = valid;
    changed_readiness[contact_sensor::kReadinessOffset] ^= 0x01U;
    Check(!contact_sensor::DecodeFrame(changed_readiness).has_value(),
          "readiness mask was not covered by CRC");

    for (std::uint8_t bit = 4; bit < 8; ++bit) {
        auto reserved_readiness = valid;
        reserved_readiness[contact_sensor::kReadinessOffset] |=
            static_cast<std::uint8_t>(1U << bit);
        RefreshCrc(reserved_readiness);
        Check(!contact_sensor::DecodeFrame(reserved_readiness).has_value(),
              "reserved readiness bit was accepted");
    }

    auto inconsistent = MakeFrame(
        RawData{1, 2, 3, 4}, ChannelReadiness{false, true, true, true});
    Check(!contact_sensor::DecodeFrame(inconsistent).has_value(),
          "non-marker value for an unavailable channel was accepted");

    const auto partial = MakeFrame(
        RawData{-1, 2, -1, 4},
        ChannelReadiness{false, true, false, true});
    CheckEqual(contact_sensor::DecodeFrame(partial),
               std::optional<DecodedFrame>{Decoded(
                   RawData{-1, 2, -1, 4},
                   ChannelReadiness{false, true, false, true})},
               "valid partial readiness frame was rejected");
}

void TestFrameAccumulatorFragmentsAndBackToBackFrames() {
    const auto first = MakeFrame(RawData{1, 2, 3, 4});
    const auto second = MakeFrame(
        RawData{-1, -2, -1, -4},
        ChannelReadiness{false, true, false, true});
    const auto third = MakeFrame(RawData{5, 6, 7, 8});
    std::vector<std::uint8_t> bytes;
    bytes.insert(bytes.end(), first.begin(), first.end());
    bytes.insert(bytes.end(), second.begin(), second.end());
    bytes.insert(bytes.end(), third.begin(), third.end());

    contact_sensor::FrameAccumulator accumulator;
    Check(accumulator.Append(std::span{bytes}.first(4)).empty(),
          "a partial frame must not produce data");
    CheckEqual(accumulator.buffered_size(), std::size_t{4},
               "partial byte count was not retained");

    const auto first_two =
        accumulator.Append(std::span{bytes}.subspan(
            4, 2 * contact_sensor::kFrameSize - 2));
    CheckEqual(first_two.size(), std::size_t{2},
               "fragment plus back-to-back frame count is wrong");
    CheckEqual(first_two[0], Decoded(RawData{1, 2, 3, 4}),
               "fragmented frame decoded incorrectly");
    CheckEqual(first_two[1], Decoded(
                   RawData{-1, -2, -1, -4},
                   ChannelReadiness{false, true, false, true}),
               "second consecutive frame decoded incorrectly");
    CheckEqual(accumulator.buffered_size(), std::size_t{2},
               "trailing partial frame was not retained");

    const auto final_frame = accumulator.Append(
        std::span{bytes}.last(contact_sensor::kFrameSize - 2));
    CheckEqual(final_frame.size(), std::size_t{1},
               "completion of trailing frame produced wrong count");
    CheckEqual(final_frame[0], Decoded(RawData{5, 6, 7, 8}),
               "trailing frame decoded incorrectly");
    CheckEqual(accumulator.buffered_size(), std::size_t{0},
               "buffer must be empty after complete frames");

    accumulator.Append(std::span{bytes}.first(5));
    accumulator.Reset();
    CheckEqual(accumulator.buffered_size(), std::size_t{0},
               "Reset did not discard a partial frame");
}

void TestFrameAccumulatorResynchronizes() {
    const auto valid = MakeFrame(RawData{10, 20, 30, 40});

    contact_sensor::FrameAccumulator split_header_accumulator;
    constexpr std::array<std::uint8_t, 4> leading_noise{'x', '\r', '\n', '#'};
    Check(split_header_accumulator.Append(leading_noise).empty(),
          "noise unexpectedly produced a frame");
    CheckEqual(split_header_accumulator.buffered_size(), std::size_t{1},
               "partial header was not retained after noise");

    constexpr std::array<std::uint8_t, 1> header_middle{'0'};
    Check(split_header_accumulator.Append(header_middle).empty(),
          "partial header unexpectedly produced a frame");
    CheckEqual(split_header_accumulator.buffered_size(), std::size_t{2},
               "split header was not retained");

    const auto split_result = split_header_accumulator.Append(
        std::span{valid}.subspan(2));
    CheckEqual(split_result,
               std::vector<DecodedFrame>{Decoded(RawData{10, 20, 30, 40})},
               "frame following split header was not recovered");

    // The damaged payload deliberately contains an embedded "#02". The
    // parser must reject each false candidate and still find the next frame.
    auto damaged = MakeFrame(RawData{0x3023, 0x0032, 50, 60});
    damaged[contact_sensor::kPayloadOffset + 4] ^= 0x01U;
    std::vector<std::uint8_t> stream{'?', '?'};
    stream.insert(stream.end(), damaged.begin(), damaged.end());
    stream.insert(stream.end(), valid.begin(), valid.end());

    contact_sensor::FrameAccumulator crc_accumulator;
    const auto crc_result = crc_accumulator.Append(stream);
    CheckEqual(crc_result,
               std::vector<DecodedFrame>{Decoded(RawData{10, 20, 30, 40})},
               "parser did not recover after a CRC error");
    CheckEqual(crc_accumulator.buffered_size(), std::size_t{0},
               "parser retained bytes after resynchronization");
    CheckEqual(crc_accumulator.rejected_candidates(), std::size_t{2},
               "damaged frame and embedded header must both be rejected");

    auto bad_terminator = valid;
    bad_terminator[contact_sensor::kTerminatorOffset + 1] = '!';
    stream.assign(bad_terminator.begin(), bad_terminator.end());
    stream.insert(stream.end(), valid.begin(), valid.end());

    contact_sensor::FrameAccumulator terminator_accumulator;
    const auto terminator_result = terminator_accumulator.Append(stream);
    CheckEqual(terminator_result,
               std::vector<DecodedFrame>{Decoded(RawData{10, 20, 30, 40})},
               "parser did not recover after a terminator error");
    CheckEqual(terminator_accumulator.rejected_candidates(), std::size_t{1},
               "frame with a bad terminator was not counted as rejected");

    auto shortened = MakeFrame(RawData{70, 80, 90, 100});
    stream.assign(shortened.begin(), shortened.end());
    stream.erase(stream.begin() + static_cast<std::ptrdiff_t>(
                                    contact_sensor::kPayloadOffset + 2));
    stream.insert(stream.end(), valid.begin(), valid.end());

    contact_sensor::FrameAccumulator lost_byte_accumulator;
    const auto lost_byte_result = lost_byte_accumulator.Append(stream);
    CheckEqual(lost_byte_result,
               std::vector<DecodedFrame>{Decoded(RawData{10, 20, 30, 40})},
               "parser did not recover after a lost byte");

    contact_sensor::FrameAccumulator bytewise_accumulator;
    std::vector<DecodedFrame> bytewise_result;
    for (const std::uint8_t byte : stream) {
        const std::array<std::uint8_t, 1> chunk{byte};
        const auto decoded = bytewise_accumulator.Append(chunk);
        bytewise_result.insert(
            bytewise_result.end(), decoded.begin(), decoded.end());
    }
    CheckEqual(bytewise_result,
               std::vector<DecodedFrame>{Decoded(RawData{10, 20, 30, 40})},
               "bytewise input did not recover after a lost byte");
}

void TestContactDetectionUsesEachThresholdAndEquality() {
    const ScaledRawData raw_data{9.5F, 10.0F, 11.5F, -5.0F};
    const Thresholds thresholds{10.0F, 10.0F, 12.0F, -5.0F};
    const ContactStates expected{false, true, false, true};

    CheckEqual(contact_sensor::DetectContacts(raw_data, thresholds), expected,
               "per-leg threshold comparison failed");
}

void TestRawDataScaling() {
    const RawData raw_data{142, 70, -142, -70};
    const ScaledRawData scaled_data =
        contact_sensor::ScaleRawData(raw_data, 71.0F);
    constexpr float tolerance = 1.0e-6F;

    Check(std::abs(scaled_data[0] - 2.0F) < tolerance,
          "positive exact LCM raw_data scaling is wrong");
    Check(std::abs(scaled_data[1] - 70.0F / 71.0F) < tolerance,
          "positive fractional LCM raw_data scaling is wrong");
    Check(std::abs(scaled_data[2] + 2.0F) < tolerance,
          "negative exact LCM raw_data scaling is wrong");
    Check(std::abs(scaled_data[3] + 70.0F / 71.0F) < tolerance,
          "negative fractional LCM raw_data scaling is wrong");
}

void TestAbsoluteRawData() {
    const ScaledRawData raw_data{-1.5F, 0.0F, 2.25F, -3.75F};
    CheckEqual(contact_sensor::AbsoluteRawData(raw_data),
               ScaledRawData{1.5F, 0.0F, 2.25F, 3.75F},
               "raw_data absolute-value conversion is wrong");
}

void TestLoadValidConfig() {
    const TemporaryConfig config_file{R"(
contact_sensor:
  serial_port: /dev/ttyACM7
  raw_data_divisor: 71.5
  calibration_samples: 3
  thresholds:
    r1: -12.5
    l1: -1.25
    r2: 0.5
    l2: 42.75
)"};

    const contact_sensor::Config config =
        contact_sensor::LoadConfig(config_file.path());
    CheckEqual(config.serial_port, std::string{"/dev/ttyACM7"},
               "serial_port was loaded incorrectly");
    CheckEqual(config.raw_data_divisor, 71.5F,
               "raw_data_divisor was loaded incorrectly");
    CheckEqual(config.calibration_samples, std::size_t{3},
               "calibration_samples was loaded incorrectly");
    CheckEqual(config.thresholds,
               Thresholds{-12.5F, -1.25F, 0.5F, 42.75F},
               "threshold ordering or values are wrong");
}

void TestRejectInvalidConfigs() {
    ExpectRuntimeError([] { contact_sensor::LoadConfig(""); },
                       "path must not be empty");

    const TemporaryConfig missing_section{R"(
serial_port: /dev/ttyACM0
)"};
    ExpectRuntimeError(
        [&] { contact_sensor::LoadConfig(missing_section.path()); },
        "contact_sensor must be a map");

    const TemporaryConfig missing_port{R"(
contact_sensor:
  thresholds: {r1: 1, l1: 2, r2: 3, l2: 4}
)"};
    ExpectRuntimeError(
        [&] { contact_sensor::LoadConfig(missing_port.path()); },
        "serial_port");

    const TemporaryConfig empty_port{R"(
contact_sensor:
  serial_port: ""
  raw_data_divisor: 71
  calibration_samples: 10
  thresholds: {r1: 1, l1: 2, r2: 3, l2: 4}
)"};
    ExpectRuntimeError(
        [&] { contact_sensor::LoadConfig(empty_port.path()); },
        "serial_port");

    const TemporaryConfig missing_divisor{R"(
contact_sensor:
  serial_port: /dev/ttyACM0
  calibration_samples: 10
  thresholds: {r1: 1, l1: 2, r2: 3, l2: 4}
)"};
    ExpectRuntimeError(
        [&] { contact_sensor::LoadConfig(missing_divisor.path()); },
        "raw_data_divisor");

    const TemporaryConfig zero_divisor{R"(
contact_sensor:
  serial_port: /dev/ttyACM0
  raw_data_divisor: 0
  calibration_samples: 10
  thresholds: {r1: 1, l1: 2, r2: 3, l2: 4}
)"};
    ExpectRuntimeError(
        [&] { contact_sensor::LoadConfig(zero_divisor.path()); },
        "raw_data_divisor must be greater than zero");

    const TemporaryConfig invalid_divisor{R"(
contact_sensor:
  serial_port: /dev/ttyACM0
  raw_data_divisor: invalid
  calibration_samples: 10
  thresholds: {r1: 1, l1: 2, r2: 3, l2: 4}
)"};
    ExpectRuntimeError(
        [&] { contact_sensor::LoadConfig(invalid_divisor.path()); },
        "raw_data_divisor must be a finite number");

    const TemporaryConfig missing_sample_count{R"(
contact_sensor:
  serial_port: /dev/ttyACM0
  raw_data_divisor: 71
  thresholds: {r1: 1, l1: 2, r2: 3, l2: 4}
)"};
    ExpectRuntimeError(
        [&] { contact_sensor::LoadConfig(missing_sample_count.path()); },
        "calibration_samples");

    const TemporaryConfig zero_sample_count{R"(
contact_sensor:
  serial_port: /dev/ttyACM0
  raw_data_divisor: 71
  calibration_samples: 0
  thresholds: {r1: 1, l1: 2, r2: 3, l2: 4}
)"};
    ExpectRuntimeError(
        [&] { contact_sensor::LoadConfig(zero_sample_count.path()); },
        "calibration_samples");

    const TemporaryConfig missing_leg{R"(
contact_sensor:
  serial_port: /dev/ttyACM0
  raw_data_divisor: 71
  calibration_samples: 10
  thresholds: {r1: 1, l1: 2, r2: 3}
)"};
    ExpectRuntimeError(
        [&] { contact_sensor::LoadConfig(missing_leg.path()); },
        "thresholds.l2");

    const TemporaryConfig invalid_threshold{R"(
contact_sensor:
  serial_port: /dev/ttyACM0
  raw_data_divisor: 71
  calibration_samples: 10
  thresholds: {r1: 1, l1: invalid, r2: 3, l2: 4}
)"};
    ExpectRuntimeError(
        [&] { contact_sensor::LoadConfig(invalid_threshold.path()); },
        "thresholds.l1 must be a finite number");

    const TemporaryConfig infinite_threshold{R"(
contact_sensor:
  serial_port: /dev/ttyACM0
  raw_data_divisor: 71
  calibration_samples: 10
  thresholds: {r1: .inf, l1: 2, r2: 3, l2: 4}
)"};
    ExpectRuntimeError(
        [&] { contact_sensor::LoadConfig(infinite_threshold.path()); },
        "thresholds.r1 must be a finite number");
}

void TestZeroOffsetCalibrationAndCorrection() {
    contact_sensor::ZeroOffsetCalibrator calibrator{3};
    CheckEqual(calibrator.Process(
                   RawData{9, -12, 30, 100}, kAllReady, 1).valid,
               ContactStates{}, "first calibration sample was returned");
    (void)calibrator.Process(RawData{12, -9, 33, 103}, kAllReady, 1);
    CheckEqual(calibrator.samples_collected(0), std::size_t{2}, "sample count");
    CheckEqual(calibrator.Process(
                   RawData{15, -6, 36, 106}, kAllReady, 1).valid,
               ContactStates{}, "last calibration sample was returned");
    for (std::size_t i = 0; i < 4; ++i) Check(calibrator.IsCalibrated(i), "ready");
    CheckEqual(calibrator.offset(), Offsets{12, -9, 33, 103}, "offset");
    const auto corrected = calibrator.Process(
        RawData{12, -4, 30, 120}, kAllReady, 1);
    CheckEqual(corrected.valid, ContactStates{true, true, true, true}, "valid");
    CheckEqual(corrected.values, ScaledRawData{0, 5, -3, 17}, "correction");
}

void TestZeroOffsetCalibrationReset() {
    contact_sensor::ZeroOffsetCalibrator calibrator{2};
    (void)calibrator.Process(RawData{100, 200, 300, 400}, kAllReady, 1);
    calibrator.Reset();
    Check(!calibrator.IsCalibrated(0), "Reset left calibrator ready");
    CheckEqual(calibrator.samples_collected(0), std::size_t{0}, "Reset count");
    CheckEqual(calibrator.offset(), Offsets{}, "Reset offsets");
    (void)calibrator.Process(RawData{10, 20, 30, 40}, kAllReady, 1);
    (void)calibrator.Process(RawData{20, 30, 40, 50}, kAllReady, 1);
    CheckEqual(calibrator.offset(), Offsets{15, 25, 35, 45}, "new offsets");
}

void TestAllReadinessMasks() {
    for (unsigned mask = 0; mask < 16; ++mask) {
        contact_sensor::ZeroOffsetCalibrator calibrator{2};
        RawData raw{};
        ChannelReadiness readiness{};
        for (unsigned i = 0; i < 4; ++i) {
            readiness[i] = (mask & (1U << i)) != 0U;
            raw[i] = readiness[i] ? 110 : -1;
        }
        const auto frame = MakeFrame(raw, readiness);
        CheckEqual(*contact_sensor::DecodeFrame(frame),
                   Decoded(raw, readiness), "readiness mask decoding");
        CheckEqual(contact_sensor::AbsoluteRawData(
                       calibrator.Process(raw, readiness, 55)),
                   ScaledRawData{-1, -1, -1, -1},
                   "first calibration publication");
        CheckEqual(calibrator.Process(raw, readiness, 55).valid,
                   ContactStates{},
                   "last calibration sample was published as valid");
        for (unsigned i = 0; i < 4; ++i) {
            raw[i] = readiness[i] ? 220 : -1;
        }
        const auto corrected = calibrator.Process(raw, readiness, 55);
        CheckEqual(corrected.valid, readiness,
                   "calibration readiness does not match channel mask");
        const auto output = contact_sensor::AbsoluteRawData(corrected);
        const auto contacts = contact_sensor::DetectContacts(
            output, Thresholds{1, 1, 1, 1});
        for (unsigned i = 0; i < 4; ++i) {
            CheckEqual(output[i], readiness[i] ? 2.0F : -1.0F,
                       "measurement correction");
            CheckEqual(contacts[i], readiness[i], "per-channel contact");
        }
    }
    CheckEqual(contact_sensor::DetectContacts(
                   ScaledRawData{-1, -1, -1, -1},
                   Thresholds{-10, -10, -10, -10}),
               ContactStates{},
               "invalid channel must remain false even with negative threshold");
}

void TestCalibrationInterruptionAndRecovery() {
    contact_sensor::ZeroOffsetCalibrator calibrator{2};
    (void)calibrator.Process(
        RawData{100, 200, -1, -1},
        ChannelReadiness{true, true, false, false}, 1);
    const auto interrupted = calibrator.Process(
        RawData{10, -1, 300, -1},
        ChannelReadiness{true, false, true, false}, 1);
    CheckEqual(interrupted.valid, ContactStates{},
               "last calibration sample was published as valid");
    Check(calibrator.IsCalibrated(0), "ready channel did not calibrate");
    CheckEqual(calibrator.offset()[0], 55.0F,
               "ready channel offset is wrong");
    CheckEqual(calibrator.samples_collected(1), std::size_t{0},
               "failed channel retained incomplete calibration");

    const auto partial = calibrator.Process(
        RawData{65, 20, -1, -1},
        ChannelReadiness{true, true, false, false}, 1);
    CheckEqual(partial.valid,
               ContactStates{true, false, false, false},
               "channels did not calibrate independently");
    CheckEqual(partial.values[0], 10.0F,
               "calibrated channel offset was not applied");
    CheckEqual(calibrator.samples_collected(2), std::size_t{0},
               "failed channel retained an incomplete sample");

    const auto last_sample = calibrator.Process(
        RawData{75, 40, -1, -1},
        ChannelReadiness{true, true, false, false}, 1);
    CheckEqual(last_sample.valid,
               ContactStates{true, false, false, false},
               "channel's last calibration sample was published");
    CheckEqual(calibrator.offset()[1], 30.0F,
               "reset calibration used samples from before failure");

    CheckEqual(calibrator.Process(
                   RawData{-1, -1, -1, -1}, ChannelReadiness{}, 1).valid,
               ContactStates{}, "failed channels remained valid");
    const auto recovered = calibrator.Process(
        RawData{85, 50, -1, -1},
        ChannelReadiness{true, true, false, false}, 1);
    CheckEqual(recovered.valid,
               ContactStates{true, true, false, false},
               "calibrated channels did not recover immediately");
    CheckEqual(contact_sensor::AbsoluteRawData(recovered),
               ScaledRawData{30, 20, -1, -1},
               "recovered channels did not retain their offsets");

    (void)calibrator.Process(
        RawData{-1, -1, 30, -1},
        ChannelReadiness{false, false, true, false}, 1);
    CheckEqual(calibrator.Process(
                   RawData{-1, -1, 50, -1},
                   ChannelReadiness{false, false, true, false}, 1).valid,
               ContactStates{},
               "last sample of delayed calibration was published");
    const auto delayed = calibrator.Process(
        RawData{-1, -1, 70, -1},
        ChannelReadiness{false, false, true, false}, 1);
    CheckEqual(contact_sensor::AbsoluteRawData(delayed),
               ScaledRawData{-1, -1, 30, -1},
               "delayed channel calibration is wrong");
}

void TestSignedMinusOneIntermediate() {
    contact_sensor::ZeroOffsetCalibrator calibrator{1};
    (void)calibrator.Process(RawData{-55, 110, 0, -55}, kAllReady, 55);
    CheckEqual(calibrator.offset(), Offsets{-1, 2, 0, -1},
               "signed value -1 was treated as unavailable");
    const auto output = contact_sensor::AbsoluteRawData(
        calibrator.Process(RawData{-110, 55, -55, -55}, kAllReady, 55));
    CheckEqual(output, ScaledRawData{1, 1, 1, 0}, "corrected -1 was treated as marker");
}

void TestFallbackTimingAndPartialFrameSilence() {
    using namespace std::chrono_literals;
    using Scheduler = contact_sensor::FallbackScheduler;
    const Scheduler::TimePoint start{};
    Scheduler scheduler{start};

    CheckEqual(scheduler.NextDeadline(), start + 100ms,
               "initial fallback deadline is wrong");
    Check(!scheduler.IsDue(start + 99ms),
          "fallback became due before the silence timeout");
    Check(scheduler.IsDue(start + 100ms),
          "fallback was not due at the silence timeout");

    scheduler.MarkPublished(start + 100ms);
    CheckEqual(scheduler.NextDeadline(), start + 200ms,
               "repeat deadline is wrong");

    scheduler.OnFrame(start + 150ms);
    CheckEqual(scheduler.NextDeadline(), start + 250ms,
               "a complete frame did not restart the silence timeout");

    Scheduler partial_scheduler{start};
    contact_sensor::FrameAccumulator accumulator;
    const std::array<std::uint8_t, 3> partial_bytes{'#', '0', '2'};
    Check(accumulator.Append(partial_bytes).empty(),
          "partial bytes unexpectedly formed a frame");
    Check(partial_scheduler.IsDue(start + 100ms),
          "partial bytes must not postpone fallback publication");
}

void TestFallbackIoErrorAndRecovery() {
    using namespace std::chrono_literals;
    using Scheduler = contact_sensor::FallbackScheduler;
    const Scheduler::TimePoint start{};
    Scheduler scheduler{start};

    scheduler.OnIoError(start + 10ms);
    CheckEqual(scheduler.NextDeadline(), start + 10ms,
               "I/O error did not make fallback immediately due");
    Check(scheduler.IsDue(start + 10ms),
          "I/O fallback is not immediately due");

    scheduler.MarkPublished(start + 10ms);
    CheckEqual(scheduler.NextDeadline(), start + 110ms,
               "I/O fallback repeat deadline is wrong");

    scheduler.OnFrame(start + 20ms);
    CheckEqual(scheduler.NextDeadline(), start + 120ms,
               "fresh data did not recover the silence schedule");
    Check(!scheduler.IsDue(start + 119ms),
          "recovered scheduler became due too early");
    Check(scheduler.IsDue(start + 120ms),
          "recovered scheduler missed its deadline");
}

struct TestCase {
    std::string_view name;
    void (*function)();
};

}  // namespace

int main() {
    const std::array tests{
        TestCase{"CRC golden vectors", TestCrcGoldenVectors},
        TestCase{"decode signed little-endian", TestDecodeSignedLittleEndian},
        TestCase{"reject malformed frames", TestDecodeRejectsMalformedFrames},
        TestCase{"frame accumulator", TestFrameAccumulatorFragmentsAndBackToBackFrames},
        TestCase{"frame resynchronization", TestFrameAccumulatorResynchronizes},
        TestCase{"contact detection", TestContactDetectionUsesEachThresholdAndEquality},
        TestCase{"raw_data scaling", TestRawDataScaling},
        TestCase{"raw_data absolute value", TestAbsoluteRawData},
        TestCase{"zero-offset calibration", TestZeroOffsetCalibrationAndCorrection},
        TestCase{"calibration reset", TestZeroOffsetCalibrationReset},
        TestCase{"all readiness masks", TestAllReadinessMasks},
        TestCase{"calibration interruption and recovery", TestCalibrationInterruptionAndRecovery},
        TestCase{"signed -1 intermediate", TestSignedMinusOneIntermediate},
        TestCase{"valid config", TestLoadValidConfig},
        TestCase{"invalid configs", TestRejectInvalidConfigs},
        TestCase{"fallback timing", TestFallbackTimingAndPartialFrameSilence},
        TestCase{"fallback I/O recovery", TestFallbackIoErrorAndRecovery},
    };

    std::size_t failures = 0;
    for (const TestCase& test : tests) {
        try {
            test.function();
            std::cout << "[PASS] " << test.name << '\n';
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "[FAIL] " << test.name << ": " << error.what()
                      << '\n';
        }
    }

    if (failures != 0) {
        std::cerr << failures << " test(s) failed\n";
        return 1;
    }

    std::cout << tests.size() << " test(s) passed\n";
    return 0;
}
