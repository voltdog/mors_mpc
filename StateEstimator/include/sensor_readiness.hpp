#ifndef STATE_ESTIMATOR_HMB_SENSOR_READINESS_HPP
#define STATE_ESTIMATOR_HMB_SENSOR_READINESS_HPP

#include "robot_state_source.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <span>

namespace state_estimator_hmb
{

struct SensorReadinessConfig
{
    double frequency_window_sec{0.5};
    double min_frequency_ratio{0.5};
    double data_timeout_sec{0.5};
    double encoder_window_sec{0.5};
    double encoder_std_threshold_rad{1.0e-6};
    double depth_frequency_hz{30.0};
    double imu_frequency_hz{500.0};
    double servo_frequency_hz{1000.0};
    double contact_frequency_hz{600.0};
    double t265_frequency_hz{200.0};

    void Validate() const;
};

class SensorReadiness final
{
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    using Readiness = std::array<bool, 19>;

    explicit SensorReadiness(SensorReadinessConfig config = {});

    void OnImu(std::span<const float, 4> quaternion, TimePoint now = Clock::now());
    void OnServo(std::span<const float, 12> position, TimePoint now = Clock::now());
    void OnContacts(std::span<const int8_t, 4> readiness, TimePoint now = Clock::now());
    void OnDepthFrame(TimePoint now = Clock::now());
    void OnT265Pose(TimePoint now = Clock::now());
    void SetDepthHealthy(bool healthy);
    void SetT265State(T265State state);

    [[nodiscard]] Readiness GetReadiness(TimePoint now = Clock::now());

private:
    struct Stream
    {
        std::optional<TimePoint> started;
        std::optional<TimePoint> last;
        std::deque<TimePoint> events;
    };

    struct EncoderSample
    {
        TimePoint time;
        std::array<float, 12> position;
    };

    void Tick(Stream& stream, TimePoint now);
    void Prune(Stream& stream, TimePoint now);
    bool StreamReady(Stream& stream, double frequency_hz, TimePoint now);
    void PruneEncoders(TimePoint now);

    const SensorReadinessConfig config_;
    std::mutex mutex_;
    Stream depth_;
    Stream imu_;
    Stream servo_;
    Stream contacts_;
    Stream t265_;
    bool imu_valid_{false};
    std::array<bool, 4> contacts_valid_{};
    bool depth_healthy_{false};
    bool t265_healthy_{false};
    bool t265_startup_failed_{false};
    std::optional<TimePoint> encoders_started_;
    std::deque<EncoderSample> encoder_samples_;
};

}  // namespace state_estimator_hmb

#endif
