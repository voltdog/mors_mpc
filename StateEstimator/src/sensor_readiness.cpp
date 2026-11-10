#include "sensor_readiness.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace state_estimator_hmb
{
namespace
{

double Seconds(SensorReadiness::Clock::duration duration)
{
    return std::chrono::duration<double>(duration).count();
}

void RequirePositive(double value, const char* name)
{
    if (!std::isfinite(value) || value <= 0.0)
    {
        throw std::invalid_argument(std::string("sensor_readiness.") + name +
                                    " must be finite and positive");
    }
}

}  // namespace

void SensorReadinessConfig::Validate() const
{
    RequirePositive(frequency_window_sec, "frequency_window_sec");
    RequirePositive(min_frequency_ratio, "min_frequency_ratio");
    if (min_frequency_ratio > 1.0)
        throw std::invalid_argument("sensor_readiness.min_frequency_ratio must be <= 1");
    RequirePositive(data_timeout_sec, "data_timeout_sec");
    RequirePositive(encoder_window_sec, "encoder_window_sec");
    RequirePositive(encoder_std_threshold_rad, "encoder_std_threshold_rad");
    RequirePositive(depth_frequency_hz, "depth_frequency_hz");
    RequirePositive(imu_frequency_hz, "imu_frequency_hz");
    RequirePositive(servo_frequency_hz, "servo_frequency_hz");
    RequirePositive(contact_frequency_hz, "contact_frequency_hz");
    RequirePositive(t265_frequency_hz, "t265_frequency_hz");
}

SensorReadiness::SensorReadiness(SensorReadinessConfig config)
    : config_(std::move(config))
{
    config_.Validate();
}

void SensorReadiness::Prune(Stream& stream, TimePoint now)
{
    while (!stream.events.empty() &&
           Seconds(now - stream.events.front()) >= config_.frequency_window_sec)
    {
        stream.events.pop_front();
    }
}

void SensorReadiness::Tick(Stream& stream, TimePoint now)
{
    if (stream.last && Seconds(now - *stream.last) >= config_.data_timeout_sec)
    {
        stream.started.reset();
        stream.events.clear();
    }
    if (!stream.started)
        stream.started = now;
    stream.last = now;
    Prune(stream, now);
    stream.events.push_back(now);
}

bool SensorReadiness::StreamReady(Stream& stream, double frequency_hz, TimePoint now)
{
    Prune(stream, now);
    return stream.started && stream.last &&
           Seconds(now - *stream.started) >= config_.frequency_window_sec &&
           Seconds(now - *stream.last) < config_.data_timeout_sec &&
           static_cast<double>(stream.events.size()) / config_.frequency_window_sec >=
               frequency_hz * config_.min_frequency_ratio;
}

void SensorReadiness::OnImu(std::span<const float, 4> quaternion, TimePoint now)
{
    std::lock_guard lock(mutex_);
    Tick(imu_, now);
    imu_valid_ = std::all_of(quaternion.begin(), quaternion.end(),
                            [](float x) { return std::isfinite(x); }) &&
                 std::any_of(quaternion.begin(), quaternion.end(),
                             [](float x) { return x != 0.0F; });
}

void SensorReadiness::PruneEncoders(TimePoint now)
{
    while (!encoder_samples_.empty() &&
           Seconds(now - encoder_samples_.front().time) >= config_.encoder_window_sec)
    {
        encoder_samples_.pop_front();
    }
}

void SensorReadiness::OnServo(std::span<const float, 12> position, TimePoint now)
{
    std::lock_guard lock(mutex_);
    if (servo_.last && Seconds(now - *servo_.last) >= config_.data_timeout_sec)
    {
        encoder_samples_.clear();
        encoders_started_.reset();
    }
    Tick(servo_, now);
    if (!encoders_started_)
        encoders_started_ = now;
    PruneEncoders(now);
    EncoderSample sample{now, {}};
    std::copy(position.begin(), position.end(), sample.position.begin());
    encoder_samples_.push_back(sample);
}

void SensorReadiness::OnContacts(std::span<const int8_t, 4> readiness, TimePoint now)
{
    std::lock_guard lock(mutex_);
    Tick(contacts_, now);
    std::transform(readiness.begin(), readiness.end(), contacts_valid_.begin(),
                   [](int8_t x) { return x != 0; });
}

void SensorReadiness::OnDepthFrame(TimePoint now)
{
    std::lock_guard lock(mutex_);
    Tick(depth_, now);
    depth_healthy_ = true;
}

void SensorReadiness::OnT265Pose(TimePoint now)
{
    std::lock_guard lock(mutex_);
    Tick(t265_, now);
    t265_healthy_ = !t265_startup_failed_;
}

void SensorReadiness::SetDepthHealthy(bool healthy)
{
    std::lock_guard lock(mutex_);
    depth_healthy_ = healthy;
}

void SensorReadiness::SetT265State(T265State state)
{
    std::lock_guard lock(mutex_);
    if (state == T265State::Unavailable)
        t265_startup_failed_ = true;
    t265_healthy_ = state == T265State::Streaming && !t265_startup_failed_;
}

SensorReadiness::Readiness SensorReadiness::GetReadiness(TimePoint now)
{
    std::lock_guard lock(mutex_);
    Readiness result{};
    result[0] = StreamReady(depth_, config_.depth_frequency_hz, now) &&
                depth_healthy_;
    result[1] = StreamReady(imu_, config_.imu_frequency_hz, now) &&
                imu_valid_;
    result[18] = StreamReady(t265_, config_.t265_frequency_hz, now) &&
                 t265_healthy_;
    const bool contacts_ready = StreamReady(contacts_, config_.contact_frequency_hz, now);
    for (std::size_t leg = 0; leg < 4; ++leg)
        result[14 + leg] = contacts_ready && contacts_valid_[leg];

    const bool servo_ready = StreamReady(servo_, config_.servo_frequency_hz, now);
    PruneEncoders(now);
    if (!servo_ready || !encoders_started_ ||
        Seconds(now - *encoders_started_) < config_.encoder_window_sec ||
        encoder_samples_.size() < 2)
        return result;

    // Welford avoids cancellation for small noise around a nonzero joint angle.
    std::array<double, 12> mean{};
    std::array<double, 12> m2{};
    std::array<bool, 12> valid;
    valid.fill(true);
    std::size_t count = 0;
    for (const auto& sample : encoder_samples_)
    {
        ++count;
        for (std::size_t joint = 0; joint < 12; ++joint)
        {
            const double value = sample.position[joint];
            if (!std::isfinite(value))
            {
                valid[joint] = false;
                continue;
            }
            const double delta = value - mean[joint];
            mean[joint] += delta / static_cast<double>(count);
            m2[joint] += delta * (value - mean[joint]);
        }
    }
    const double threshold_squared = config_.encoder_std_threshold_rad *
                                     config_.encoder_std_threshold_rad;
    for (std::size_t joint = 0; joint < 12; ++joint)
        result[2 + joint] = valid[joint] &&
            m2[joint] / static_cast<double>(count) > threshold_squared;
    return result;
}

}  // namespace state_estimator_hmb
