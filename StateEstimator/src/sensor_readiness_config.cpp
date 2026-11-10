#include "sensor_readiness_config.hpp"

#include <stdexcept>

namespace state_estimator_hmb
{

SensorReadinessConfig LoadSensorReadinessConfig(const YAML::Node& state_estimator)
{
    SensorReadinessConfig config;
    const auto node = state_estimator["sensor_readiness"];
    if (node && !node.IsMap())
        throw std::invalid_argument("sensor_readiness must be a YAML mapping");
    const auto load = [&node](const char* key, double& value)
    {
        if (node && node[key])
            value = node[key].as<double>();
    };
    load("frequency_window_sec", config.frequency_window_sec);
    load("min_frequency_ratio", config.min_frequency_ratio);
    load("data_timeout_sec", config.data_timeout_sec);
    load("encoder_window_sec", config.encoder_window_sec);
    load("encoder_std_threshold_rad", config.encoder_std_threshold_rad);
    load("depth_frequency_hz", config.depth_frequency_hz);
    load("imu_frequency_hz", config.imu_frequency_hz);
    load("servo_frequency_hz", config.servo_frequency_hz);
    load("contact_frequency_hz", config.contact_frequency_hz);
    load("t265_frequency_hz", config.t265_frequency_hz);
    config.Validate();
    return config;
}

}  // namespace state_estimator_hmb
