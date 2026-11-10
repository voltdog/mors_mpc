#include "sensor_readiness_config.hpp"

#include <array>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
using namespace state_estimator_hmb;

void Check(bool condition, const char* message)
{
    if (!condition)
        throw std::runtime_error(message);
}

template <typename Function>
void Rejects(Function function)
{
    try
    {
        function();
    }
    catch (const std::exception&)
    {
        return;
    }
    throw std::runtime_error("invalid readiness configuration accepted");
}

void TestLoading()
{
    auto defaults = LoadSensorReadinessConfig(YAML::Load("{}"));
    Check(defaults.imu_frequency_hz == 500 && defaults.servo_frequency_hz == 1000 &&
          defaults.contact_frequency_hz == 600 && defaults.t265_frequency_hz == 200 &&
          defaults.depth_frequency_hz == 30 && defaults.frequency_window_sec == 0.5 &&
          defaults.min_frequency_ratio == 0.5 && defaults.data_timeout_sec == 0.5 &&
          defaults.encoder_window_sec == 0.5 && defaults.encoder_std_threshold_rad == 1e-6,
          "incorrect defaults");
    const auto root = YAML::Load(R"(
sensor_readiness:
  frequency_window_sec: 1.0
  min_frequency_ratio: 0.8
  data_timeout_sec: 0.2
  encoder_window_sec: 0.3
  encoder_std_threshold_rad: 0.0001
  depth_frequency_hz: 15
  imu_frequency_hz: 400
  servo_frequency_hz: 900
  contact_frequency_hz: 500
  t265_frequency_hz: 100
)");
    const auto config = LoadSensorReadinessConfig(root);
    Check(config.frequency_window_sec == 1 && config.min_frequency_ratio == 0.8 &&
          config.data_timeout_sec == 0.2 && config.encoder_window_sec == 0.3 &&
          config.encoder_std_threshold_rad == 0.0001 && config.depth_frequency_hz == 15 &&
          config.imu_frequency_hz == 400 && config.servo_frequency_hz == 900 &&
          config.contact_frequency_hz == 500 && config.t265_frequency_hz == 100,
          "YAML overrides ignored");
}

void TestInvalidValues()
{
    const std::array<double SensorReadinessConfig::*, 10> fields{
        &SensorReadinessConfig::frequency_window_sec,
        &SensorReadinessConfig::min_frequency_ratio,
        &SensorReadinessConfig::data_timeout_sec,
        &SensorReadinessConfig::encoder_window_sec,
        &SensorReadinessConfig::encoder_std_threshold_rad,
        &SensorReadinessConfig::depth_frequency_hz,
        &SensorReadinessConfig::imu_frequency_hz,
        &SensorReadinessConfig::servo_frequency_hz,
        &SensorReadinessConfig::contact_frequency_hz,
        &SensorReadinessConfig::t265_frequency_hz};
    for (const auto field : fields)
        for (const double invalid : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                                     std::numeric_limits<double>::quiet_NaN()})
            Rejects([&]
            {
                SensorReadinessConfig config;
                config.*field = invalid;
                SensorReadiness sensor(config);
            });
    Rejects([]
    {
        auto root = YAML::Load("sensor_readiness: {min_frequency_ratio: 1.1}");
        LoadSensorReadinessConfig(root);
    });
    for (const char* yaml : {"sensor_readiness: []", "sensor_readiness: true",
                             "sensor_readiness: {frequency_window_sec: [1]}",
                             "sensor_readiness: {servo_frequency_hz: bad}"})
        Rejects([&] { LoadSensorReadinessConfig(YAML::Load(yaml)); });
}
}  // namespace

int main()
{
    try
    {
        TestLoading();
        TestInvalidValues();
    }
    catch (const std::exception& error)
    {
        std::cerr << "sensor_readiness_config_test: " << error.what() << '\n';
        return 1;
    }
    std::cout << "sensor_readiness_config_test passed\n";
}
