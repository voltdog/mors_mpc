#ifndef STATE_ESTIMATOR_HMB_SENSOR_READINESS_CONFIG_HPP
#define STATE_ESTIMATOR_HMB_SENSOR_READINESS_CONFIG_HPP

#include "sensor_readiness.hpp"

#include <yaml-cpp/yaml.h>

namespace state_estimator_hmb
{

SensorReadinessConfig LoadSensorReadinessConfig(const YAML::Node& state_estimator);

}  // namespace state_estimator_hmb

#endif
