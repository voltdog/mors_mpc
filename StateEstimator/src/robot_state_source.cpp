#include "robot_state_source.hpp"

#include <stdexcept>
#include <string>

namespace state_estimator_hmb
{

RobotStateSource ParseRobotStateSource(std::string_view value)
{
    if (value == "kalman")
    {
        return RobotStateSource::Kalman;
    }
    if (value == "t265")
    {
        return RobotStateSource::T265;
    }

    throw std::invalid_argument(
        "robot state source must be 'kalman' or 't265'; got '" +
        std::string(value) + "'.");
}

std::string_view ToString(RobotStateSource source) noexcept
{
    switch (source)
    {
        case RobotStateSource::Kalman:
            return "kalman";
        case RobotStateSource::T265:
            return "t265";
    }

    return "unknown";
}

RobotStateSource Other(RobotStateSource source) noexcept
{
    return source == RobotStateSource::Kalman ? RobotStateSource::T265
                                              : RobotStateSource::Kalman;
}

}  // namespace state_estimator_hmb
