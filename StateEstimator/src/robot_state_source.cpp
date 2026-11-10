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

RobotStateSources SelectRobotStateSources(
    RobotStateSource requested, bool kalman_enabled, T265State t265_state) noexcept
{
    const bool fallback = requested == RobotStateSource::T265 &&
                          t265_state == T265State::Unavailable;
    const auto primary = fallback ? RobotStateSource::Kalman : requested;
    return {
        primary,
        Other(primary),
        kalman_enabled || fallback,
        t265_state == T265State::Streaming,
        requested == RobotStateSource::T265 && t265_state == T265State::Disconnected,
    };
}

}  // namespace state_estimator_hmb
