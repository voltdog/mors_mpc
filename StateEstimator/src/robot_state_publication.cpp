#include "robot_state_publication.hpp"

#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace state_estimator_hmb
{

RobotStatePublication::RobotStatePublication(lcm::LCM& publisher, std::string primary_channel)
    : publisher_(publisher), primary_channel_(std::move(primary_channel))
{
}

void RobotStatePublication::Publish(
    const std::string& channel, mors_msgs::robot_state_msg message,
    const SensorReadiness::Readiness& readiness)
{
    static_assert(std::size(message.readiness) ==
                  std::tuple_size_v<SensorReadiness::Readiness>);
    std::copy(readiness.begin(), readiness.end(), std::begin(message.readiness));
    if (publisher_.publish(channel, &message) < 0)
        throw std::runtime_error("Failed to publish robot state on " + channel);
    if (channel == primary_channel_)
        last_primary_ = message;
}

bool RobotStatePublication::PublishT265Failure(SensorReadiness::Readiness readiness)
{
    if (!last_primary_)
        return false;
    readiness[18] = false;
    Publish(primary_channel_, *last_primary_, readiness);
    return true;
}

}  // namespace state_estimator_hmb
