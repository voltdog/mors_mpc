#ifndef STATE_ESTIMATOR_HMB_ROBOT_STATE_PUBLICATION_HPP
#define STATE_ESTIMATOR_HMB_ROBOT_STATE_PUBLICATION_HPP

#include "sensor_readiness.hpp"
#include "mors_msgs/robot_state_msg.hpp"

#include <lcm/lcm-cpp.hpp>

#include <optional>
#include <string>

namespace state_estimator_hmb
{

// Called only by the estimator thread. Retains the last successful primary packet.
class RobotStatePublication final
{
public:
    RobotStatePublication(lcm::LCM& publisher, std::string primary_channel);
    void Publish(const std::string& channel, mors_msgs::robot_state_msg message,
                 const SensorReadiness::Readiness& readiness);
    // False means no primary packet exists yet; build one from the last pose.
    bool PublishT265Failure(SensorReadiness::Readiness readiness);

private:
    lcm::LCM& publisher_;
    const std::string primary_channel_;
    std::optional<mors_msgs::robot_state_msg> last_primary_;
};

}  // namespace state_estimator_hmb

#endif
