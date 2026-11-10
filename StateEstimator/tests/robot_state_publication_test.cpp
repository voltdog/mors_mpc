#include "robot_state_publication.hpp"
#include "robot_state_source.hpp"

#include <algorithm>
#include <array>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using namespace state_estimator_hmb;

void Check(bool condition, const char* message)
{
    if (!condition)
        throw std::runtime_error(message);
}

struct Receiver
{
    std::vector<std::string> channels;
    std::vector<mors_msgs::robot_state_msg> messages;
    void OnMessage(const lcm::ReceiveBuffer*, const std::string& channel,
                   const mors_msgs::robot_state_msg* message)
    {
        channels.push_back(channel);
        messages.push_back(*message);
    }
};

std::vector<int8_t> Encode(const mors_msgs::robot_state_msg& message)
{
    std::vector<int8_t> bytes(message.getEncodedSize());
    Check(message.encode(bytes.data(), 0, bytes.size()) > 0, "encode failed");
    return bytes;
}

void TestPublicationAndFailure()
{
    lcm::LCM bus("memq://");
    Check(bus.good(), "LCM memq unavailable");
    Receiver receiver;
    bus.subscribe("PRIMARY", &Receiver::OnMessage, &receiver);
    bus.subscribe("CHECK", &Receiver::OnMessage, &receiver);
    RobotStatePublication publication(bus, "PRIMARY");
    SensorReadiness::Readiness readiness;
    for (std::size_t i = 0; i < readiness.size(); ++i)
        readiness[i] = i % 2 == 0;
    Check(!publication.PublishT265Failure(readiness), "fabricated a state before first pose");
    Check(bus.handleTimeout(0) == 0, "published before first state");

    mors_msgs::robot_state_msg primary{};
    primary.timestamp = 123456789;
    primary.body.position[0] = 1.23;
    primary.body.orientation_quaternion[3] = 1.0;
    primary.legs.r1_grf[2] = 25.0;
    primary.legs.contact_states[1] = 1;
    primary.joints.position[11] = 0.4F;
    publication.Publish("PRIMARY", primary, readiness);
    auto check = primary;
    check.timestamp = 987654321;
    check.body.position[0] = -2.34;
    publication.Publish("CHECK", check, readiness);
    Check(bus.handleTimeout(100) > 0 && bus.handleTimeout(100) > 0,
          "normal state not delivered");
    Check(receiver.messages.size() == 2, "wrong normal publication count");
    Check(std::equal(readiness.begin(), readiness.end(), receiver.messages[0].readiness) &&
          std::equal(readiness.begin(), readiness.end(), receiver.messages[1].readiness),
          "channels do not share readiness snapshot");

    const auto policy = SelectRobotStateSources(
        RobotStateSource::T265, true, T265State::Disconnected);
    Check(policy.fatal, "T265 loss not fatal");
    readiness.fill(true);
    readiness[1] = false;
    Check(publication.PublishT265Failure(readiness), "final state was not published");
    // Publication completes before the caller stops the estimator.
    Check(bus.handleTimeout(100) > 0, "final state not delivered");
    Check(receiver.messages.size() == 3 && receiver.channels.back() == "PRIMARY",
          "final state sent to the wrong channel");
    readiness[18] = false;
    std::copy(readiness.begin(), readiness.end(), primary.readiness);
    Check(Encode(receiver.messages.back()) == Encode(primary),
          "final packet changed primary state/timestamp or used check state");
    Check(bus.handleTimeout(0) == 0, "unexpected additional final packet");

    // Even if the camera fails before the first estimator tick, the first packet
    // built from its retained pose can carry the failure snapshot unchanged.
    RobotStatePublication early(bus, "PRIMARY");
    Check(!early.PublishT265Failure(readiness), "early failure has a cached state");
    early.Publish("PRIMARY", primary, readiness);
    Check(bus.handleTimeout(100) > 0, "early failure state not delivered");
    Check(Encode(receiver.messages.back()) == Encode(primary), "early failure packet changed");
}
}  // namespace

int main()
{
    try
    {
        TestPublicationAndFailure();
    }
    catch (const std::exception& error)
    {
        std::cerr << "robot_state_publication_test: " << error.what() << '\n';
        return 1;
    }
    std::cout << "robot_state_publication_test passed\n";
}
