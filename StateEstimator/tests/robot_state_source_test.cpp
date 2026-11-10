#include "robot_state_source.hpp"

#include <exception>
#include <iostream>
#include <stdexcept>

namespace
{

using state_estimator_hmb::RobotStateSource;
using state_estimator_hmb::SelectRobotStateSources;
using state_estimator_hmb::T265State;

void Check(bool condition, const char* message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void TestKalmanPrimary()
{
    for (const auto camera : {T265State::WaitingForPose, T265State::Streaming,
                              T265State::Unavailable, T265State::Disconnected})
    {
        const auto sources = SelectRobotStateSources(RobotStateSource::Kalman, true, camera);
        Check(sources.primary == RobotStateSource::Kalman, "Kalman primary changed");
        Check(sources.check == RobotStateSource::T265, "check duplicates Kalman");
        Check(sources.kalman_enabled && !sources.fatal, "Kalman stopped on camera failure");
        Check(sources.t265_valid == (camera == T265State::Streaming),
              "missing or stale camera data can be published");
    }
}

void TestT265Startup()
{
    for (const bool enabled : {false, true})
    {
        for (const auto camera : {T265State::WaitingForPose, T265State::Streaming})
        {
            const auto sources = SelectRobotStateSources(RobotStateSource::T265, enabled, camera);
            Check(sources.primary == RobotStateSource::T265, "premature startup fallback");
            Check(sources.check == RobotStateSource::Kalman, "incorrect check source");
            Check(sources.kalman_enabled == enabled, "Kalman enabled before failure");
            Check(!sources.fatal, "startup is fatal");
            Check(sources.t265_valid == (camera == T265State::Streaming),
                  "camera valid before first pose");
        }

        // Ошибка открытия и ошибка ожидания первого кадра имеют один результат.
        const auto fallback = SelectRobotStateSources(
            RobotStateSource::T265, enabled, T265State::Unavailable);
        Check(fallback.primary == RobotStateSource::Kalman, "startup fallback missing");
        Check(fallback.check == RobotStateSource::T265, "fallback duplicates Kalman");
        Check(fallback.kalman_enabled, "disabled Kalman not enabled for fallback");
        Check(!fallback.t265_valid && !fallback.fatal, "fallback publishes T265 or stops");
    }
}

void TestPrimaryT265Loss()
{
    for (const bool enabled : {false, true})
    {
        // Disconnected остаётся фатальным, даже если state loop пропустил Streaming.
        const auto sources = SelectRobotStateSources(
            RobotStateSource::T265, enabled, T265State::Disconnected);
        Check(sources.fatal, "primary T265 loss is not fatal");
        Check(!sources.t265_valid, "last pose remains valid after disconnect");
        Check(sources.primary == RobotStateSource::T265, "runtime loss switched coordinates");
    }
}

}  // namespace

int main()
{
    try
    {
        TestKalmanPrimary();
        TestT265Startup();
        TestPrimaryT265Loss();
    }
    catch (const std::exception& error)
    {
        std::cerr << "robot_state_source_test failed: " << error.what() << '\n';
        return 1;
    }
    std::cout << "robot_state_source_test passed\n";
    return 0;
}
