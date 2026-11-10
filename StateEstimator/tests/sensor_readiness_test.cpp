#include "sensor_readiness.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>

namespace
{
using namespace state_estimator_hmb;
using Readiness = SensorReadiness::Readiness;

void Check(bool condition, const char* message)
{
    if (!condition)
        throw std::runtime_error(message);
}

SensorReadiness::TimePoint At(int milliseconds)
{
    return SensorReadiness::TimePoint{} + std::chrono::milliseconds(milliseconds);
}

bool All(const Readiness& flags)
{
    return std::all_of(flags.begin(), flags.end(), [](bool x) { return x; });
}

void Feed(SensorReadiness& sensor, int first, int last, int frozen_joint = -1,
          float amplitude = 1.0e-4F)
{
    const std::array<float, 4> quaternion{0, 0, 0, 1};
    const std::array<int8_t, 4> contacts{1, 1, 1, 1};
    for (int i = first; i <= last; ++i)
    {
        std::array<float, 12> position;
        position.fill(1.5F + (i % 2 ? amplitude : -amplitude));
        if (frozen_joint >= 0)
            position[frozen_joint] = 1.5F;
        sensor.OnServo(position, At(i));
        if (i % 2 == 0)
            sensor.OnImu(quaternion, At(i));
        if (i == 0 || i * 600 / 1000 != (i - 1) * 600 / 1000)
            sensor.OnContacts(contacts, At(i));
        if (i == 0 || i * 30 / 1000 != (i - 1) * 30 / 1000)
            sensor.OnDepthFrame(At(i));
        if (i % 5 == 0)
            sensor.OnT265Pose(At(i));
    }
}

void TestWarmupAndTimeout()
{
    SensorReadiness sensor;
    Check(sensor.GetReadiness(At(0)) == Readiness{}, "ready before first data");
    Feed(sensor, 0, 499);
    Check(sensor.GetReadiness(At(499)) == Readiness{}, "ready before full window");
    Feed(sensor, 500, 500);
    Check(All(sensor.GetReadiness(At(500))), "nominal sensor rates not ready");
    Check(sensor.GetReadiness(At(1000)) == Readiness{}, "timeout retains ready flags");
    Feed(sensor, 2000, 2499);
    Check(sensor.GetReadiness(At(2499)) == Readiness{}, "recovery skips warmup");
    Feed(sensor, 2500, 2500);
    Check(All(sensor.GetReadiness(At(2500))), "sensors do not recover");
}

void TestFrequencyBoundaryAndPolling()
{
    SensorReadinessConfig config;
    config.depth_frequency_hz = 20.0;
    SensorReadiness sensor(config);
    for (int i = 0; i <= 500; i += 100)
        sensor.OnDepthFrame(At(i));
    Check(sensor.GetReadiness(At(500))[0], "exact frequency threshold rejected");
    for (int i = 0; i < 100; ++i)
        Check(!sensor.GetReadiness(At(600))[0], "polling counted as an input event");
    for (int i = 600; i <= 1100; i += 100)
        sensor.OnDepthFrame(At(i));
    Check(sensor.GetReadiness(At(1100))[0], "frequency does not recover");

    config.min_frequency_ratio = 0.8;
    SensorReadiness strict(config);
    for (int i = 0; i <= 500; i += 100)
        strict.OnDepthFrame(At(i));
    Check(!strict.GetReadiness(At(500))[0], "configured ratio ignored");
}

void TestImuAndContactFlags()
{
    SensorReadiness sensor;
    Feed(sensor, 0, 500);
    const std::array<float, 4> zero{};
    sensor.OnImu(zero, At(500));
    auto expected = Readiness{};
    expected.fill(true);
    expected[1] = false;
    Check(sensor.GetReadiness(At(500)) == expected, "zero quaternion mapping");
    for (const float invalid : {std::numeric_limits<float>::quiet_NaN(),
                                std::numeric_limits<float>::infinity()})
    {
        const std::array<float, 4> q{invalid, 0, 0, 1};
        sensor.OnImu(q, At(500));
        Check(!sensor.GetReadiness(At(500))[1], "nonfinite quaternion accepted");
    }
    const std::array<float, 4> good{0, 0, 0, 1};
    sensor.OnImu(good, At(500));
    Check(All(sensor.GetReadiness(At(500))), "IMU does not recover");

    for (int leg = 0; leg < 4; ++leg)
    {
        std::array<int8_t, 4> flags{1, 1, 1, 1};
        flags[leg] = 0;
        sensor.OnContacts(flags, At(500));
        expected.fill(true);
        expected[14 + leg] = false;
        Check(sensor.GetReadiness(At(500)) == expected, "contact readiness mapping");
    }
}

void TestEncoders()
{
    for (int joint = 0; joint < 12; ++joint)
    {
        SensorReadiness sensor;
        Feed(sensor, 0, 500);
        Check(All(sensor.GetReadiness(At(500))), "noisy encoders not ready");
        Feed(sensor, 501, 1001, joint);
        auto expected = Readiness{};
        expected.fill(true);
        expected[2 + joint] = false;
        Check(sensor.GetReadiness(At(1001)) == expected, "frozen encoder mapping");
        Feed(sensor, 1002, 1501);
        Check(All(sensor.GetReadiness(At(1501))), "encoder does not recover");
    }

    SensorReadinessConfig config;
    config.encoder_std_threshold_rad = 0.125;
    SensorReadiness boundary(config);
    Feed(boundary, 0, 500, -1, 0.125F);
    const auto flags = boundary.GetReadiness(At(500));
    Check(std::none_of(flags.begin() + 2, flags.begin() + 14,
                       [](bool x) { return x; }), "equal noise threshold accepted");
    config.encoder_std_threshold_rad = 0.0625;
    SensorReadiness noisy(config);
    Feed(noisy, 0, 500, -1, 0.125F);
    Check(All(noisy.GetReadiness(At(500))), "noise threshold configuration ignored");

    std::array<float, 12> position;
    position.fill(1.5F);
    position[4] = std::numeric_limits<float>::quiet_NaN();
    noisy.OnServo(position, At(500));
    Check(!noisy.GetReadiness(At(500))[6], "NaN encoder accepted");
    position[4] = std::numeric_limits<float>::infinity();
    noisy.OnServo(position, At(500));
    Check(!noisy.GetReadiness(At(500))[6], "infinite encoder accepted");
    Feed(noisy, 501, 1001, -1, 0.125F);
    Check(All(noisy.GetReadiness(At(1001))), "nonfinite encoder does not recover");
}

void TestSharedStreamLoss()
{
    SensorReadiness servo_only;
    std::array<float, 12> position;
    for (int i = 0; i <= 500; ++i)
    {
        position.fill(i % 2 ? 1.0F : 1.001F);
        servo_only.OnServo(position, At(i));
    }
    auto expected = Readiness{};
    std::fill(expected.begin() + 2, expected.begin() + 14, true);
    Check(servo_only.GetReadiness(At(500)) == expected, "servo stream mapping");
    Check(servo_only.GetReadiness(At(1000)) == Readiness{}, "servo timeout mapping");
    // Old noise must not make a constant encoder ready after reconnection.
    position.fill(1.0F);
    for (int i = 2000; i <= 2500; ++i)
        servo_only.OnServo(position, At(i));
    Check(servo_only.GetReadiness(At(2500)) == Readiness{}, "old noise survived timeout");

    SensorReadiness contacts_only;
    const std::array<int8_t, 4> contacts{1, 1, 1, 1};
    for (int i = 0; i <= 500; ++i)
        contacts_only.OnContacts(contacts, At(i));
    expected.fill(false);
    std::fill(expected.begin() + 14, expected.begin() + 18, true);
    Check(contacts_only.GetReadiness(At(500)) == expected, "contact stream mapping");
    Check(contacts_only.GetReadiness(At(1000)) == Readiness{}, "contact timeout mapping");
}

void TestCameras()
{
    SensorReadiness sensor;
    Feed(sensor, 0, 500);
    sensor.SetDepthHealthy(false);
    sensor.SetT265State(T265State::Disconnected);
    const auto flags = sensor.GetReadiness(At(500));
    Check(!flags[0] && !flags[18], "camera failure not reflected immediately");

    Feed(sensor, 501, 1001);
    sensor.SetT265State(T265State::Streaming);
    Check(sensor.GetReadiness(At(1001))[0], "depth does not recover");
    Check(sensor.GetReadiness(At(1001))[18], "T265 does not recover after disconnect");

    sensor.SetT265State(T265State::Unavailable);
    Feed(sensor, 1002, 1502);
    sensor.SetT265State(T265State::Streaming);
    Check(!sensor.GetReadiness(At(1502))[18], "startup T265 failure not latched");
}

void TestConcurrentAccess()
{
    SensorReadiness sensor;
    std::atomic_bool start{false};
    std::vector<std::thread> threads;
    for (int stream = 0; stream < 5; ++stream)
        threads.emplace_back([&, stream]
        {
            while (!start.load())
                std::this_thread::yield();
            for (int i = 0; i < 1000; ++i)
            {
                const std::array<float, 4> q{0, 0, 0, 1};
                const std::array<float, 12> p{};
                const std::array<int8_t, 4> c{};
                switch (stream)
                {
                    case 0: sensor.OnImu(q); break;
                    case 1: sensor.OnServo(p); break;
                    case 2: sensor.OnContacts(c); break;
                    case 3: sensor.OnDepthFrame(); break;
                    case 4: sensor.OnT265Pose(); break;
                }
                static_cast<void>(sensor.GetReadiness());
            }
        });
    start.store(true);
    for (auto& thread : threads)
        thread.join();
}
}  // namespace

int main()
{
    try
    {
        TestWarmupAndTimeout();
        TestFrequencyBoundaryAndPolling();
        TestImuAndContactFlags();
        TestEncoders();
        TestSharedStreamLoss();
        TestCameras();
        TestConcurrentAccess();
    }
    catch (const std::exception& error)
    {
        std::cerr << "sensor_readiness_test: " << error.what() << '\n';
        return 1;
    }
    std::cout << "sensor_readiness_test passed\n";
}
