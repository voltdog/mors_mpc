#include "StateEstimatorHMB/HeightMapBuilder.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

constexpr int64_t kInitialTimestampNs = 1'000'000'000;
constexpr int64_t kFramePeriodNs = 50'000'000;
constexpr double kTolerance = 2e-5;

struct PointCloud
{
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
};

void Check(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void CheckNear(double actual, double expected, const std::string& message)
{
    if (!std::isfinite(actual) || std::fabs(actual - expected) > kTolerance)
    {
        throw std::runtime_error(
            message + ": actual=" + std::to_string(actual) +
            ", expected=" + std::to_string(expected));
    }
}

hmb::RobotStateSnapshot MakeRobotState(double x = 0.0, double y = 0.0)
{
    hmb::RobotStateSnapshot state;
    state.valid = true;
    state.position = {x, y, 0.0};
    state.orientation_quaternion = {0.0, 0.0, 0.0, 1.0};
    return state;
}

PointCloud MakePlane(bool omit_invalid_cell = false)
{
    PointCloud cloud;
    cloud.x.reserve(400);
    cloud.y.reserve(400);
    cloud.z.reserve(400);
    for (int iy = 0; iy < 20; ++iy)
    {
        for (int ix = 0; ix < 20; ++ix)
        {
            if (omit_invalid_cell && ix == 8 && iy == 8)
            {
                continue;
            }
            cloud.x.push_back(static_cast<float>(-0.95 + 0.10 * ix));
            cloud.y.push_back(static_cast<float>(-0.95 + 0.10 * iy));
            cloud.z.push_back(0.0F);
        }
    }
    return cloud;
}

PointCloud MakePoint(double x, double y, double z)
{
    return PointCloud{
        {static_cast<float>(x)},
        {static_cast<float>(y)},
        {static_cast<float>(z)}};
}

hmb::HeightCorrectionObservation MakeCorrection(double first, double second)
{
    hmb::HeightCorrectionObservation observation;
    observation.residuals[0] = first;
    observation.residuals[1] = second;
    observation.valid[0] = true;
    observation.valid[1] = true;
    return observation;
}

void InitializeMap(hmb::HeightMapBuilderNode* builder, bool omit_invalid_cell = false)
{
    const PointCloud plane = MakePlane(omit_invalid_cell);
    Check(
        builder->ProcessCameraPointCloudFrame(
            kInitialTimestampNs,
            MakeRobotState(),
            plane.x,
            plane.y,
            plane.z),
        "Failed to initialize the synthetic map");
}

double ReadHeight(
    const hmb::HeightMapBuilderNode& builder,
    double x,
    double y,
    int64_t query_timestamp_ns)
{
    double height = 0.0;
    Check(
        builder.EstimateFilteredHeightAtWorldXY(
            x, y, query_timestamp_ns, &height),
        "Expected a valid filtered height");
    return height;
}

void TestMeanRegionAndUpdateOrder(const std::string& config_path)
{
    hmb::HeightMapBuilderNode builder(config_path, false);
    InitializeMap(&builder, true);

    CheckNear(
        ReadHeight(builder, 0.25, 0.25, kInitialTimestampNs + 1),
        0.0,
        "Initial map height is wrong");
    double stale_height = 0.0;
    Check(
        !builder.EstimateFilteredHeightAtWorldXY(
            0.25, 0.25, kInitialTimestampNs + 200'000'000, &stale_height),
        "Stale map sample was accepted");

    const PointCloud current_point = MakePoint(0.05, 0.05, 0.20);
    Check(
        builder.ProcessCameraPointCloudFrame(
            kInitialTimestampNs + kFramePeriodNs,
            MakeRobotState(),
            current_point.x,
            current_point.y,
            current_point.z,
            MakeCorrection(0.02, 0.04)),
        "Correction frame failed");

    const double alpha = 1.0 - std::exp(-0.05 / 0.30);
    const double expected_step = alpha * 0.03;
    const int64_t query_timestamp = kInitialTimestampNs + kFramePeriodNs + 1;
    CheckNear(
        ReadHeight(builder, 0.25, 0.25, query_timestamp),
        expected_step,
        "Arithmetic-mean correction has the wrong value");
    CheckNear(
        ReadHeight(builder, 0.35, 0.35, query_timestamp),
        0.0,
        "Cell outside the 0.60 m square was corrected");
    CheckNear(
        ReadHeight(builder, 0.05, 0.05, query_timestamp),
        0.20,
        "Current-frame point was corrected a second time");

    double invalid_height = 0.0;
    Check(
        !builder.EstimateFilteredHeightAtWorldXY(
            -0.15, -0.15, query_timestamp, &invalid_height),
        "Correction made an invalid map cell valid");
}

void TestStepLimitAndSpreadRejection(const std::string& config_path)
{
    {
        hmb::HeightMapBuilderNode builder(config_path, false);
        InitializeMap(&builder);
        const PointCloud point = MakePoint(0.35, 0.35, 0.0);
        Check(
            builder.ProcessCameraPointCloudFrame(
                kInitialTimestampNs + kFramePeriodNs,
                MakeRobotState(),
                point.x,
                point.y,
                point.z,
                MakeCorrection(0.10, 0.10)),
            "Limited correction frame failed");
        CheckNear(
            ReadHeight(
                builder, 0.25, 0.25,
                kInitialTimestampNs + kFramePeriodNs + 1),
            0.005,
            "Per-frame correction limit was not applied");
    }

    {
        hmb::HeightMapBuilderNode builder(config_path, false);
        InitializeMap(&builder);
        const PointCloud point = MakePoint(0.35, 0.35, 0.0);
        Check(
            builder.ProcessCameraPointCloudFrame(
                kInitialTimestampNs + kFramePeriodNs,
                MakeRobotState(),
                point.x,
                point.y,
                point.z,
                MakeCorrection(0.00, 0.04)),
            "Spread-rejection frame failed");
        CheckNear(
            ReadHeight(
                builder, 0.25, 0.25,
                kInitialTimestampNs + kFramePeriodNs + 1),
            0.0,
            "Inconsistent residuals changed the map");
    }

    {
        hmb::HeightMapBuilderNode builder(config_path, false);
        InitializeMap(&builder);
        const PointCloud point = MakePoint(0.35, 0.35, 0.0);
        hmb::HeightCorrectionObservation observation = MakeCorrection(0.04, 0.04);
        observation.valid[1] = false;
        Check(
            builder.ProcessCameraPointCloudFrame(
                kInitialTimestampNs + kFramePeriodNs,
                MakeRobotState(),
                point.x,
                point.y,
                point.z,
                observation),
            "Insufficient-contact frame failed");
        CheckNear(
            ReadHeight(
                builder, 0.25, 0.25,
                kInitialTimestampNs + kFramePeriodNs + 1),
            0.0,
            "A single residual changed the map");
    }

    {
        hmb::HeightMapBuilderNode builder(config_path, false);
        InitializeMap(&builder);
        const PointCloud point = MakePoint(0.35, 0.35, 0.0);
        Check(
            builder.ProcessCameraPointCloudFrame(
                kInitialTimestampNs + kFramePeriodNs,
                MakeRobotState(),
                point.x,
                point.y,
                point.z,
                MakeCorrection(-0.02, -0.04)),
            "Negative correction frame failed");
        const double alpha = 1.0 - std::exp(-0.05 / 0.30);
        CheckNear(
            ReadHeight(
                builder, 0.25, 0.25,
                kInitialTimestampNs + kFramePeriodNs + 1),
            -alpha * 0.03,
            "Negative residual moved the map in the wrong direction");
    }
}

void TestStaleMapAndRollingShift(const std::string& config_path)
{
    {
        hmb::HeightMapBuilderNode builder(config_path, false);
        InitializeMap(&builder);
        const PointCloud point = MakePoint(0.35, 0.35, 0.0);
        const int64_t stale_frame_timestamp = kInitialTimestampNs + 200'000'000;
        Check(
            builder.ProcessCameraPointCloudFrame(
                stale_frame_timestamp,
                MakeRobotState(),
                point.x,
                point.y,
                point.z,
                MakeCorrection(0.10, 0.10)),
            "Stale-map frame failed");
        CheckNear(
            ReadHeight(builder, 0.25, 0.25, stale_frame_timestamp + 1),
            0.0,
            "Stale map residual changed the map");
    }

    {
        hmb::HeightMapBuilderNode builder(config_path, false);
        InitializeMap(&builder);
        const PointCloud point = MakePoint(0.35, 0.35, 0.0);
        Check(
            builder.ProcessCameraPointCloudFrame(
                kInitialTimestampNs + kFramePeriodNs,
                MakeRobotState(0.60, 0.0),
                point.x,
                point.y,
                point.z,
                MakeCorrection(0.02, 0.04)),
            "Rolling correction frame failed");
        const double alpha = 1.0 - std::exp(-0.05 / 0.30);
        CheckNear(
            ReadHeight(
                builder, 0.65, 0.05,
                kInitialTimestampNs + kFramePeriodNs + 1),
            alpha * 0.03,
            "Correction failed after rolling-map shift");
    }
}

}  // namespace

int main()
{
    try
    {
        Check(
            ::setenv(
                "LCM_CONTROL_URL", "udpm://239.255.76.67:17667?ttl=0", 1) == 0,
            "Failed to set LCM_CONTROL_URL");
        Check(
            ::setenv(
                "LCM_VISION_URL", "udpm://239.255.76.67:17668?ttl=0", 1) == 0,
            "Failed to set LCM_VISION_URL");

        const std::string config_path = HEIGHTMAP_BUILDER_CORRECTION_TEST_CONFIG;
        TestMeanRegionAndUpdateOrder(config_path);
        TestStepLimitAndSpreadRejection(config_path);
        TestStaleMapAndRollingShift(config_path);
    }
    catch (const std::exception& error)
    {
        std::cerr << "heightmap_builder_correction_test failed: "
                  << error.what() << '\n';
        return 1;
    }

    std::cout << "heightmap_builder_correction_test passed\n";
    return 0;
}
