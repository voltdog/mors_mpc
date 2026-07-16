#include "heightmap_residual_estimator.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

namespace
{

using state_estimator_hmb::HeightmapResidualEstimator;

constexpr double kTolerance = 1e-12;

void CheckNear(double actual, double expected, const std::string& message)
{
    if (!std::isfinite(actual) || std::fabs(actual - expected) > kTolerance)
    {
        throw std::runtime_error(
            message + ": actual=" + std::to_string(actual) +
            ", expected=" + std::to_string(expected));
    }
}

HeightmapResidualEstimator::FootPositions MakeFootPositions()
{
    return {{{0.10, -0.20, 0.35},
             {0.20, 0.30, 0.42},
             {-0.30, -0.10, 0.18},
             {-0.25, 0.15, 0.27}}};
}

HeightmapResidualEstimator::MapHeights MakeMapHeights()
{
    return {{0.05, 0.12, -0.02, 0.07}};
}

void TestInitialValueAndIndependentUpdate()
{
    HeightmapResidualEstimator estimator;
    for (const double residual : estimator.residuals())
    {
        CheckNear(residual, 0.0, "Residual was not initialized to zero");
    }

    auto trust = HeightmapResidualEstimator::TrustCoefficients{};
    trust[1] = 1.0F;
    const auto& residuals = estimator.Update(
        MakeFootPositions(), trust, MakeMapHeights());

    CheckNear(residuals[0], 0.0, "Untrusted R1 residual changed");
    CheckNear(residuals[1], 0.30, "Trusted L1 residual was not updated");
    CheckNear(residuals[2], 0.0, "Untrusted R2 residual changed");
    CheckNear(residuals[3], 0.0, "Untrusted L2 residual changed");
}

void TestMultipleLegsAndOrder()
{
    HeightmapResidualEstimator estimator;
    const HeightmapResidualEstimator::TrustCoefficients trust{
        1.0F, 0.0F, 1.0F, 1.0F};
    const auto& residuals = estimator.Update(
        MakeFootPositions(), trust, MakeMapHeights());

    CheckNear(residuals[0], 0.30, "R1 residual has the wrong index");
    CheckNear(residuals[1], 0.0, "Untrusted L1 residual changed");
    CheckNear(residuals[2], 0.20, "R2 residual has the wrong index");
    CheckNear(residuals[3], 0.20, "L2 residual has the wrong index");
}

void TestHeldValuesAndRecovery()
{
    HeightmapResidualEstimator estimator;
    auto positions = MakeFootPositions();
    auto heights = MakeMapHeights();
    HeightmapResidualEstimator::TrustCoefficients trust{
        1.0F, 1.0F, 1.0F, 1.0F};
    static_cast<void>(estimator.Update(positions, trust, heights));
    const auto previous = estimator.residuals();

    trust[0] = HeightmapResidualEstimator::kMinimumTrust;
    trust[1] = 0.5F;
    positions[2][0] = std::numeric_limits<double>::quiet_NaN();
    heights[3] = std::nullopt;
    const auto& held = estimator.Update(positions, trust, heights);
    for (std::size_t leg = 0; leg < held.size(); ++leg)
    {
        CheckNear(held[leg], previous[leg], "Invalid input changed a held residual");
    }

    positions = MakeFootPositions();
    heights = MakeMapHeights();
    heights[2] = std::numeric_limits<double>::quiet_NaN();
    trust.fill(1.0F);
    const auto& held_nan_height = estimator.Update(positions, trust, heights);
    CheckNear(
        held_nan_height[2], previous[2],
        "NaN map height changed a held residual");

    heights[2] = 0.08;
    const auto& recovered = estimator.Update(positions, trust, heights);
    CheckNear(recovered[2], 0.10, "Residual did not recover after invalid map data");

    estimator.Reset();
    for (const double residual : estimator.residuals())
    {
        CheckNear(residual, 0.0, "Reset did not clear residuals");
    }
}

}  // namespace

int main()
{
    try
    {
        TestInitialValueAndIndependentUpdate();
        TestMultipleLegsAndOrder();
        TestHeldValuesAndRecovery();
    }
    catch (const std::exception& error)
    {
        std::cerr << "heightmap_residual_estimator_test failed: " << error.what() << '\n';
        return 1;
    }

    std::cout << "heightmap_residual_estimator_test passed\n";
    return 0;
}
