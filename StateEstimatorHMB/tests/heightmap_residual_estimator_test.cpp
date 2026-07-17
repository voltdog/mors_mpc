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
constexpr double kFootContactOffsetM = 0.015;

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
    auto contacts = HeightmapResidualEstimator::Contacts{};
    trust[1] = 1.0F;
    contacts[1] = true;
    const auto& estimate = estimator.Update(
        MakeFootPositions(), contacts, trust, MakeMapHeights(), kFootContactOffsetM);
    const auto& residuals = estimate.residuals;

    CheckNear(residuals[0], 0.0, "Untrusted R1 residual changed");
    CheckNear(residuals[1], 0.285, "Trusted L1 residual was not updated");
    CheckNear(residuals[2], 0.0, "Untrusted R2 residual changed");
    CheckNear(residuals[3], 0.0, "Untrusted L2 residual changed");
    for (std::size_t leg = 0; leg < estimate.valid.size(); ++leg)
    {
        const bool expected = leg == 1;
        if (estimate.valid[leg] != expected)
        {
            throw std::runtime_error("Current-frame validity has the wrong leg mask");
        }
    }
}

void TestMultipleLegsAndOrder()
{
    HeightmapResidualEstimator estimator;
    const HeightmapResidualEstimator::Contacts contacts{true, true, true, true};
    const HeightmapResidualEstimator::TrustCoefficients trust{
        1.0F, 0.0F, 1.0F, 1.0F};
    const auto& estimate = estimator.Update(
        MakeFootPositions(), contacts, trust, MakeMapHeights(), kFootContactOffsetM);
    const auto& residuals = estimate.residuals;

    CheckNear(residuals[0], 0.285, "R1 residual has the wrong index");
    CheckNear(residuals[1], 0.0, "Untrusted L1 residual changed");
    CheckNear(residuals[2], 0.185, "R2 residual has the wrong index");
    CheckNear(residuals[3], 0.185, "L2 residual has the wrong index");
    if (!estimate.valid[0] || estimate.valid[1] ||
        !estimate.valid[2] || !estimate.valid[3])
    {
        throw std::runtime_error("Trust filtering produced an invalid current-frame mask");
    }
}

void TestHeldValuesAndRecovery()
{
    HeightmapResidualEstimator estimator;
    auto positions = MakeFootPositions();
    auto heights = MakeMapHeights();
    HeightmapResidualEstimator::Contacts contacts{true, true, true, true};
    HeightmapResidualEstimator::TrustCoefficients trust{
        1.0F, 1.0F, 1.0F, 1.0F};
    static_cast<void>(estimator.Update(
        positions, contacts, trust, heights, kFootContactOffsetM));
    const auto previous = estimator.residuals();

    contacts[0] = false;
    trust[0] = HeightmapResidualEstimator::kMinimumTrust;
    trust[1] = 0.5F;
    positions[2][0] = std::numeric_limits<double>::quiet_NaN();
    heights[3] = std::nullopt;
    const auto& held = estimator.Update(
        positions, contacts, trust, heights, kFootContactOffsetM);
    for (std::size_t leg = 0; leg < held.residuals.size(); ++leg)
    {
        CheckNear(
            held.residuals[leg], previous[leg],
            "Invalid input changed a held residual");
        if (held.valid[leg])
        {
            throw std::runtime_error("Invalid input retained current-frame validity");
        }
    }

    positions = MakeFootPositions();
    heights = MakeMapHeights();
    heights[2] = std::numeric_limits<double>::quiet_NaN();
    contacts.fill(true);
    trust.fill(1.0F);
    const auto& held_nan_height = estimator.Update(
        positions, contacts, trust, heights, kFootContactOffsetM);
    CheckNear(
        held_nan_height.residuals[2], previous[2],
        "NaN map height changed a held residual");
    if (held_nan_height.valid[2])
    {
        throw std::runtime_error("NaN map height was marked valid");
    }

    heights[2] = 0.08;
    const auto& recovered = estimator.Update(
        positions, contacts, trust, heights, kFootContactOffsetM);
    CheckNear(
        recovered.residuals[2], 0.085,
        "Residual did not recover after invalid map data");
    if (!recovered.valid[2])
    {
        throw std::runtime_error("Recovered residual was not marked valid");
    }

    estimator.Reset();
    for (const double residual : estimator.residuals())
    {
        CheckNear(residual, 0.0, "Reset did not clear residuals");
    }
    for (const bool valid : estimator.validity())
    {
        if (valid)
        {
            throw std::runtime_error("Reset did not clear residual validity");
        }
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
