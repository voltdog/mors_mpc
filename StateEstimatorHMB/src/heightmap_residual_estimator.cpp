#include "heightmap_residual_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace state_estimator_hmb
{

const HeightmapResidualEstimator::Residuals& HeightmapResidualEstimator::Update(
    const FootPositions& foot_positions_world,
    const TrustCoefficients& trust_coefficients,
    const MapHeights& map_heights) noexcept
{
    for (std::size_t leg = 0; leg < kLegCount; ++leg)
    {
        const FootPosition& foot = foot_positions_world[leg];
        if (!(trust_coefficients[leg] > kMinimumTrust) ||
            !std::all_of(foot.begin(), foot.end(), [](double value)
            {
                return std::isfinite(value);
            }) ||
            !map_heights[leg].has_value() ||
            !std::isfinite(*map_heights[leg]))
        {
            continue;
        }

        residuals_[leg] = foot[2] - *map_heights[leg];
    }

    return residuals_;
}

void HeightmapResidualEstimator::Reset() noexcept
{
    residuals_.fill(0.0);
}

const HeightmapResidualEstimator::Residuals&
HeightmapResidualEstimator::residuals() const noexcept
{
    return residuals_;
}

}  // namespace state_estimator_hmb
