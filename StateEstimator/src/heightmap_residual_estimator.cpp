#include "heightmap_residual_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace state_estimator_hmb
{

const HeightmapResidualEstimator::Estimate& HeightmapResidualEstimator::Update(
    const FootPositions& foot_positions_world,
    const Contacts& contacts,
    const TrustCoefficients& trust_coefficients,
    const MapHeights& map_heights,
    double foot_contact_offset_m) noexcept
{
    estimate_.valid.fill(false);
    if (!std::isfinite(foot_contact_offset_m) || foot_contact_offset_m < 0.0)
    {
        return estimate_;
    }

    for (std::size_t leg = 0; leg < kLegCount; ++leg)
    {
        const FootPosition& foot = foot_positions_world[leg];
        if (!contacts[leg] ||
            !(trust_coefficients[leg] > kMinimumTrust) ||
            !std::all_of(foot.begin(), foot.end(), [](double value)
            {
                return std::isfinite(value);
            }) ||
            !map_heights[leg].has_value() ||
            !std::isfinite(*map_heights[leg]))
        {
            continue;
        }

        estimate_.residuals[leg] =
            foot[2] - foot_contact_offset_m - *map_heights[leg];
        estimate_.valid[leg] = true;
    }

    return estimate_;
}

void HeightmapResidualEstimator::Reset() noexcept
{
    estimate_ = Estimate{};
}

const HeightmapResidualEstimator::Estimate&
HeightmapResidualEstimator::estimate() const noexcept
{
    return estimate_;
}

const HeightmapResidualEstimator::Residuals&
HeightmapResidualEstimator::residuals() const noexcept
{
    return estimate_.residuals;
}

const HeightmapResidualEstimator::Validity&
HeightmapResidualEstimator::validity() const noexcept
{
    return estimate_.valid;
}

}  // namespace state_estimator_hmb
