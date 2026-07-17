#ifndef STATE_ESTIMATOR_HMB_HEIGHTMAP_RESIDUAL_ESTIMATOR_HPP
#define STATE_ESTIMATOR_HMB_HEIGHTMAP_RESIDUAL_ESTIMATOR_HPP

#include <array>
#include <cstddef>
#include <optional>

namespace state_estimator_hmb
{

class HeightmapResidualEstimator final
{
public:
    static constexpr std::size_t kLegCount = 4;
    static constexpr float kMinimumTrust = 0.99F;

    using FootPosition = std::array<double, 3>;
    using FootPositions = std::array<FootPosition, kLegCount>;
    using Contacts = std::array<bool, kLegCount>;
    using TrustCoefficients = std::array<float, kLegCount>;
    using MapHeights = std::array<std::optional<double>, kLegCount>;
    using Residuals = std::array<double, kLegCount>;
    using Validity = std::array<bool, kLegCount>;

    struct Estimate
    {
        Residuals residuals{};
        Validity valid{};
    };

    [[nodiscard]] const Estimate& Update(
        const FootPositions& foot_positions_world,
        const Contacts& contacts,
        const TrustCoefficients& trust_coefficients,
        const MapHeights& map_heights,
        double foot_contact_offset_m) noexcept;

    void Reset() noexcept;
    [[nodiscard]] const Estimate& estimate() const noexcept;
    [[nodiscard]] const Residuals& residuals() const noexcept;
    [[nodiscard]] const Validity& validity() const noexcept;

private:
    Estimate estimate_{};
};

}  // namespace state_estimator_hmb

#endif  // STATE_ESTIMATOR_HMB_HEIGHTMAP_RESIDUAL_ESTIMATOR_HPP
