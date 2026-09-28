#ifndef STATE_ESTIMATOR_HMB_Z_POS_ESTIMATOR_HPP
#define STATE_ESTIMATOR_HMB_Z_POS_ESTIMATOR_HPP

#include <array>
#include <cstddef>
#include <memory>
#include <optional>

#include <Eigen/Core>

namespace state_estimator_hmb
{

struct ZPosEstimate
{
    double position_z;
    std::size_t contacts_used;
    bool updated_from_contacts;
};

class ZPosEstimator final
{
public:
    static constexpr std::size_t kLegCount = 4;
    static constexpr std::size_t kMinimumSupportLegs = 2;
    using Contacts = std::array<bool, kLegCount>;
    using GaitPhases = std::array<double, kLegCount>;
    using ContactStates = std::array<int, kLegCount>;

    explicit ZPosEstimator(double initial_contact_z = 0.0);
    ~ZPosEstimator();

    ZPosEstimator(const ZPosEstimator&) = delete;
    ZPosEstimator& operator=(const ZPosEstimator&) = delete;

    [[nodiscard]] std::optional<ZPosEstimate> Update(
        const Eigen::Ref<const Eigen::VectorXd>& joint_positions,
        const Eigen::Matrix3d& world_R_body,
        const Contacts& contacts,
        const GaitPhases& gait_phases,
        const ContactStates& contact_states);

    void Reset() noexcept;
    [[nodiscard]] bool initialized() const noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace state_estimator_hmb

#endif  // STATE_ESTIMATOR_HMB_Z_POS_ESTIMATOR_HPP
