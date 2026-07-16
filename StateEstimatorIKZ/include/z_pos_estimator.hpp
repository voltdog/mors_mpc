#ifndef STATE_ESTIMATOR_IKZ_Z_POS_ESTIMATOR_HPP_
#define STATE_ESTIMATOR_IKZ_Z_POS_ESTIMATOR_HPP_

#include <array>
#include <cstddef>
#include <memory>
#include <optional>

#include <Eigen/Core>

struct ZPosEstimate
{
    double position_z;
    std::size_t contacts_used;
    bool updated_from_contacts;
};

class ZPosEstimator final
{
public:
    using Contacts = std::array<bool, 4>;

    explicit ZPosEstimator(double initial_contact_z = 0.0);
    ~ZPosEstimator();

    ZPosEstimator(const ZPosEstimator&) = delete;
    ZPosEstimator& operator=(const ZPosEstimator&) = delete;

    [[nodiscard]] std::optional<ZPosEstimate> Update(
        const Eigen::Ref<const Eigen::VectorXd>& joint_positions,
        const Eigen::Matrix3d& world_R_body,
        const Contacts& contacts);

    void Reset() noexcept;
    [[nodiscard]] bool initialized() const noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

#endif  // STATE_ESTIMATOR_IKZ_Z_POS_ESTIMATOR_HPP_
