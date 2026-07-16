#include "z_pos_estimator.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "Robot.hpp"
#include "reliable_contact.hpp"

namespace state_estimator_hmb
{
namespace
{

constexpr Eigen::Index kNumJoints = 12;
constexpr float kMinimumTrust = 0.99F;

// Index by servo leg order (R1, L1, R2, L2); values refer to Pinocchio's
// toe order (L1, L2, R1, R2).
constexpr std::array<std::size_t, ZPosEstimator::kLegCount> kServoToPinocchio{
    2, 0, 3, 1};

}  // namespace

class ZPosEstimator::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit Impl(double initial_contact_z)
        : initial_contact_z_(initial_contact_z)
    {
        if (!std::isfinite(initial_contact_z_))
        {
            throw std::invalid_argument("initial_contact_z must be finite");
        }

        robot_.BuildPinocchioModel();
        q_ = Eigen::VectorXd::Zero(robot_.nq);
        v_ = Eigen::VectorXd::Zero(robot_.nv);
        if (q_.size() < 19 || v_.size() < 18)
        {
            throw std::runtime_error(
                "Pinocchio model has an incompatible configuration layout");
        }
        q_(6) = 1.0;
    }

    [[nodiscard]] std::optional<ZPosEstimate> Update(
        const Eigen::Ref<const Eigen::VectorXd>& joint_positions,
        const Eigen::Matrix3d& world_R_body,
        const Contacts& contacts,
        const GaitPhases& gait_phases,
        const ContactStates& contact_states)
    {
        if (joint_positions.size() != kNumJoints)
        {
            throw std::invalid_argument(
                "ZPosEstimator expects exactly 12 joint positions");
        }

        if (!joint_positions.allFinite() || !world_R_body.allFinite())
        {
            return EstimateWithoutUpdate();
        }

        q_.setZero();
        v_.setZero();
        q_(6) = 1.0;
        q_.segment<3>(7) = joint_positions.segment<3>(3);   // L1
        q_.segment<3>(10) = joint_positions.segment<3>(9);  // L2
        q_.segment<3>(13) = joint_positions.segment<3>(0);  // R1
        q_.segment<3>(16) = joint_positions.segment<3>(6);  // R2

        robot_.ComputeForwardKinematics(q_, v_);
        const std::vector<Eigen::Vector3d> pinocchio_foot_positions =
            robot_.GetToePositionsInBaseFrame();
        if (pinocchio_foot_positions.size() != kLegCount)
        {
            throw std::runtime_error(
                "Pinocchio model must provide exactly four toe frames");
        }

        std::array<double, kLegCount> relative_foot_z{};
        for (std::size_t leg = 0; leg < kLegCount; ++leg)
        {
            relative_foot_z[leg] =
                (world_R_body *
                 pinocchio_foot_positions[kServoToPinocchio[leg]])
                    .z();
            if (!std::isfinite(relative_foot_z[leg]))
            {
                return EstimateWithoutUpdate();
            }
        }

        if (!initialized_)
        {
            return Initialize(
                relative_foot_z,
                contacts,
                gait_phases,
                contact_states);
        }

        double position_sum = 0.0;
        std::size_t contacts_used = 0;
        for (std::size_t leg = 0; leg < kLegCount; ++leg)
        {
            if (contacts[leg] && anchor_valid_[leg])
            {
                position_sum += anchor_z_[leg] - relative_foot_z[leg];
                ++contacts_used;
            }
        }

        const bool has_reliable_support =
            contacts_used >= kMinimumSupportLegs &&
            AreContactsTrusted(contacts, gait_phases, contact_states);
        if (has_reliable_support)
        {
            position_z_ = position_sum / static_cast<double>(contacts_used);
        }

        // Transfer support only after estimating from anchors that existed at
        // the beginning of this update.
        for (std::size_t leg = 0; leg < kLegCount; ++leg)
        {
            if (!contacts[leg])
            {
                anchor_valid_[leg] = false;
                continue;
            }

            if (!anchor_valid_[leg])
            {
                anchor_z_[leg] = position_z_ + relative_foot_z[leg];
                anchor_valid_[leg] = true;
            }
        }

        return ZPosEstimate{
            position_z_,
            contacts_used,
            has_reliable_support};
    }

    void Reset() noexcept
    {
        anchor_z_.fill(0.0);
        anchor_valid_.fill(false);
        position_z_ = 0.0;
        initialized_ = false;
    }

    [[nodiscard]] bool initialized() const noexcept
    {
        return initialized_;
    }

private:
    [[nodiscard]] bool AreContactsTrusted(
        const Contacts& contacts,
        const GaitPhases& gait_phases,
        const ContactStates& contact_states) const noexcept
    {
        for (std::size_t leg = 0; leg < kLegCount; ++leg)
        {
            if (contacts[leg] &&
                reliable_contact_.get_trust_coefficient(
                    gait_phases[leg], contact_states[leg]) <= kMinimumTrust)
            {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] std::optional<ZPosEstimate> Initialize(
        const std::array<double, kLegCount>& relative_foot_z,
        const Contacts& contacts,
        const GaitPhases& gait_phases,
        const ContactStates& contact_states)
    {
        double position_sum = 0.0;
        std::size_t contacts_used = 0;
        for (std::size_t leg = 0; leg < kLegCount; ++leg)
        {
            if (contacts[leg])
            {
                position_sum += initial_contact_z_ - relative_foot_z[leg];
                ++contacts_used;
            }
        }

        if (contacts_used < kMinimumSupportLegs ||
            !AreContactsTrusted(contacts, gait_phases, contact_states))
        {
            return std::nullopt;
        }

        position_z_ = position_sum / static_cast<double>(contacts_used);
        for (std::size_t leg = 0; leg < kLegCount; ++leg)
        {
            if (contacts[leg])
            {
                anchor_z_[leg] = initial_contact_z_;
                anchor_valid_[leg] = true;
            }
        }
        initialized_ = true;
        return ZPosEstimate{position_z_, contacts_used, true};
    }

    [[nodiscard]] std::optional<ZPosEstimate> EstimateWithoutUpdate() const
    {
        if (!initialized_)
        {
            return std::nullopt;
        }
        return ZPosEstimate{position_z_, 0, false};
    }

    Robot robot_;
    Eigen::VectorXd q_;
    Eigen::VectorXd v_;
    ReliableContact reliable_contact_;
    std::array<double, kLegCount> anchor_z_{};
    std::array<bool, kLegCount> anchor_valid_{};
    double initial_contact_z_ = 0.0;
    double position_z_ = 0.0;
    bool initialized_ = false;
};

ZPosEstimator::ZPosEstimator(double initial_contact_z)
    : impl_(std::make_unique<Impl>(initial_contact_z))
{
}

ZPosEstimator::~ZPosEstimator() = default;

std::optional<ZPosEstimate> ZPosEstimator::Update(
    const Eigen::Ref<const Eigen::VectorXd>& joint_positions,
    const Eigen::Matrix3d& world_R_body,
    const Contacts& contacts,
    const GaitPhases& gait_phases,
    const ContactStates& contact_states)
{
    return impl_->Update(
        joint_positions,
        world_R_body,
        contacts,
        gait_phases,
        contact_states);
}

void ZPosEstimator::Reset() noexcept
{
    impl_->Reset();
}

bool ZPosEstimator::initialized() const noexcept
{
    return impl_->initialized();
}

}  // namespace state_estimator_hmb
