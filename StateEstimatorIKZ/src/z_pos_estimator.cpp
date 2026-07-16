#include "z_pos_estimator.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "Robot.hpp"

namespace
{
constexpr std::size_t kNumLegs = 4;
constexpr Eigen::Index kNumJoints = 12;

// Index by servo leg order (R1, L1, R2, L2); values refer to Pinocchio's
// toe order (L1, L2, R1, R2).
constexpr std::array<std::size_t, kNumLegs> kServoToPinocchio{
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

        // The MORS model uses a free-flyer base followed by twelve actuated
        // joints: 3 position + 4 quaternion + 12 joint coordinates.
        if (q_.size() < 19 || v_.size() < 18)
        {
            throw std::runtime_error(
                "Pinocchio model has an incompatible configuration layout");
        }
        q_(6) = 1.0;  // Identity quaternion, stored as (x, y, z, w).
    }

    [[nodiscard]] std::optional<ZPosEstimate> Update(
        const Eigen::Ref<const Eigen::VectorXd>& joint_positions,
        const Eigen::Matrix3d& world_R_body,
        const Contacts& contacts)
    {
        if (joint_positions.size() != kNumJoints)
        {
            throw std::invalid_argument(
                "ZPosEstimator expects exactly 12 joint positions");
        }

        if (!joint_positions.allFinite() || !world_R_body.allFinite())
        {
            InvalidateLostContacts(contacts);
            return EstimateWithoutUpdate();
        }

        q_.setZero();
        v_.setZero();
        q_(6) = 1.0;

        // Servo order:      R1, L1, R2, L2.
        // Pinocchio order:  L1, L2, R1, R2.
        q_.segment<3>(7) = joint_positions.segment<3>(3);   // L1
        q_.segment<3>(10) = joint_positions.segment<3>(9);  // L2
        q_.segment<3>(13) = joint_positions.segment<3>(0);  // R1
        q_.segment<3>(16) = joint_positions.segment<3>(6);  // R2

        robot_.ComputeForwardKinematics(q_, v_);
        const std::vector<Eigen::Vector3d> pinocchio_foot_positions =
            robot_.GetToePositionsInBaseFrame();
        if (pinocchio_foot_positions.size() != kNumLegs)
        {
            throw std::runtime_error(
                "Pinocchio model must provide exactly four toe frames");
        }

        std::array<double, kNumLegs> relative_foot_z{};
        for (std::size_t leg = 0; leg < kNumLegs; ++leg)
        {
            const Eigen::Vector3d foot_offset_world =
                world_R_body *
                pinocchio_foot_positions[kServoToPinocchio[leg]];
            relative_foot_z[leg] = foot_offset_world.z();

            // Validate every FK result before updating Z or creating anchors.
            // A reported liftoff is still honored on an invalid sample so that
            // a later touchdown cannot reuse a stale anchor.
            if (!std::isfinite(relative_foot_z[leg]))
            {
                InvalidateLostContacts(contacts);
                return EstimateWithoutUpdate();
            }
        }

        if (!initialized_)
        {
            return Initialize(relative_foot_z, contacts);
        }

        double position_sum = 0.0;
        std::size_t contacts_used = 0;

        // Estimate from anchors that existed at the beginning of this update.
        // A newly touching foot must not participate until its anchor has been
        // created using the estimate supplied by the old support set.
        for (std::size_t leg = 0; leg < kNumLegs; ++leg)
        {
            if (contacts[leg] && anchor_valid_[leg])
            {
                position_sum += anchor_z_[leg] - relative_foot_z[leg];
                ++contacts_used;
            }
        }

        if (contacts_used > 0)
        {
            position_z_ =
                position_sum / static_cast<double>(contacts_used);
        }

        // Only after estimating from the old anchors do we transfer support to
        // newly touching feet and invalidate feet that have lifted off.
        for (std::size_t leg = 0; leg < kNumLegs; ++leg)
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
            position_z_, contacts_used, contacts_used > 0};
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
    void InvalidateLostContacts(const Contacts& contacts) noexcept
    {
        for (std::size_t leg = 0; leg < kNumLegs; ++leg)
        {
            if (!contacts[leg])
            {
                anchor_valid_[leg] = false;
            }
        }
    }

    [[nodiscard]] std::optional<ZPosEstimate> Initialize(
        const std::array<double, kNumLegs>& relative_foot_z,
        const Contacts& contacts)
    {
        double position_sum = 0.0;
        std::size_t contacts_used = 0;

        for (std::size_t leg = 0; leg < kNumLegs; ++leg)
        {
            if (contacts[leg])
            {
                position_sum += initial_contact_z_ - relative_foot_z[leg];
                ++contacts_used;
            }
        }

        if (contacts_used == 0)
        {
            return std::nullopt;
        }

        position_z_ = position_sum / static_cast<double>(contacts_used);
        for (std::size_t leg = 0; leg < kNumLegs; ++leg)
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
    std::array<double, kNumLegs> anchor_z_{};
    std::array<bool, kNumLegs> anchor_valid_{};
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
    const Contacts& contacts)
{
    return impl_->Update(joint_positions, world_R_body, contacts);
}

void ZPosEstimator::Reset() noexcept
{
    impl_->Reset();
}

bool ZPosEstimator::initialized() const noexcept
{
    return impl_->initialized();
}
