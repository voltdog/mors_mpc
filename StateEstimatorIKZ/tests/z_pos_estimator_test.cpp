#include "z_pos_estimator.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include "Robot.hpp"

namespace
{

constexpr double kTolerance = 1e-10;

void Check(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void CheckNear(double actual, double expected, const std::string& message)
{
    if (!std::isfinite(actual) ||
        std::fabs(actual - expected) > kTolerance)
    {
        throw std::runtime_error(
            message + ": actual=" + std::to_string(actual) +
            ", expected=" + std::to_string(expected));
    }
}

Eigen::Matrix3d RotationFromRpy(double roll, double pitch, double yaw)
{
    return (
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

Eigen::VectorXd NominalJointPositions()
{
    Eigen::VectorXd joints(12);
    joints <<
        0.08, -0.72, 1.18,   // R1
       -0.11,  0.67, -1.24,  // L1
        0.14, -0.83, 1.31,   // R2
       -0.06,  0.76, -1.12;  // L2
    return joints;
}

class ReferenceKinematics
{
public:
    ReferenceKinematics()
    {
        robot_.BuildPinocchioModel();
        q_ = Eigen::VectorXd::Zero(robot_.nq);
        v_ = Eigen::VectorXd::Zero(robot_.nv);
    }

    std::array<double, 4> RelativeFootZ(
        const Eigen::VectorXd& joint_positions,
        const Eigen::Matrix3d& world_R_body)
    {
        Check(joint_positions.size() == 12, "Reference FK expects 12 joints");

        q_.setZero();
        v_.setZero();
        q_(6) = 1.0;
        q_.segment<3>(7) = joint_positions.segment<3>(3);   // L1
        q_.segment<3>(10) = joint_positions.segment<3>(9);  // L2
        q_.segment<3>(13) = joint_positions.segment<3>(0);  // R1
        q_.segment<3>(16) = joint_positions.segment<3>(6);  // R2

        robot_.ComputeForwardKinematics(q_, v_);
        const std::vector<Eigen::Vector3d> pin_positions =
            robot_.GetToePositionsInBaseFrame();
        Check(pin_positions.size() == 4, "Reference FK returned wrong toe count");

        return {
            (world_R_body * pin_positions[PIN_R1]).z(),
            (world_R_body * pin_positions[PIN_L1]).z(),
            (world_R_body * pin_positions[PIN_R2]).z(),
            (world_R_body * pin_positions[PIN_L2]).z()};
    }

private:
    Robot robot_;
    Eigen::VectorXd q_;
    Eigen::VectorXd v_;
};

double MeanInitialPosition(
    const std::array<double, 4>& relative_z,
    const ZPosEstimator::Contacts& contacts,
    double contact_z = 0.0)
{
    double sum = 0.0;
    std::size_t count = 0;
    for (std::size_t leg = 0; leg < contacts.size(); ++leg)
    {
        if (contacts[leg])
        {
            sum += contact_z - relative_z[leg];
            ++count;
        }
    }
    Check(count > 0, "MeanInitialPosition needs a contact");
    return sum / static_cast<double>(count);
}

void TestInitializationAndLegOrder(ReferenceKinematics& reference)
{
    const Eigen::VectorXd joints = NominalJointPositions();
    const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    const std::array<double, 4> relative_z =
        reference.RelativeFootZ(joints, rotation);

    ZPosEstimator estimator;
    const ZPosEstimator::Contacts no_contacts{};
    Check(!estimator.Update(joints, rotation, no_contacts).has_value(),
          "Estimator initialized without a contact");
    Check(!estimator.initialized(), "initialized() is true before contact");

    for (std::size_t leg = 0; leg < 4; ++leg)
    {
        estimator.Reset();
        ZPosEstimator::Contacts contacts{};
        contacts[leg] = true;
        const auto estimate = estimator.Update(joints, rotation, contacts);
        Check(estimate.has_value(), "Single-contact initialization failed");
        CheckNear(
            estimate->position_z,
            -relative_z[leg],
            "Wrong servo-to-Pinocchio leg mapping");
        Check(estimate->contacts_used == 1,
              "Single-contact initialization used wrong contact count");
        Check(estimate->updated_from_contacts,
              "Single-contact initialization was not marked as constrained");
    }

    estimator.Reset();
    const ZPosEstimator::Contacts two_contacts{true, false, true, false};
    auto estimate = estimator.Update(joints, rotation, two_contacts);
    Check(estimate.has_value(), "Two-contact initialization failed");
    CheckNear(
        estimate->position_z,
        MeanInitialPosition(relative_z, two_contacts),
        "Two-contact estimate is not the arithmetic mean");
    Check(estimate->contacts_used == 2, "Wrong two-contact count");

    estimator.Reset();
    const ZPosEstimator::Contacts all_contacts{true, true, true, true};
    estimate = estimator.Update(joints, rotation, all_contacts);
    Check(estimate.has_value(), "Four-contact initialization failed");
    CheckNear(
        estimate->position_z,
        MeanInitialPosition(relative_z, all_contacts),
        "Four-contact estimate is not the arithmetic mean");
    Check(estimate->contacts_used == 4, "Wrong four-contact count");

    constexpr double kContactPlaneZ = 0.15;
    ZPosEstimator offset_estimator(kContactPlaneZ);
    const ZPosEstimator::Contacts one_contact{true, false, false, false};
    estimate = offset_estimator.Update(joints, rotation, one_contact);
    Check(estimate.has_value(), "Nonzero contact-plane initialization failed");
    CheckNear(
        estimate->position_z,
        kContactPlaneZ - relative_z[0],
        "initial_contact_z was not applied");
}

void TestOrientationProjection(ReferenceKinematics& reference)
{
    const Eigen::VectorXd joints = NominalJointPositions();
    const ZPosEstimator::Contacts contacts{true, true, false, true};
    const Eigen::Matrix3d rotation_a = RotationFromRpy(0.21, -0.17, 0.0);
    const Eigen::Matrix3d rotation_b = RotationFromRpy(0.21, -0.17, 1.3);

    const std::array<double, 4> relative_a =
        reference.RelativeFootZ(joints, rotation_a);
    const std::array<double, 4> relative_b =
        reference.RelativeFootZ(joints, rotation_b);

    ZPosEstimator estimator;
    auto estimate_a = estimator.Update(joints, rotation_a, contacts);
    Check(estimate_a.has_value(), "Tilted initialization failed");
    CheckNear(
        estimate_a->position_z,
        MeanInitialPosition(relative_a, contacts),
        "Estimator did not use the full body rotation");

    estimator.Reset();
    auto estimate_b = estimator.Update(joints, rotation_b, contacts);
    Check(estimate_b.has_value(), "Yawed initialization failed");
    CheckNear(
        estimate_b->position_z,
        MeanInitialPosition(relative_b, contacts),
        "Yawed estimate disagrees with reference FK");
    CheckNear(
        estimate_b->position_z,
        estimate_a->position_z,
        "Vertical estimate depends on yaw");
}

void TestAnchorUpdatesAndContactHandoff(ReferenceKinematics& reference)
{
    const Eigen::Matrix3d rotation = RotationFromRpy(-0.12, 0.09, 0.4);
    const Eigen::VectorXd joints_initial = NominalJointPositions();
    Eigen::VectorXd joints_overlap = joints_initial;
    joints_overlap(1) += 0.16;
    joints_overlap(2) -= 0.09;
    Eigen::VectorXd joints_after_handoff = joints_overlap;
    joints_after_handoff(3) -= 0.13;
    joints_after_handoff(4) += 0.18;

    const auto relative_initial =
        reference.RelativeFootZ(joints_initial, rotation);
    const auto relative_overlap =
        reference.RelativeFootZ(joints_overlap, rotation);
    const auto relative_after_handoff =
        reference.RelativeFootZ(joints_after_handoff, rotation);

    ZPosEstimator estimator;
    const ZPosEstimator::Contacts r1_only{true, false, false, false};
    auto estimate = estimator.Update(joints_initial, rotation, r1_only);
    Check(estimate.has_value(), "Initial stance anchor was not created");
    CheckNear(
        estimate->position_z,
        -relative_initial[0],
        "Wrong initial stance height");

    const ZPosEstimator::Contacts overlap{true, true, false, false};
    estimate = estimator.Update(joints_overlap, rotation, overlap);
    Check(estimate.has_value(), "Overlap update failed");
    const double overlap_z = -relative_overlap[0];
    CheckNear(
        estimate->position_z,
        overlap_z,
        "New touchdown changed the estimate in its creation sample");
    Check(estimate->contacts_used == 1,
          "New touchdown participated before its anchor existed");

    const double l1_anchor_z = overlap_z + relative_overlap[1];
    const ZPosEstimator::Contacts l1_only{false, true, false, false};
    estimate = estimator.Update(joints_after_handoff, rotation, l1_only);
    Check(estimate.has_value(), "Handoff update failed");
    CheckNear(
        estimate->position_z,
        l1_anchor_z - relative_after_handoff[1],
        "Contact anchor was not transferred across overlap");
    Check(estimate->contacts_used == 1, "Handoff used wrong anchor count");
}

void TestNoContactAndNoOverlap(ReferenceKinematics& reference)
{
    const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    const Eigen::VectorXd joints_initial = NominalJointPositions();
    Eigen::VectorXd joints_touchdown = joints_initial;
    joints_touchdown(7) += 0.11;
    Eigen::VectorXd joints_stance = joints_touchdown;
    joints_stance(7) -= 0.19;
    joints_stance(8) += 0.08;

    const auto relative_touchdown =
        reference.RelativeFootZ(joints_touchdown, rotation);
    const auto relative_stance =
        reference.RelativeFootZ(joints_stance, rotation);

    ZPosEstimator estimator;
    const ZPosEstimator::Contacts r1_only{true, false, false, false};
    auto estimate = estimator.Update(joints_initial, rotation, r1_only);
    Check(estimate.has_value(), "No-overlap setup failed");
    const double held_z = estimate->position_z;

    const ZPosEstimator::Contacts no_contacts{};
    estimate = estimator.Update(joints_initial, rotation, no_contacts);
    Check(estimate.has_value(), "Initialized estimator returned null in flight");
    CheckNear(estimate->position_z, held_z, "Flight did not hold the last Z");
    Check(!estimate->updated_from_contacts && estimate->contacts_used == 0,
          "Flight update was marked as contact-constrained");

    const ZPosEstimator::Contacts r2_only{false, false, true, false};
    estimate = estimator.Update(joints_touchdown, rotation, r2_only);
    Check(estimate.has_value(), "First contact after flight failed");
    CheckNear(
        estimate->position_z,
        held_z,
        "No-overlap touchdown changed the unobservable Z");
    Check(!estimate->updated_from_contacts && estimate->contacts_used == 0,
          "Fresh post-flight anchor was used in its creation sample");

    const double r2_anchor_z = held_z + relative_touchdown[2];
    estimate = estimator.Update(joints_stance, rotation, r2_only);
    Check(estimate.has_value(), "Post-flight stance update failed");
    CheckNear(
        estimate->position_z,
        r2_anchor_z - relative_stance[2],
        "Post-flight anchor did not constrain the next sample");
}

void TestResetAndInvalidInputs()
{
    const Eigen::VectorXd joints = NominalJointPositions();
    const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    const ZPosEstimator::Contacts contact{true, false, false, false};

    ZPosEstimator estimator;
    auto estimate = estimator.Update(joints, rotation, contact);
    Check(estimate.has_value(), "Invalid-input setup failed");
    const double valid_z = estimate->position_z;

    Eigen::VectorXd nan_joints = joints;
    nan_joints(0) = std::numeric_limits<double>::quiet_NaN();
    estimate = estimator.Update(nan_joints, rotation, contact);
    Check(estimate.has_value(), "NaN sample discarded an initialized estimate");
    CheckNear(estimate->position_z, valid_z, "NaN sample changed estimator state");
    Check(!estimate->updated_from_contacts && estimate->contacts_used == 0,
          "NaN sample was marked as a valid update");

    Eigen::Matrix3d infinite_rotation = rotation;
    infinite_rotation(0, 0) = std::numeric_limits<double>::infinity();
    estimate = estimator.Update(joints, infinite_rotation, contact);
    Check(estimate.has_value(), "Infinite rotation discarded the estimate");
    CheckNear(
        estimate->position_z,
        valid_z,
        "Infinite rotation changed estimator state");

    bool invalid_size_thrown = false;
    try
    {
        const Eigen::VectorXd wrong_size = Eigen::VectorXd::Zero(11);
        static_cast<void>(estimator.Update(wrong_size, rotation, contact));
    }
    catch (const std::invalid_argument&)
    {
        invalid_size_thrown = true;
    }
    Check(invalid_size_thrown, "Wrong joint-vector size did not throw");

    const ZPosEstimator::Contacts no_contacts{};
    estimate = estimator.Update(nan_joints, rotation, no_contacts);
    Check(estimate.has_value(), "Invalid liftoff sample discarded the estimate");
    CheckNear(
        estimate->position_z,
        valid_z,
        "Invalid liftoff sample changed the position estimate");

    Eigen::VectorXd joints_after_swing = joints;
    joints_after_swing(1) += 0.2;
    estimate = estimator.Update(joints_after_swing, rotation, contact);
    Check(estimate.has_value(), "Recontact after invalid swing failed");
    CheckNear(
        estimate->position_z,
        valid_z,
        "Recontact reused a stale anchor from the previous step");
    Check(!estimate->updated_from_contacts && estimate->contacts_used == 0,
          "Recontact used the stale anchor before creating a new one");

    estimator.Reset();
    Check(!estimator.initialized(), "Reset did not clear initialized state");
    Check(!estimator.Update(joints, rotation, no_contacts).has_value(),
          "Reset estimator returned an old estimate");

    ZPosEstimator uninitialized;
    Check(!uninitialized.Update(nan_joints, rotation, contact).has_value(),
          "NaN sample initialized a fresh estimator");
}

}  // namespace

int main()
{
    try
    {
        ReferenceKinematics reference;
        TestInitializationAndLegOrder(reference);
        TestOrientationProjection(reference);
        TestAnchorUpdatesAndContactHandoff(reference);
        TestNoContactAndNoOverlap(reference);
        TestResetAndInvalidInputs();
        return EXIT_SUCCESS;
    }
    catch (const std::exception& error)
    {
        std::cerr << "z_pos_estimator_test: " << error.what() << std::endl;
        return EXIT_FAILURE;
    }
}
