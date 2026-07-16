#include "z_pos_estimator.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "Robot.hpp"
#include "structs.hpp"

namespace
{

using state_estimator_hmb::ZPosEstimate;
using state_estimator_hmb::ZPosEstimator;

constexpr double kTolerance = 1e-10;
constexpr ZPosEstimator::GaitPhases kTrustedPhases{0.5, 0.5, 0.5, 0.5};
constexpr ZPosEstimator::ContactStates kStanceStates{
    STANCE, STANCE, STANCE, STANCE};

void Check(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void CheckNear(double actual, double expected, const std::string& message)
{
    if (!std::isfinite(actual) || std::fabs(actual - expected) > kTolerance)
    {
        throw std::runtime_error(
            message + ": actual=" + std::to_string(actual) +
            ", expected=" + std::to_string(expected));
    }
}

std::optional<ZPosEstimate> UpdateEstimator(
    ZPosEstimator& estimator,
    const Eigen::Ref<const Eigen::VectorXd>& joint_positions,
    const Eigen::Matrix3d& world_R_body,
    const ZPosEstimator::Contacts& contacts,
    const ZPosEstimator::GaitPhases& gait_phases = kTrustedPhases,
    const ZPosEstimator::ContactStates& contact_states = kStanceStates)
{
    return estimator.Update(
        joint_positions,
        world_R_body,
        contacts,
        gait_phases,
        contact_states);
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

Eigen::Matrix3d RotationFromRpy(double roll, double pitch, double yaw)
{
    return (
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
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
        const Eigen::VectorXd& joints,
        const Eigen::Matrix3d& world_R_body)
    {
        q_.setZero();
        v_.setZero();
        q_(6) = 1.0;
        q_.segment<3>(7) = joints.segment<3>(3);   // L1
        q_.segment<3>(10) = joints.segment<3>(9);  // L2
        q_.segment<3>(13) = joints.segment<3>(0);  // R1
        q_.segment<3>(16) = joints.segment<3>(6);  // R2

        robot_.ComputeForwardKinematics(q_, v_);
        const std::vector<Eigen::Vector3d> positions =
            robot_.GetToePositionsInBaseFrame();
        Check(positions.size() == 4, "Reference FK returned wrong toe count");
        return {
            (world_R_body * positions[PIN_R1]).z(),
            (world_R_body * positions[PIN_L1]).z(),
            (world_R_body * positions[PIN_R2]).z(),
            (world_R_body * positions[PIN_L2]).z()};
    }

private:
    Robot robot_;
    Eigen::VectorXd q_;
    Eigen::VectorXd v_;
};

double MeanPosition(
    const std::array<double, 4>& relative_z,
    const ZPosEstimator::Contacts& contacts)
{
    double sum = 0.0;
    std::size_t count = 0;
    for (std::size_t leg = 0; leg < contacts.size(); ++leg)
    {
        if (contacts[leg])
        {
            sum -= relative_z[leg];
            ++count;
        }
    }
    return sum / static_cast<double>(count);
}

void CheckReliable(
    const std::optional<ZPosEstimate>& estimate,
    std::size_t expected_contacts,
    const std::string& message)
{
    Check(estimate.has_value(), message + ": missing estimate");
    Check(estimate->updated_from_contacts, message + ": unreliable estimate");
    Check(estimate->contacts_used == expected_contacts,
          message + ": wrong support count");
}

void TestInitializationAndLegOrder(ReferenceKinematics& reference)
{
    const Eigen::VectorXd joints = NominalJointPositions();
    const Eigen::Matrix3d rotation = RotationFromRpy(0.13, -0.09, 0.4);
    const auto relative_z = reference.RelativeFootZ(joints, rotation);
    ZPosEstimator estimator;

    Check(!UpdateEstimator(estimator, joints, rotation, {}).has_value(),
          "Estimator initialized without contacts");
    for (std::size_t leg = 0; leg < ZPosEstimator::kLegCount; ++leg)
    {
        estimator.Reset();
        ZPosEstimator::Contacts one_contact{};
        one_contact[leg] = true;
        Check(!UpdateEstimator(estimator, joints, rotation, one_contact).has_value(),
              "Estimator initialized from one contact");
    }

    constexpr std::array<std::array<std::size_t, 2>, 6> pairs{{
        {{0, 1}}, {{0, 2}}, {{0, 3}},
        {{1, 2}}, {{1, 3}}, {{2, 3}}}};
    for (const auto& pair : pairs)
    {
        estimator.Reset();
        ZPosEstimator::Contacts contacts{};
        contacts[pair[0]] = true;
        contacts[pair[1]] = true;
        const auto estimate = UpdateEstimator(estimator, joints, rotation, contacts);
        CheckReliable(estimate, 2, "Two-contact initialization failed");
        CheckNear(
            estimate->position_z,
            MeanPosition(relative_z, contacts),
            "Wrong servo-to-Pinocchio leg order");
    }

    estimator.Reset();
    const ZPosEstimator::Contacts all_contacts{true, true, true, true};
    const auto estimate = UpdateEstimator(estimator, joints, rotation, all_contacts);
    CheckReliable(estimate, 4, "Four-contact initialization failed");
    CheckNear(
        estimate->position_z,
        MeanPosition(relative_z, all_contacts),
        "Four-contact mean is wrong");
}

void TestSupportHandoff(ReferenceKinematics& reference)
{
    const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    const Eigen::VectorXd initial_joints = NominalJointPositions();
    Eigen::VectorXd overlap_joints = initial_joints;
    overlap_joints(1) += 0.16;
    overlap_joints(2) -= 0.09;
    overlap_joints(10) -= 0.13;
    Eigen::VectorXd final_joints = overlap_joints;
    final_joints(4) += 0.18;
    final_joints(7) -= 0.15;

    ZPosEstimator estimator;
    const ZPosEstimator::Contacts old_pair{true, false, false, true};
    auto estimate = UpdateEstimator(estimator, initial_joints, rotation, old_pair);
    CheckReliable(estimate, 2, "Initial diagonal support failed");

    const auto relative_overlap =
        reference.RelativeFootZ(overlap_joints, rotation);
    const ZPosEstimator::Contacts overlap{true, true, true, true};
    estimate = UpdateEstimator(estimator, overlap_joints, rotation, overlap);
    CheckReliable(estimate, 2, "New contacts were used before anchoring");
    const double overlap_z =
        (-relative_overlap[0] - relative_overlap[3]) / 2.0;
    CheckNear(estimate->position_z, overlap_z, "Old support estimate is wrong");

    const auto relative_final = reference.RelativeFootZ(final_joints, rotation);
    const double l1_anchor = overlap_z + relative_overlap[1];
    const double r2_anchor = overlap_z + relative_overlap[2];
    const ZPosEstimator::Contacts new_pair{false, true, true, false};
    estimate = UpdateEstimator(estimator, final_joints, rotation, new_pair);
    CheckReliable(estimate, 2, "Transferred support was not used");
    CheckNear(
        estimate->position_z,
        ((l1_anchor - relative_final[1]) +
         (r2_anchor - relative_final[2])) / 2.0,
        "Transferred support estimate is wrong");
}

void TestInsufficientSupportAndInvalidInputs()
{
    const Eigen::VectorXd joints = NominalJointPositions();
    const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    const ZPosEstimator::Contacts pair{true, false, true, false};
    ZPosEstimator estimator;

    auto estimate = UpdateEstimator(estimator, joints, rotation, pair);
    CheckReliable(estimate, 2, "Test setup failed");
    const double held_z = estimate->position_z;

    const ZPosEstimator::Contacts one_contact{true, false, false, false};
    estimate = UpdateEstimator(estimator, joints, rotation, one_contact);
    Check(estimate.has_value() && !estimate->updated_from_contacts &&
              estimate->contacts_used == 1,
          "One contact was treated as reliable");
    CheckNear(estimate->position_z, held_z, "One contact changed Z");

    Eigen::VectorXd invalid_joints = joints;
    invalid_joints(0) = std::numeric_limits<double>::quiet_NaN();
    estimate = UpdateEstimator(estimator, invalid_joints, rotation, pair);
    Check(estimate.has_value() && !estimate->updated_from_contacts,
          "NaN was treated as a valid update");
    CheckNear(estimate->position_z, held_z, "NaN changed Z");

    bool wrong_size_rejected = false;
    try
    {
        static_cast<void>(estimator.Update(
            Eigen::VectorXd::Zero(11),
            rotation,
            pair,
            kTrustedPhases,
            kStanceStates));
    }
    catch (const std::invalid_argument&)
    {
        wrong_size_rejected = true;
    }
    Check(wrong_size_rejected, "Wrong joint-vector size was accepted");
}

void TestContactTrust()
{
    const Eigen::VectorXd joints = NominalJointPositions();
    const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    const ZPosEstimator::Contacts pair{true, false, true, false};
    ZPosEstimator estimator;

    auto phases = kTrustedPhases;
    phases[0] = 0.198;
    Check(!UpdateEstimator(estimator, joints, rotation, pair, phases).has_value(),
          "Estimator initialized at trust <= 0.99");

    auto estimate = UpdateEstimator(estimator, joints, rotation, pair);
    CheckReliable(estimate, 2, "Trusted-contact initialization failed");
    const double held_z = estimate->position_z;

    Eigen::VectorXd moved_joints = joints;
    moved_joints(1) += 0.12;
    moved_joints(7) -= 0.10;

    phases = kTrustedPhases;
    phases[0] = 0.19;
    estimate = UpdateEstimator(estimator, moved_joints, rotation, pair, phases);
    Check(estimate.has_value() && !estimate->updated_from_contacts,
          "Transition-phase contact was treated as reliable");
    CheckNear(estimate->position_z, held_z, "Low trust changed Z");

    phases = kTrustedPhases;
    phases[2] = std::numeric_limits<double>::quiet_NaN();
    estimate = UpdateEstimator(estimator, moved_joints, rotation, pair, phases);
    Check(estimate.has_value() && !estimate->updated_from_contacts,
          "Invalid gait phase was treated as reliable");
    CheckNear(estimate->position_z, held_z, "Invalid gait phase changed Z");

    auto states = kStanceStates;
    states[0] = SWING;
    estimate = UpdateEstimator(
        estimator, moved_joints, rotation, pair, kTrustedPhases, states);
    Check(estimate.has_value() && !estimate->updated_from_contacts,
          "Non-stance contact was treated as reliable");
    CheckNear(estimate->position_z, held_z, "Non-stance contact changed Z");

    const ZPosEstimator::Contacts all_contacts{true, true, true, true};
    phases = kTrustedPhases;
    phases[1] = 0.1;
    estimate = UpdateEstimator(
        estimator, moved_joints, rotation, all_contacts, phases);
    Check(estimate.has_value() && !estimate->updated_from_contacts,
          "Four-contact support ignored one unreliable leg");
    CheckNear(estimate->position_z, held_z, "Unreliable four-leg support changed Z");

    estimate = UpdateEstimator(estimator, moved_joints, rotation, all_contacts);
    CheckReliable(estimate, 4, "Update did not resume with trusted contacts");
}

}  // namespace

int main()
{
    try
    {
        ReferenceKinematics reference;
        TestInitializationAndLegOrder(reference);
        TestSupportHandoff(reference);
        TestInsufficientSupportAndInvalidInputs();
        TestContactTrust();
        std::cout << "z_pos_estimator_test passed\n";
        return EXIT_SUCCESS;
    }
    catch (const std::exception& error)
    {
        std::cerr << "z_pos_estimator_test failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
