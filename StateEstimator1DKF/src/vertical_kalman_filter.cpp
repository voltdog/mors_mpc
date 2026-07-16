#include "vertical_kalman_filter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace state_estimator_1d
{

VerticalKalmanFilter::VerticalKalmanFilter(VerticalProcessNoise process_noise)
    : process_noise_(process_noise)
{
    if (!(process_noise_.acceleration_std > 0.0) ||
        !(process_noise_.acceleration_bias_random_walk_std > 0.0) ||
        !(process_noise_.stance_foot_height_random_walk_std >= 0.0) ||
        !(process_noise_.max_prediction_step_sec > 0.0))
    {
        throw std::invalid_argument("VerticalKalmanFilter process noise values are invalid.");
    }
}

void VerticalKalmanFilter::Initialize(
    double position,
    double velocity,
    double acceleration_bias,
    double position_std,
    double velocity_std,
    double acceleration_bias_std)
{
    if (!std::isfinite(position) || !std::isfinite(velocity) ||
        !std::isfinite(acceleration_bias) || !(position_std > 0.0) ||
        !(velocity_std > 0.0) || !(acceleration_bias_std > 0.0))
    {
        throw std::invalid_argument("VerticalKalmanFilter initial estimate is invalid.");
    }

    state_.setZero();
    state_(kPositionIndex) = position;
    state_(kVelocityIndex) = velocity;
    state_(kAccelerationBiasIndex) = acceleration_bias;

    covariance_.setZero();
    covariance_(kPositionIndex, kPositionIndex) = position_std * position_std;
    covariance_(kVelocityIndex, kVelocityIndex) = velocity_std * velocity_std;
    covariance_(kAccelerationBiasIndex, kAccelerationBiasIndex) =
        acceleration_bias_std * acceleration_bias_std;
    for (int leg = 0; leg < kLegCount; ++leg)
    {
        covariance_(FootHeightIndex(leg), FootHeightIndex(leg)) = 1e6;
    }

    foot_anchor_valid_.fill(false);
    initialized_ = true;
}

bool VerticalKalmanFilter::initialized() const
{
    return initialized_;
}

void VerticalKalmanFilter::Predict(double measured_world_acceleration_z, double dt_sec)
{
    if (!initialized_ || !std::isfinite(measured_world_acceleration_z) ||
        !std::isfinite(dt_sec) || !(dt_sec > 0.0))
    {
        return;
    }

    double remaining = dt_sec;
    while (remaining > 0.0)
    {
        const double step = std::min(remaining, process_noise_.max_prediction_step_sec);
        PredictSingleStep(measured_world_acceleration_z, step);
        remaining -= step;
    }
}

bool VerticalKalmanFilter::UpdatePosition(
    double measured_position,
    double measurement_std,
    double innovation_gate_sigma)
{
    Eigen::Matrix<double, 1, kStateSize> observation =
        Eigen::Matrix<double, 1, kStateSize>::Zero();
    observation(kPositionIndex) = 1.0;
    return ScalarUpdate(
        observation,
        measured_position,
        measurement_std,
        innovation_gate_sigma);
}

bool VerticalKalmanFilter::UpdateVelocity(
    double measured_velocity,
    double measurement_std,
    double innovation_gate_sigma)
{
    Eigen::Matrix<double, 1, kStateSize> observation =
        Eigen::Matrix<double, 1, kStateSize>::Zero();
    observation(kVelocityIndex) = 1.0;
    return ScalarUpdate(
        observation,
        measured_velocity,
        measurement_std,
        innovation_gate_sigma);
}

void VerticalKalmanFilter::InitializeFootAnchor(
    int leg,
    double foot_offset_world_z,
    double initialization_std)
{
    if (!initialized_ || !IsLegIndexValid(leg) ||
        !std::isfinite(foot_offset_world_z) || !(initialization_std > 0.0))
    {
        return;
    }

    const int foot_index = FootHeightIndex(leg);
    state_(foot_index) = state_(kPositionIndex) + foot_offset_world_z;

    const Eigen::Matrix<double, 1, kStateSize> position_covariance_row =
        covariance_.row(kPositionIndex);
    const Eigen::Matrix<double, kStateSize, 1> position_covariance_column =
        covariance_.col(kPositionIndex);
    covariance_.row(foot_index) = position_covariance_row;
    covariance_.col(foot_index) = position_covariance_column;
    covariance_(foot_index, foot_index) =
        covariance_(kPositionIndex, kPositionIndex) + initialization_std * initialization_std;

    foot_anchor_valid_[static_cast<size_t>(leg)] = true;
    SymmetrizeCovariance();
}

void VerticalKalmanFilter::InvalidateFootAnchor(int leg)
{
    if (!IsLegIndexValid(leg))
    {
        return;
    }
    foot_anchor_valid_[static_cast<size_t>(leg)] = false;
}

bool VerticalKalmanFilter::UpdateFootConstraint(
    int leg,
    double foot_offset_world_z,
    double measurement_std,
    double innovation_gate_sigma)
{
    if (!foot_anchor_valid(leg) || !std::isfinite(foot_offset_world_z))
    {
        return false;
    }

    Eigen::Matrix<double, 1, kStateSize> observation =
        Eigen::Matrix<double, 1, kStateSize>::Zero();
    observation(kPositionIndex) = 1.0;
    observation(FootHeightIndex(leg)) = -1.0;

    return ScalarUpdate(
        observation,
        -foot_offset_world_z,
        measurement_std,
        innovation_gate_sigma);
}

bool VerticalKalmanFilter::foot_anchor_valid(int leg) const
{
    return IsLegIndexValid(leg) && foot_anchor_valid_[static_cast<size_t>(leg)];
}

double VerticalKalmanFilter::position() const
{
    return state_(kPositionIndex);
}

double VerticalKalmanFilter::velocity() const
{
    return state_(kVelocityIndex);
}

double VerticalKalmanFilter::acceleration_bias() const
{
    return state_(kAccelerationBiasIndex);
}

double VerticalKalmanFilter::foot_height(int leg) const
{
    return IsLegIndexValid(leg) ? state_(FootHeightIndex(leg)) : 0.0;
}

const Eigen::Matrix<double, VerticalKalmanFilter::kStateSize, 1>&
VerticalKalmanFilter::state() const
{
    return state_;
}

const Eigen::Matrix<double, VerticalKalmanFilter::kStateSize, VerticalKalmanFilter::kStateSize>&
VerticalKalmanFilter::covariance() const
{
    return covariance_;
}

bool VerticalKalmanFilter::IsLegIndexValid(int leg)
{
    return leg >= 0 && leg < kLegCount;
}

int VerticalKalmanFilter::FootHeightIndex(int leg)
{
    return kFirstFootHeightIndex + leg;
}

bool VerticalKalmanFilter::ScalarUpdate(
    const Eigen::Matrix<double, 1, kStateSize>& observation,
    double measurement,
    double measurement_std,
    double innovation_gate_sigma)
{
    if (!initialized_ || !std::isfinite(measurement) || !(measurement_std > 0.0))
    {
        return false;
    }

    const double measurement_variance = measurement_std * measurement_std;
    const double innovation = measurement - (observation * state_)(0);
    const double innovation_variance =
        (observation * covariance_ * observation.transpose())(0, 0) + measurement_variance;
    if (!std::isfinite(innovation) || !(innovation_variance > 0.0))
    {
        return false;
    }

    if (innovation_gate_sigma > 0.0 &&
        std::fabs(innovation) > innovation_gate_sigma * std::sqrt(innovation_variance))
    {
        return false;
    }

    const Eigen::Matrix<double, kStateSize, 1> gain =
        covariance_ * observation.transpose() / innovation_variance;
    state_ += gain * innovation;

    const Eigen::Matrix<double, kStateSize, kStateSize> identity =
        Eigen::Matrix<double, kStateSize, kStateSize>::Identity();
    const Eigen::Matrix<double, kStateSize, kStateSize> correction =
        identity - gain * observation;
    covariance_ = correction * covariance_ * correction.transpose() +
                  gain * measurement_variance * gain.transpose();
    SymmetrizeCovariance();
    return state_.allFinite() && covariance_.allFinite();
}

void VerticalKalmanFilter::PredictSingleStep(
    double measured_world_acceleration_z,
    double dt_sec)
{
    const double dt_squared = dt_sec * dt_sec;
    const double unbiased_acceleration =
        measured_world_acceleration_z - state_(kAccelerationBiasIndex);

    state_(kPositionIndex) +=
        state_(kVelocityIndex) * dt_sec + 0.5 * unbiased_acceleration * dt_squared;
    state_(kVelocityIndex) += unbiased_acceleration * dt_sec;

    Eigen::Matrix<double, kStateSize, kStateSize> transition =
        Eigen::Matrix<double, kStateSize, kStateSize>::Identity();
    transition(kPositionIndex, kVelocityIndex) = dt_sec;
    transition(kPositionIndex, kAccelerationBiasIndex) = -0.5 * dt_squared;
    transition(kVelocityIndex, kAccelerationBiasIndex) = -dt_sec;

    Eigen::Matrix<double, kStateSize, kStateSize> process_covariance =
        Eigen::Matrix<double, kStateSize, kStateSize>::Zero();
    Eigen::Matrix<double, kStateSize, 1> acceleration_noise_gain =
        Eigen::Matrix<double, kStateSize, 1>::Zero();
    acceleration_noise_gain(kPositionIndex) = 0.5 * dt_squared;
    acceleration_noise_gain(kVelocityIndex) = dt_sec;
    process_covariance +=
        process_noise_.acceleration_std * process_noise_.acceleration_std *
        acceleration_noise_gain * acceleration_noise_gain.transpose();
    process_covariance(kAccelerationBiasIndex, kAccelerationBiasIndex) +=
        process_noise_.acceleration_bias_random_walk_std *
        process_noise_.acceleration_bias_random_walk_std * dt_sec;

    const double foot_height_variance =
        process_noise_.stance_foot_height_random_walk_std *
        process_noise_.stance_foot_height_random_walk_std * dt_sec;
    for (int leg = 0; leg < kLegCount; ++leg)
    {
        if (foot_anchor_valid_[static_cast<size_t>(leg)])
        {
            process_covariance(FootHeightIndex(leg), FootHeightIndex(leg)) +=
                foot_height_variance;
        }
    }

    covariance_ = transition * covariance_ * transition.transpose() + process_covariance;
    SymmetrizeCovariance();
}

void VerticalKalmanFilter::SymmetrizeCovariance()
{
    covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}

}  // namespace state_estimator_1d
