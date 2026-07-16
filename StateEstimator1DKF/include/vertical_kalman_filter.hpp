#ifndef STATE_ESTIMATOR_1D_KF_VERTICAL_KALMAN_FILTER_HPP_
#define STATE_ESTIMATOR_1D_KF_VERTICAL_KALMAN_FILTER_HPP_

#include <array>

#include <Eigen/Dense>

namespace state_estimator_1d
{

struct VerticalProcessNoise
{
    double acceleration_std{0.8};
    double acceleration_bias_random_walk_std{0.02};
    double stance_foot_height_random_walk_std{1e-4};
    double max_prediction_step_sec{0.01};
};

class VerticalKalmanFilter
{
public:
    static constexpr int kLegCount = 4;
    static constexpr int kStateSize = 3 + kLegCount;

    explicit VerticalKalmanFilter(VerticalProcessNoise process_noise = {});

    void Initialize(
        double position,
        double velocity,
        double acceleration_bias,
        double position_std,
        double velocity_std,
        double acceleration_bias_std);

    [[nodiscard]] bool initialized() const;

    void Predict(double measured_world_acceleration_z, double dt_sec);

    [[nodiscard]] bool UpdatePosition(
        double measured_position,
        double measurement_std,
        double innovation_gate_sigma);

    [[nodiscard]] bool UpdateVelocity(
        double measured_velocity,
        double measurement_std,
        double innovation_gate_sigma);

    void InitializeFootAnchor(
        int leg,
        double foot_offset_world_z,
        double initialization_std);

    void InvalidateFootAnchor(int leg);

    [[nodiscard]] bool UpdateFootConstraint(
        int leg,
        double foot_offset_world_z,
        double measurement_std,
        double innovation_gate_sigma);

    [[nodiscard]] bool foot_anchor_valid(int leg) const;
    [[nodiscard]] double position() const;
    [[nodiscard]] double velocity() const;
    [[nodiscard]] double acceleration_bias() const;
    [[nodiscard]] double foot_height(int leg) const;
    [[nodiscard]] const Eigen::Matrix<double, kStateSize, 1>& state() const;
    [[nodiscard]] const Eigen::Matrix<double, kStateSize, kStateSize>& covariance() const;

private:
    static constexpr int kPositionIndex = 0;
    static constexpr int kVelocityIndex = 1;
    static constexpr int kAccelerationBiasIndex = 2;
    static constexpr int kFirstFootHeightIndex = 3;

    [[nodiscard]] static bool IsLegIndexValid(int leg);
    [[nodiscard]] static int FootHeightIndex(int leg);

    [[nodiscard]] bool ScalarUpdate(
        const Eigen::Matrix<double, 1, kStateSize>& observation,
        double measurement,
        double measurement_std,
        double innovation_gate_sigma);

    void PredictSingleStep(double measured_world_acceleration_z, double dt_sec);
    void SymmetrizeCovariance();

    VerticalProcessNoise process_noise_;
    Eigen::Matrix<double, kStateSize, 1> state_{
        Eigen::Matrix<double, kStateSize, 1>::Zero()};
    Eigen::Matrix<double, kStateSize, kStateSize> covariance_{
        Eigen::Matrix<double, kStateSize, kStateSize>::Zero()};
    std::array<bool, kLegCount> foot_anchor_valid_{false, false, false, false};
    bool initialized_{false};
};

}  // namespace state_estimator_1d

#endif  // STATE_ESTIMATOR_1D_KF_VERTICAL_KALMAN_FILTER_HPP_
