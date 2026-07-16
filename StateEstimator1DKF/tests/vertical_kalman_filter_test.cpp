#include "vertical_kalman_filter.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace
{

bool Near(double lhs, double rhs, double tolerance)
{
    return std::fabs(lhs - rhs) <= tolerance;
}

int Fail(const char* message)
{
    std::cerr << message << std::endl;
    return EXIT_FAILURE;
}

}  // namespace

int main()
{
    state_estimator_1d::VerticalProcessNoise noise;
    noise.acceleration_std = 0.2;
    noise.acceleration_bias_random_walk_std = 0.01;
    noise.stance_foot_height_random_walk_std = 1e-5;
    noise.max_prediction_step_sec = 0.005;

    state_estimator_1d::VerticalKalmanFilter filter(noise);
    filter.Initialize(0.2, 0.0, 0.0, 0.1, 0.2, 0.5);

    const double position_variance_before_anchor = filter.covariance()(0, 0);
    filter.InitializeFootAnchor(0, -0.2, 0.01);
    if (!Near(filter.covariance()(0, 0), position_variance_before_anchor, 1e-12))
    {
        return Fail("Foot initialization changed the body position covariance.");
    }

    constexpr double kAccelerationBias = 0.12;
    constexpr double kDt = 0.002;
    for (int i = 0; i < 5000; ++i)
    {
        filter.Predict(kAccelerationBias, kDt);
        if (!filter.UpdateFootConstraint(0, -0.2, 0.003, 10.0))
        {
            return Fail("Valid foot constraint was rejected.");
        }
        if (!filter.UpdateVelocity(0.0, 0.01, 10.0))
        {
            return Fail("Valid zero-velocity measurement was rejected.");
        }
    }

    if (!Near(filter.acceleration_bias(), kAccelerationBias, 0.02))
    {
        return Fail("Acceleration bias did not converge.");
    }
    if (!Near(filter.position(), 0.2, 0.02) || !Near(filter.velocity(), 0.0, 0.01))
    {
        return Fail("Static contact state drifted.");
    }

    const double position_before_outlier = filter.position();
    if (filter.UpdatePosition(10.0, 0.01, 3.0))
    {
        return Fail("Position outlier passed the innovation gate.");
    }
    if (!Near(filter.position(), position_before_outlier, 1e-12))
    {
        return Fail("Rejected outlier changed the state.");
    }

    return EXIT_SUCCESS;
}
