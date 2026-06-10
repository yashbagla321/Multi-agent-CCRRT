/**
 * @file kalman_filter.cpp
 * @brief Scalar isotropic Kalman predict/update for receding-horizon localization.
 *
 * Implements paper Eq. 5 (predict) and Eq. 6–7 (update) for identity dynamics
 * with diagonal Px0, Pω, and Q.
 *
 * @see ccrrt/kalman_filter.hpp
 */

#include "ccrrt/kalman_filter.hpp"

#include <cmath>

namespace ccrrt {

KalmanFilter::KalmanFilter(const PlannerConfig& config) : config_(config) {}

GaussianState KalmanFilter::predict(const GaussianState& state) const {
    GaussianState next = state;
    // Eq. 5: P_{t+j} = P_{t+j-1} + P_omega (isotropic scalar form).
    next.variance = state.variance + config_.process_noise;
    return next;
}

GaussianState KalmanFilter::update(const GaussianState& predicted, const Vec2& measurement) const {
    const double prior_variance = predicted.variance;
    const double measurement_variance = config_.measurement_noise;

    // Eq. 7: Kalman gain for scalar variance.
    const double kalman_gain = prior_variance / (prior_variance + measurement_variance);

    GaussianState updated;
    updated.mean.x = predicted.mean.x + kalman_gain * (measurement.x - predicted.mean.x);
    updated.mean.y = predicted.mean.y + kalman_gain * (measurement.y - predicted.mean.y);
    // Eq. 6: posterior covariance.
    updated.variance = (1.0 - kalman_gain) * prior_variance;
    return updated;
}

GaussianState KalmanFilter::measurementUpdate(
    const GaussianState& state,
    const Vec2& measurement) const {
    GaussianState updated = update(predict(state), measurement);
    updated.variance = config_.measurement_noise;
    return updated;
}

Vec2 KalmanFilter::simulateMeasurement(const Vec2& true_position, std::mt19937& rng) const {
    std::normal_distribution<double> noise(0.0, std::sqrt(config_.measurement_noise));
    return {true_position.x + noise(rng), true_position.y + noise(rng)};
}

}  // namespace ccrrt
