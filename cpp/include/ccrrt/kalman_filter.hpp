/**
 * @file kalman_filter.hpp
 * @brief Isotropic Kalman predict/update for receding-horizon localization (Eq. 5–7).
 */

#pragma once

#include "ccrrt/config.hpp"
#include "ccrrt/types.hpp"

#include <random>

namespace ccrrt {

/**
 * @brief Scalar isotropic Kalman filter for planar position (paper Eq. 5–7).
 *
 * Implements predict/update steps for identity dynamics with GPS-like measurements.
 * Used in the receding-horizon loop to shrink uncertainty after each executed step.
 */
class KalmanFilter {
public:
    /**
     * @brief Constructs a filter using noise parameters from @p config.
     * @param config Planner configuration (process and measurement noise).
     */
    explicit KalmanFilter(const PlannerConfig& config);

    /**
     * @brief Predicts the next state covariance (Eq. 5).
     *
     * Mean is unchanged; variance increases by process_noise.
     *
     * @param state Current Gaussian belief.
     * @return Predicted state before measurement incorporation.
     */
    GaussianState predict(const GaussianState& state) const;

    /**
     * @brief Incorporates a position measurement (Eq. 6–7).
     *
     * @param predicted Predicted state from predict().
     * @param measurement Observed position (e.g. simulated GPS reading).
     * @return Updated Gaussian belief with reduced variance.
     */
    GaussianState update(const GaussianState& predicted, const Vec2& measurement) const;

    /**
     * @brief Generates a noisy GPS measurement for simulation.
     *
     * Adds zero-mean Gaussian noise with variance measurement_noise to
     * @p true_position.
     *
     * @param true_position Actual robot position.
     * @param rng Random number generator for reproducible simulations.
     * @return Noisy measurement vector.
     */
    Vec2 simulateMeasurement(const Vec2& true_position, std::mt19937& rng) const;

private:
    PlannerConfig config_;
};

}  // namespace ccrrt
