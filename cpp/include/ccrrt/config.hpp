/**
 * @file config.hpp
 * @brief Global planner and simulation parameters (paper Section 5 defaults).
 */

#pragma once

namespace ccrrt {

/**
 * @brief Simulation and planner parameters (paper Section 5).
 *
 * Centralizes tunables for CC-RRT tree expansion, chance constraints,
 * Kalman filtering, and the sampling workspace bounds.
 */
struct PlannerConfig {
    /** @brief RRT steering distance / robot step size (paper: 0.5). */
    double expand_distance = 0.5;

    /** @brief Upper bound M on total collision probability per time step (paper: 0.2). */
    double collision_bound_M = 0.2;

    /** @brief Confidence level alpha for alpha-confidence sets (paper: 0.99). */
    double confidence_alpha = 0.99;

    /** @brief Number of Monte Carlo samples per collision probability estimate (paper: 1000). */
    int mc_samples = 1000;

    /** @brief Maximum RRT iterations before declaring planning failure. */
    int max_iterations = 5000;

    /** @brief Goal-biased sampling rate in percent [0, 100] (paper uses 5%). */
    int goal_sample_rate = 5;

    /** @brief Initial position variance (diagonal of Px0; paper: 0.2). */
    double initial_variance = 0.2;

    /** @brief Process noise variance added per prediction step (diagonal of Pω; paper: 0.2). */
    double process_noise = 0.2;

    /** @brief Measurement noise variance (diagonal of Q; paper: 0.2). */
    double measurement_noise = 0.2;

    /** @brief Lower bound of the random sampling workspace. */
    double bounds_min = -2.0;

    /** @brief Upper bound of the random sampling workspace. */
    double bounds_max = 17.0;

    /** @brief Seed for reproducible Monte Carlo and RRT sampling. */
    unsigned int rng_seed = 42;
};

}  // namespace ccrrt
