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

    /**
     * @brief Distance traveled per simulation timestep during execution.
     *
     * Smaller values produce smoother executed paths (more steps per RRT edge).
     * Set to 0 or a value >= expand_distance to move one full RRT edge per timestep.
     */
    double motion_step = 0.2;

    /** @brief Greedy shortcut smoothing on extracted RRT paths before execution. */
    bool enable_path_smoothing = false;

    /** @brief Upper bound M on total collision probability per time step (paper: 0.2). */
    double collision_bound_M = 0.2;

    /** @brief Confidence level alpha for alpha-confidence sets (paper: 0.99). */
    double confidence_alpha = 0.99;

    /** @brief Number of Monte Carlo samples per collision probability estimate (paper: 1000). */
    int mc_samples = 1000;

    /** @brief Maximum RRT iterations before declaring planning failure. */
    int max_iterations = 5000;

    /** @brief Maximum simulation timesteps before termination (Algorithm 2). */
    int max_timesteps = 500;

    /** @brief Goal-biased sampling rate in percent [0, 100] (paper uses 5%). */
    int goal_sample_rate = 5;

    /** @brief Initial position variance (diagonal of Px0; paper: 0.2). */
    double initial_variance = 0.2;

    /** @brief Process noise variance added per prediction step (diagonal of Pω; paper: 0.2). */
    double process_noise = 0.2;

    /**
     * @brief Cap on variance used for other agents / dynamic obstacle confidence discs.
     *
     * Prevents late-horizon prediction tubes from growing large enough to block the
     * entire workspace during RRT planning (variance still grows uncapped on the ego tree).
     */
    double max_prediction_variance = 0.8;

    /** @brief Measurement noise variance (diagonal of Q; paper: 0.2). */
    double measurement_noise = 0.2;

    /** @brief Lower bound of the random sampling workspace. */
    double bounds_min = -2.0;

    /** @brief Upper bound of the random sampling workspace. */
    double bounds_max = 17.0;

    /** @brief Seed for reproducible Monte Carlo and RRT sampling. */
    unsigned int rng_seed = 42;

    /**
     * @brief Use deterministic checks from Multiagent CCRRT.py instead of Monte Carlo.
     *
     * Enables multiplicative variance growth (variance_growth_alpha) and legacy_p_safe.
     */
    bool use_legacy_collision = false;

    /** @brief Safety factor in legacy mode (Python p_safe, default 0.8). */
    double legacy_p_safe = 0.8;

    /**
     * @brief Multiplicative variance growth per tree edge in legacy mode.
     *
     * Python: covariance *= (1 + alpha) with alpha = 0.1.
     */
    double variance_growth_alpha = 0.1;
};

/** @brief Planner settings aligned with Multiagent CCRRT.py defaults. */
inline PlannerConfig pythonCompatPlannerConfig() {
    PlannerConfig config;
    config.expand_distance = 1.0;
    config.max_iterations = 500;
    config.goal_sample_rate = 5;
    config.initial_variance = 0.2;
    config.bounds_min = -2.0;
    config.bounds_max = 15.0;
    config.use_legacy_collision = true;
    config.legacy_p_safe = 0.8;
    config.variance_growth_alpha = 0.1;
    config.rng_seed = 42;
    return config;
}

}  // namespace ccrrt
