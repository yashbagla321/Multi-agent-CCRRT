#pragma once

namespace ccrrt {

/// Simulation and planner parameters from paper Section 5.
struct PlannerConfig {
    double expand_distance = 0.5;
    double collision_bound_M = 0.2;
    double confidence_alpha = 0.99;
    int mc_samples = 1000;
    int max_iterations = 5000;
    int goal_sample_rate = 5;
    double initial_variance = 0.2;
    double process_noise = 0.2;
    double measurement_noise = 0.2;
    double bounds_min = -2.0;
    double bounds_max = 17.0;
    unsigned int rng_seed = 42;
};

}  // namespace ccrrt
