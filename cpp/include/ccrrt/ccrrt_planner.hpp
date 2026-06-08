/**
 * @file ccrrt_planner.hpp
 * @brief Single-agent Chance-Constrained RRT planner (paper Section 2.2).
 */

#pragma once

#include "ccrrt/collision_checker.hpp"
#include "ccrrt/config.hpp"
#include "ccrrt/types.hpp"

#include <random>
#include <vector>

namespace ccrrt {

/**
 * @brief Internal node stored in the CC-RRT search tree.
 */
struct RRTNode {
    /** @brief Mean position of this tree node. */
    Vec2 position;

    /** @brief Position variance accumulated from the root (Eq. 5). */
    double variance = 0.2;

    /** @brief Index of the parent node in the tree vector (-1 for root). */
    int parent = -1;

    /** @brief Obstacle-proximity cost assigned at node creation. */
    double cost = 0.0;
};

/**
 * @brief Single-agent Chance-Constrained RRT planner (paper Section 2.2).
 *
 * Grows a random tree from a Gaussian start state toward a goal while rejecting
 * nodes and edges that violate chance constraints against static and moving obstacles.
 */
class CCRRTPlanner {
public:
    /**
     * @brief Constructs a planner with injected dependencies.
     * @param config Planner parameters (step size, iteration limit, bounds).
     * @param collision_checker Chance-constraint checker for node/edge validation.
     * @param rng Shared random engine for sampling.
     */
    CCRRTPlanner(
        const PlannerConfig& config,
        ICollisionChecker& collision_checker,
        std::mt19937& rng);

    /**
     * @brief Plans a probabilistically feasible trajectory from start to goal.
     *
     * Runs the standard RRT loop (sample, nearest, steer, chance-constrained accept)
     * until the goal is reached or max_iterations is exceeded.
     *
     * @param start Gaussian start state (mean + variance).
     * @param goal Target position.
     * @param static_obstacles Fixed environment obstacles.
     * @param agent_predictions Higher-priority agents' broadcast trajectories.
     * @param dynamic_predictions Dynamic obstacle mean trajectories with uncertainty.
     * @param time_offset Index offset for aligning with prediction horizons.
     * @return Feasible trajectory, or an empty trajectory on failure.
     */
    Trajectory plan(
        const GaussianState& start,
        const Vec2& goal,
        const std::vector<StaticObstacle>& static_obstacles,
        const std::vector<TrajectoryPrediction>& agent_predictions,
        const std::vector<TrajectoryPrediction>& dynamic_predictions,
        int time_offset = 0);

private:
    /** @brief Returns the index of the tree node closest to @p sample. */
    int nearestNodeIndex(const std::vector<RRTNode>& nodes, const Vec2& sample) const;

    /** @brief Steers from @p from toward @p to by at most expand_distance. */
    Vec2 steer(const Vec2& from, const Vec2& to) const;

    /** @brief Reconstructs the goal-to-start path by following parent pointers. */
    Trajectory extractPath(const std::vector<RRTNode>& nodes, int goal_parent_index) const;

    /** @brief Computes obstacle-proximity penalty for a candidate node. */
    double nodeCost(
        const RRTNode& node,
        const std::vector<StaticObstacle>& static_obstacles) const;

    PlannerConfig config_;
    ICollisionChecker& collision_checker_;
    std::mt19937& rng_;
};

}  // namespace ccrrt
