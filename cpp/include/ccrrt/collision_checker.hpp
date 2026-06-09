/**
 * @file collision_checker.hpp
 * @brief Chance-constrained collision checking interface and Monte Carlo implementation.
 */

#pragma once

#include "ccrrt/config.hpp"
#include "ccrrt/types.hpp"

#include <random>
#include <vector>

namespace ccrrt {

/**
 * @brief Abstract interface for chance-constrained collision checking.
 *
 * Allows swapping Monte Carlo, analytic, or mock implementations in tests.
 */
class ICollisionChecker {
public:
    virtual ~ICollisionChecker() = default;

    /**
     * @brief Checks whether a robot state satisfies P_collision <= M.
     *
     * @param robot Gaussian state of the planning robot at the check time.
     * @param static_obstacles Fixed circular obstacles.
     * @param agent_predictions Broadcast trajectories of other agents.
     * @param dynamic_predictions Predicted trajectories of dynamic obstacles.
     * @param time_index Horizon index into prediction trajectories.
     * @return True if estimated collision probability is at most collision_bound_M.
     */
    virtual bool isNodeSafe(
        const GaussianState& robot,
        const std::vector<StaticObstacle>& static_obstacles,
        const std::vector<TrajectoryPrediction>& agent_predictions,
        const std::vector<TrajectoryPrediction>& dynamic_predictions,
        int time_index) const = 0;

    /**
     * @brief Checks whether an RRT edge is safe to add to the tree.
     *
     * Validates the endpoint node (Monte Carlo) and rejects edges that intersect
     * static obstacle discs.
     *
     * @param edge_start Parent node position.
     * @param edge_end Candidate child node position.
     * @param robot_variance Variance at the child node.
     * @param static_obstacles Fixed circular obstacles.
     * @param agent_predictions Broadcast trajectories of other agents.
     * @param dynamic_predictions Predicted trajectories of dynamic obstacles.
     * @param time_index Horizon index for aligning with prediction trajectories.
     * @return True if the edge satisfies all chance and geometric constraints.
     */
    virtual bool isEdgeSafe(
        const Vec2& edge_start,
        const Vec2& edge_end,
        double robot_variance,
        const std::vector<StaticObstacle>& static_obstacles,
        const std::vector<TrajectoryPrediction>& agent_predictions,
        const std::vector<TrajectoryPrediction>& dynamic_predictions,
        int time_index) const = 0;

    /**
     * @brief Estimates total collision probability via Monte Carlo sampling.
     *
     * Samples from the robot distribution and counts the fraction landing inside
     * any static obstacle or alpha-confidence set of a moving object.
     *
     * @param robot Gaussian state of the planning robot.
     * @param static_obstacles Fixed circular obstacles.
     * @param agent_predictions Broadcast trajectories of other agents.
     * @param dynamic_predictions Predicted trajectories of dynamic obstacles.
     * @param time_index Horizon index into prediction trajectories.
     * @return Estimated collision probability in [0, 1].
     */
    virtual double estimateCollisionProbability(
        const GaussianState& robot,
        const std::vector<StaticObstacle>& static_obstacles,
        const std::vector<TrajectoryPrediction>& agent_predictions,
        const std::vector<TrajectoryPrediction>& dynamic_predictions,
        int time_index) const = 0;
};

/**
 * @brief Monte Carlo chance-constrained collision checker (paper Section 3.2).
 *
 * Uses mc_samples draws from the robot's Gaussian and alpha-confidence sets
 * for all moving objects. Compares the result against collision_bound_M.
 */
class MonteCarloCollisionChecker final : public ICollisionChecker {
public:
    /**
     * @brief Constructs a checker bound to a shared RNG for reproducibility.
     * @param config Planner parameters (M, alpha, mc_samples).
     * @param rng Random engine used for Monte Carlo sampling.
     */
    MonteCarloCollisionChecker(const PlannerConfig& config, std::mt19937& rng);

    bool isNodeSafe(
        const GaussianState& robot,
        const std::vector<StaticObstacle>& static_obstacles,
        const std::vector<TrajectoryPrediction>& agent_predictions,
        const std::vector<TrajectoryPrediction>& dynamic_predictions,
        int time_index) const override;

    bool isEdgeSafe(
        const Vec2& edge_start,
        const Vec2& edge_end,
        double robot_variance,
        const std::vector<StaticObstacle>& static_obstacles,
        const std::vector<TrajectoryPrediction>& agent_predictions,
        const std::vector<TrajectoryPrediction>& dynamic_predictions,
        int time_index) const override;

    double estimateCollisionProbability(
        const GaussianState& robot,
        const std::vector<StaticObstacle>& static_obstacles,
        const std::vector<TrajectoryPrediction>& agent_predictions,
        const std::vector<TrajectoryPrediction>& dynamic_predictions,
        int time_index) const override;

private:
    /** @brief Draws one sample from an isotropic Gaussian state. */
    Vec2 samplePosition(const GaussianState& state) const;

    /** @brief Returns true if a sample collides with any obstacle or confidence set. */
    bool sampleInCollision(
        const Vec2& sample,
        const std::vector<StaticObstacle>& static_obstacles,
        const std::vector<TrajectoryPrediction>& agent_predictions,
        const std::vector<TrajectoryPrediction>& dynamic_predictions,
        int time_index) const;

    /** @brief Tests membership in an object's alpha-confidence disc. */
    bool pointInsideConfidenceSet(const Vec2& sample, const TrajectoryNode& object) const;

    const PlannerConfig& config_;
    std::mt19937& rng_;
    mutable std::normal_distribution<double> normal_;
};

}  // namespace ccrrt
