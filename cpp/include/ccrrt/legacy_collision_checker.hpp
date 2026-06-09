/**
 * @file legacy_collision_checker.hpp
 * @brief Deterministic collision checks matching Multiagent CCRRT.py.
 */

#pragma once

#include "ccrrt/collision_checker.hpp"

namespace ccrrt {

/**
 * @brief Simplified chance-constrained checks from the original Python prototype.
 *
 * Uses point-to-segment clearance and segment intersection instead of Monte Carlo
 * sampling. Intended for regression against Multiagent CCRRT.py.
 */
class LegacyPythonCollisionChecker final : public ICollisionChecker {
public:
    explicit LegacyPythonCollisionChecker(const PlannerConfig& config);

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
    bool edgeViolatesStaticObstacles(
        const Vec2& edge_start,
        const Vec2& edge_end,
        double child_variance,
        const std::vector<StaticObstacle>& static_obstacles) const;

    bool edgeViolatesPrediction(
        const Vec2& edge_start,
        const Vec2& edge_end,
        double child_variance,
        const TrajectoryPrediction& prediction,
        int time_index) const;

    const PlannerConfig& config_;
};

}  // namespace ccrrt
