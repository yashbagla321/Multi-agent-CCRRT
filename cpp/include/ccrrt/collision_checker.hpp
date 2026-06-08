#pragma once

#include "ccrrt/config.hpp"
#include "ccrrt/types.hpp"

#include <random>
#include <vector>

namespace ccrrt {

class ICollisionChecker {
public:
    virtual ~ICollisionChecker() = default;

    virtual bool isNodeSafe(
        const GaussianState& robot,
        const std::vector<StaticObstacle>& static_obstacles,
        const std::vector<TrajectoryPrediction>& agent_predictions,
        const std::vector<TrajectoryPrediction>& dynamic_predictions,
        int time_index) const = 0;

    virtual bool isEdgeSafe(
        const Vec2& edge_start,
        const Vec2& edge_end,
        double robot_variance,
        const std::vector<StaticObstacle>& static_obstacles,
        const std::vector<TrajectoryPrediction>& agent_predictions,
        const std::vector<TrajectoryPrediction>& dynamic_predictions,
        int time_index) const = 0;

    virtual double estimateCollisionProbability(
        const GaussianState& robot,
        const std::vector<StaticObstacle>& static_obstacles,
        const std::vector<TrajectoryPrediction>& agent_predictions,
        const std::vector<TrajectoryPrediction>& dynamic_predictions,
        int time_index) const = 0;
};

class MonteCarloCollisionChecker final : public ICollisionChecker {
public:
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
    Vec2 samplePosition(const GaussianState& state) const;
    bool sampleInCollision(
        const Vec2& sample,
        const std::vector<StaticObstacle>& static_obstacles,
        const std::vector<TrajectoryPrediction>& agent_predictions,
        const std::vector<TrajectoryPrediction>& dynamic_predictions,
        int time_index) const;

    bool pointInsideConfidenceSet(const Vec2& sample, const TrajectoryNode& object) const;
    bool edgeIntersectsPredictedPath(
        const Vec2& edge_start,
        const Vec2& edge_end,
        const TrajectoryPrediction& prediction,
        int time_index) const;

    PlannerConfig config_;
    std::mt19937& rng_;
    mutable std::normal_distribution<double> normal_;
};

}  // namespace ccrrt
