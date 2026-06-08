#pragma once

#include "ccrrt/collision_checker.hpp"
#include "ccrrt/config.hpp"
#include "ccrrt/types.hpp"

#include <random>
#include <vector>

namespace ccrrt {

struct RRTNode {
    Vec2 position;
    double variance = 0.2;
    int parent = -1;
    double cost = 0.0;
};

class CCRRTPlanner {
public:
    CCRRTPlanner(
        const PlannerConfig& config,
        ICollisionChecker& collision_checker,
        std::mt19937& rng);

    Trajectory plan(
        const GaussianState& start,
        const Vec2& goal,
        const std::vector<StaticObstacle>& static_obstacles,
        const std::vector<TrajectoryPrediction>& agent_predictions,
        const std::vector<TrajectoryPrediction>& dynamic_predictions,
        int time_offset = 0);

private:
    int nearestNodeIndex(const std::vector<RRTNode>& nodes, const Vec2& sample) const;
    Vec2 steer(const Vec2& from, const Vec2& to) const;
    Trajectory extractPath(const std::vector<RRTNode>& nodes, int goal_parent_index) const;
    double nodeCost(
        const RRTNode& node,
        const std::vector<StaticObstacle>& static_obstacles) const;

    PlannerConfig config_;
    ICollisionChecker& collision_checker_;
    std::mt19937& rng_;
};

}  // namespace ccrrt
