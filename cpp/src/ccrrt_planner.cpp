#include "ccrrt/ccrrt_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ccrrt {

CCRRTPlanner::CCRRTPlanner(
    const PlannerConfig& config,
    ICollisionChecker& collision_checker,
    std::mt19937& rng)
    : config_(config), collision_checker_(collision_checker), rng_(rng) {}

int CCRRTPlanner::nearestNodeIndex(const std::vector<RRTNode>& nodes, const Vec2& sample) const {
    int best_index = 0;
    double best_distance = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        const double distance = nodes[i].position.distance(sample);
        if (distance < best_distance) {
            best_distance = distance;
            best_index = static_cast<int>(i);
        }
    }
    return best_index;
}

Vec2 CCRRTPlanner::steer(const Vec2& from, const Vec2& to) const {
    const Vec2 delta = to - from;
    const double distance = delta.norm();
    if (distance <= config_.expand_distance || distance <= 1e-9) {
        return to;
    }
    const double scale = config_.expand_distance / distance;
    return from + delta * scale;
}

double CCRRTPlanner::nodeCost(
    const RRTNode& node,
    const std::vector<StaticObstacle>& static_obstacles) const {
    double cost = 0.0;
    for (const auto& obstacle : static_obstacles) {
        const double distance = node.position.distance(obstacle.center);
        const double clearance = distance - obstacle.radius;
        if (clearance <= std::sqrt(node.variance)) {
            cost += std::abs(std::sqrt(node.variance) + obstacle.radius - distance) /
                    std::max(std::sqrt(node.variance), 1e-6);
        }
    }
    return cost;
}

Trajectory CCRRTPlanner::extractPath(
    const std::vector<RRTNode>& nodes,
    int goal_parent_index) const {
    Trajectory trajectory;
    int index = goal_parent_index;
    while (index >= 0) {
        TrajectoryNode node;
        node.position = nodes[static_cast<std::size_t>(index)].position;
        node.variance = nodes[static_cast<std::size_t>(index)].variance;
        node.time_step = static_cast<int>(trajectory.nodes.size());
        trajectory.total_cost += nodes[static_cast<std::size_t>(index)].cost;
        trajectory.nodes.push_back(node);
        index = nodes[static_cast<std::size_t>(index)].parent;
    }

    std::reverse(trajectory.nodes.begin(), trajectory.nodes.end());
    for (std::size_t i = 0; i < trajectory.nodes.size(); ++i) {
        trajectory.nodes[i].time_step = static_cast<int>(i);
    }
    return trajectory;
}

Trajectory CCRRTPlanner::plan(
    const GaussianState& start,
    const Vec2& goal,
    const std::vector<StaticObstacle>& static_obstacles,
    const std::vector<TrajectoryPrediction>& agent_predictions,
    const std::vector<TrajectoryPrediction>& dynamic_predictions,
    int time_offset) {
    std::vector<RRTNode> nodes;
    RRTNode root;
    root.position = start.mean;
    root.variance = start.variance;
    root.parent = -1;
    nodes.push_back(root);

    std::uniform_real_distribution<double> coord(config_.bounds_min, config_.bounds_max);
    std::uniform_int_distribution<int> percent(0, 100);

    int goal_parent = -1;

    for (int iter = 0; iter < config_.max_iterations; ++iter) {
        Vec2 sample;
        if (percent(rng_) > config_.goal_sample_rate) {
            sample = {coord(rng_), coord(rng_)};
        } else {
            sample = goal;
        }

        const int nearest = nearestNodeIndex(nodes, sample);
        const Vec2 new_position = steer(nodes[static_cast<std::size_t>(nearest)].position, sample);

        int depth = 0;
        for (int index = nearest; index >= 0; index = nodes[static_cast<std::size_t>(index)].parent) {
            ++depth;
        }

        RRTNode candidate;
        candidate.position = new_position;
        candidate.variance =
            nodes[static_cast<std::size_t>(nearest)].variance + config_.process_noise;
        candidate.parent = nearest;
        candidate.cost = nodeCost(candidate, static_obstacles);

        const int horizon_index = time_offset + depth;
        if (!collision_checker_.isEdgeSafe(
                nodes[static_cast<std::size_t>(nearest)].position,
                candidate.position,
                candidate.variance,
                static_obstacles,
                agent_predictions,
                dynamic_predictions,
                horizon_index)) {
            continue;
        }

        nodes.push_back(candidate);

        if (candidate.position.distance(goal) <= config_.expand_distance) {
            goal_parent = static_cast<int>(nodes.size()) - 1;
            break;
        }
    }

    if (goal_parent < 0) {
        return {};
    }

    Trajectory path = extractPath(nodes, goal_parent);

    TrajectoryNode goal_node;
    goal_node.position = goal;
    goal_node.variance = nodes[static_cast<std::size_t>(goal_parent)].variance + config_.process_noise;
    goal_node.time_step = static_cast<int>(path.nodes.size());
    path.nodes.push_back(goal_node);
    return path;
}

}  // namespace ccrrt
