/**
 * @file ccrrt_planner.cpp
 * @brief Single-agent Chance-Constrained RRT tree expansion and path extraction.
 *
 * Standard RRT loop with variance growth along tree depth and chance-constrained
 * edge acceptance via ICollisionChecker.
 *
 * @see ccrrt/ccrrt_planner.hpp
 */

#include "ccrrt/ccrrt_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

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

Trajectory CCRRTPlanner::shortcutSmooth(
    const Trajectory& path,
    const std::vector<StaticObstacle>& static_obstacles,
    const std::vector<TrajectoryPrediction>& agent_predictions,
    const std::vector<TrajectoryPrediction>& dynamic_predictions,
    int time_offset) const {
    if (!config_.enable_path_smoothing || path.nodes.size() < 3) {
        return path;
    }

    Trajectory smoothed;
    smoothed.total_cost = path.total_cost;
    smoothed.nodes.push_back(path.nodes.front());
    std::size_t anchor = 0;

    while (anchor < path.nodes.size() - 1) {
        std::size_t best = anchor + 1;
        for (std::size_t j = path.nodes.size() - 1; j > anchor + 1; --j) {
            std::vector<double> variances;
            variances.reserve(j - anchor + 1);
            for (std::size_t t = anchor; t <= j; ++t) {
                variances.push_back(path.nodes[t].variance);
            }

            const int time_start = time_offset + static_cast<int>(anchor);
            const int time_end = time_offset + static_cast<int>(j);
            if (isSpanEdgeSafe(
                    collision_checker_,
                    smoothed.nodes.back().position,
                    path.nodes[j].position,
                    time_start,
                    time_end,
                    variances,
                    static_obstacles,
                    agent_predictions,
                    dynamic_predictions)) {
                best = j;
                break;
            }
        }

        const TrajectoryNode start_node = smoothed.nodes.back();
        const TrajectoryNode end_node = path.nodes[best];
        const std::size_t span = best - anchor;
        for (std::size_t step = 1; step <= span; ++step) {
            const double alpha = static_cast<double>(step) / static_cast<double>(span);
            TrajectoryNode node;
            node.position = start_node.position + (end_node.position - start_node.position) * alpha;
            node.variance = path.nodes[anchor + step].variance;
            node.time_step = static_cast<int>(smoothed.nodes.size());
            smoothed.nodes.push_back(node);
        }
        anchor = best;
    }

    return smoothed;
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

    Trajectory best_path;
    int goal_candidate_count = 0;
    constexpr int max_goal_candidates = 8;

    for (int iter = 0; iter < config_.max_iterations; ++iter) {
        // --- Sample: goal-biased or uniform over workspace ---
        Vec2 sample;
        if (percent(rng_) > config_.goal_sample_rate) {
            sample = {coord(rng_), coord(rng_)};
        } else {
            sample = goal;
        }

        const int nearest = nearestNodeIndex(nodes, sample);
        const Vec2 new_position = steer(nodes[static_cast<std::size_t>(nearest)].position, sample);

        // Tree depth from root determines prediction horizon for moving obstacles.
        int depth = 0;
        for (int index = nearest; index >= 0; index = nodes[static_cast<std::size_t>(index)].parent) {
            ++depth;
        }

        RRTNode candidate;
        candidate.position = new_position;
        if (config_.use_legacy_collision) {
            candidate.variance = nodes[static_cast<std::size_t>(nearest)].variance *
                               (1.0 + config_.variance_growth_alpha);
        } else {
            candidate.variance =
                nodes[static_cast<std::size_t>(nearest)].variance + config_.process_noise;
        }
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
            TrajectoryNode goal_node;
            goal_node.position = goal;
            if (config_.use_legacy_collision) {
                goal_node.variance = candidate.variance * (1.0 + config_.variance_growth_alpha);
            } else {
                goal_node.variance = candidate.variance + config_.process_noise;
            }

            const int goal_horizon_index = horizon_index + 1;
            if (!collision_checker_.isEdgeSafe(
                    candidate.position,
                    goal_node.position,
                    goal_node.variance,
                    static_obstacles,
                    agent_predictions,
                    dynamic_predictions,
                    goal_horizon_index)) {
                continue;
            }

            Trajectory path = extractPath(nodes, static_cast<int>(nodes.size()) - 1);
            goal_node.time_step = static_cast<int>(path.nodes.size());
            path.total_cost += nodeCost(
                RRTNode{goal_node.position, goal_node.variance, -1, 0.0},
                static_obstacles);
            path.nodes.push_back(goal_node);

            if (best_path.empty() ||
                path.total_cost < best_path.total_cost ||
                (path.total_cost == best_path.total_cost &&
                 path.nodes.size() < best_path.nodes.size())) {
                best_path = std::move(path);
            }

            ++goal_candidate_count;
            if (goal_candidate_count >= max_goal_candidates) {
                break;
            }
        }
    }

    return shortcutSmooth(
        best_path,
        static_obstacles,
        agent_predictions,
        dynamic_predictions,
        time_offset);
}

}  // namespace ccrrt
