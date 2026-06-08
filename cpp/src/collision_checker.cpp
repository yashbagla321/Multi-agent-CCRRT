/**
 * @file collision_checker.cpp
 * @brief Monte Carlo chance-constrained collision checking.
 *
 * Samples from the planning robot's Gaussian and estimates the fraction of
 * samples that fall inside static obstacles or moving objects' alpha-confidence sets.
 * Edge checks additionally reject geometric intersections with broadcast paths.
 *
 * @see ccrrt/collision_checker.hpp
 */

#include "ccrrt/collision_checker.hpp"

#include "ccrrt/geometry.hpp"

#include <cmath>

namespace ccrrt {

MonteCarloCollisionChecker::MonteCarloCollisionChecker(const PlannerConfig& config, std::mt19937& rng)
    : config_(config), rng_(rng), normal_(0.0, 1.0) {}

Vec2 MonteCarloCollisionChecker::samplePosition(const GaussianState& state) const {
    const double sigma = std::sqrt(std::max(state.variance, 1e-9));
    return {state.mean.x + normal_(rng_) * sigma, state.mean.y + normal_(rng_) * sigma};
}

bool MonteCarloCollisionChecker::pointInsideConfidenceSet(
    const Vec2& sample,
    const TrajectoryNode& object) const {
    const double radius = std::sqrt(object.variance * chiSquaredThreshold2D(config_.confidence_alpha));
    return sample.distance(object.position) <= radius;
}

bool MonteCarloCollisionChecker::sampleInCollision(
    const Vec2& sample,
    const std::vector<StaticObstacle>& static_obstacles,
    const std::vector<TrajectoryPrediction>& agent_predictions,
    const std::vector<TrajectoryPrediction>& dynamic_predictions,
    int time_index) const {
    // Static obstacles: sample inside disc => collision.
    for (const auto& obstacle : static_obstacles) {
        if (sample.distance(obstacle.center) <= obstacle.radius) {
            return true;
        }
    }

    // Other agents: sample inside alpha-confidence set of predicted position.
    for (const auto& prediction : agent_predictions) {
        if (prediction.empty()) {
            continue;
        }
        if (pointInsideConfidenceSet(sample, prediction.nodeAt(static_cast<std::size_t>(time_index)))) {
            return true;
        }
    }

    // Dynamic obstacles: same confidence-set test as agents.
    for (const auto& prediction : dynamic_predictions) {
        if (prediction.empty()) {
            continue;
        }
        if (pointInsideConfidenceSet(sample, prediction.nodeAt(static_cast<std::size_t>(time_index)))) {
            return true;
        }
    }

    return false;
}

bool MonteCarloCollisionChecker::edgeIntersectsPredictedPath(
    const Vec2& edge_start,
    const Vec2& edge_end,
    const TrajectoryPrediction& prediction,
    int time_index) const {
    if (prediction.nodes.size() < 2) {
        return false;
    }

    const std::size_t start_index = static_cast<std::size_t>(std::max(time_index, 0));
    if (start_index + 1 >= prediction.nodes.size()) {
        return false;
    }

    // Reject if the candidate edge crosses any segment of the broadcast path.
    for (std::size_t i = start_index; i + 1 < prediction.nodes.size(); ++i) {
        if (segmentsIntersect(
                edge_start,
                edge_end,
                prediction.nodes[i].position,
                prediction.nodes[i + 1].position)) {
            return true;
        }
    }
    return false;
}

double MonteCarloCollisionChecker::estimateCollisionProbability(
    const GaussianState& robot,
    const std::vector<StaticObstacle>& static_obstacles,
    const std::vector<TrajectoryPrediction>& agent_predictions,
    const std::vector<TrajectoryPrediction>& dynamic_predictions,
    int time_index) const {
    int collision_count = 0;
    for (int i = 0; i < config_.mc_samples; ++i) {
        if (sampleInCollision(
                samplePosition(robot),
                static_obstacles,
                agent_predictions,
                dynamic_predictions,
                time_index)) {
            ++collision_count;
        }
    }
    return static_cast<double>(collision_count) / static_cast<double>(config_.mc_samples);
}

bool MonteCarloCollisionChecker::isNodeSafe(
    const GaussianState& robot,
    const std::vector<StaticObstacle>& static_obstacles,
    const std::vector<TrajectoryPrediction>& agent_predictions,
    const std::vector<TrajectoryPrediction>& dynamic_predictions,
    int time_index) const {
    return estimateCollisionProbability(
               robot,
               static_obstacles,
               agent_predictions,
               dynamic_predictions,
               time_index) <= config_.collision_bound_M;
}

bool MonteCarloCollisionChecker::isEdgeSafe(
    const Vec2& edge_start,
    const Vec2& edge_end,
    double robot_variance,
    const std::vector<StaticObstacle>& static_obstacles,
    const std::vector<TrajectoryPrediction>& agent_predictions,
    const std::vector<TrajectoryPrediction>& dynamic_predictions,
    int time_index) const {
    // 1. Chance constraint at the child node (paper: check probability at edge end).
    GaussianState endpoint;
    endpoint.mean = edge_end;
    endpoint.variance = robot_variance;

    if (!isNodeSafe(
            endpoint,
            static_obstacles,
            agent_predictions,
            dynamic_predictions,
            time_index + 1)) {
        return false;
    }

    // 2. Geometric intersection with other agents' broadcast trajectories (Section 4.1).
    for (const auto& prediction : agent_predictions) {
        if (edgeIntersectsPredictedPath(edge_start, edge_end, prediction, time_index)) {
            return false;
        }
    }

    for (const auto& prediction : dynamic_predictions) {
        if (edgeIntersectsPredictedPath(edge_start, edge_end, prediction, time_index)) {
            return false;
        }
    }

    // 3. Static obstacle segment clearance.
    for (const auto& obstacle : static_obstacles) {
        if (pointSegmentDistance(obstacle.center, edge_start, edge_end) <= obstacle.radius) {
            return false;
        }
    }

    return true;
}

}  // namespace ccrrt
