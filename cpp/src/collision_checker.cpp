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
    for (const auto& obstacle : static_obstacles) {
        if (sample.distance(obstacle.center) <= obstacle.radius) {
            return true;
        }
    }

    for (const auto& prediction : agent_predictions) {
        if (prediction.empty()) {
            continue;
        }
        if (pointInsideConfidenceSet(sample, prediction.nodeAt(static_cast<std::size_t>(time_index)))) {
            return true;
        }
    }

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

    for (const auto& obstacle : static_obstacles) {
        if (pointSegmentDistance(obstacle.center, edge_start, edge_end) <= obstacle.radius) {
            return false;
        }
    }

    return true;
}

}  // namespace ccrrt
