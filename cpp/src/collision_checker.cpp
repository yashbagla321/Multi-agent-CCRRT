/**
 * @file collision_checker.cpp
 * @brief Monte Carlo chance-constrained collision checking.
 *
 * Samples from the planning robot's Gaussian and estimates the fraction of
 * samples that fall inside static obstacles or moving objects' alpha-confidence sets.
 * Edge checks additionally reject static obstacle segment intersections.
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
    const double variance =
        std::min(object.variance, config_.max_prediction_variance);
    const double radius = std::sqrt(variance * chiSquaredThreshold2D(config_.confidence_alpha));
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

    // 2. Static obstacle segment clearance.
    for (const auto& obstacle : static_obstacles) {
        if (pointSegmentDistance(obstacle.center, edge_start, edge_end) <= obstacle.radius) {
            return false;
        }
    }

    // 3. Reject edges that cross higher-priority broadcast segments (paper Section 4.1).
    for (const auto& prediction : agent_predictions) {
        if (prediction.nodes.size() < 2) {
            continue;
        }
        const TrajectoryNode first = prediction.nodeAt(static_cast<std::size_t>(time_index));
        const TrajectoryNode second = prediction.nodeAt(static_cast<std::size_t>(time_index + 1));
        if (segmentsIntersect(edge_start, edge_end, first.position, second.position)) {
            return false;
        }
    }

    for (const auto& prediction : dynamic_predictions) {
        if (prediction.nodes.size() < 2) {
            continue;
        }
        const TrajectoryNode first = prediction.nodeAt(static_cast<std::size_t>(time_index));
        const TrajectoryNode second = prediction.nodeAt(static_cast<std::size_t>(time_index + 1));
        if (segmentsIntersect(edge_start, edge_end, first.position, second.position)) {
            return false;
        }
    }

    return true;
}

bool isSpanEdgeSafe(
    const ICollisionChecker& checker,
    const Vec2& edge_start,
    const Vec2& edge_end,
    int time_start,
    int time_end,
    const std::vector<double>& variances_at_integer_times,
    const std::vector<StaticObstacle>& static_obstacles,
    const std::vector<TrajectoryPrediction>& agent_predictions,
    const std::vector<TrajectoryPrediction>& dynamic_predictions) {
    if (time_end < time_start) {
        return false;
    }

    const int span = time_end - time_start;
    if (span == 0) {
        GaussianState endpoint;
        endpoint.mean = edge_end;
        endpoint.variance = variances_at_integer_times.empty() ? 0.2 : variances_at_integer_times.back();
        return checker.isNodeSafe(
            endpoint,
            static_obstacles,
            agent_predictions,
            dynamic_predictions,
            time_end);
    }

    if (span == 1) {
        if (variances_at_integer_times.size() < 2) {
            return false;
        }
        return checker.isEdgeSafe(
            edge_start,
            edge_end,
            variances_at_integer_times[1],
            static_obstacles,
            agent_predictions,
            dynamic_predictions,
            time_start);
    }

    if (static_cast<int>(variances_at_integer_times.size()) != span + 1) {
        return false;
    }

    for (const auto& obstacle : static_obstacles) {
        if (pointSegmentDistance(obstacle.center, edge_start, edge_end) <= obstacle.radius) {
            return false;
        }
    }

    for (int time_index = time_start + 1; time_index <= time_end; ++time_index) {
        const double alpha =
            static_cast<double>(time_index - time_start) / static_cast<double>(span);
        GaussianState sample_state;
        sample_state.mean = edge_start + (edge_end - edge_start) * alpha;
        sample_state.variance = variances_at_integer_times[static_cast<std::size_t>(time_index - time_start)];
        if (!checker.isNodeSafe(
                sample_state,
                static_obstacles,
                agent_predictions,
                dynamic_predictions,
                time_index)) {
            return false;
        }
    }

    for (int time_index = time_start; time_index < time_end; ++time_index) {
        for (const auto& prediction : agent_predictions) {
            if (prediction.nodes.size() < 2) {
                continue;
            }
            const TrajectoryNode first = prediction.nodeAt(static_cast<std::size_t>(time_index));
            const TrajectoryNode second = prediction.nodeAt(static_cast<std::size_t>(time_index + 1));
            if (segmentsIntersect(edge_start, edge_end, first.position, second.position)) {
                return false;
            }
        }
        for (const auto& prediction : dynamic_predictions) {
            if (prediction.nodes.size() < 2) {
                continue;
            }
            const TrajectoryNode first = prediction.nodeAt(static_cast<std::size_t>(time_index));
            const TrajectoryNode second = prediction.nodeAt(static_cast<std::size_t>(time_index + 1));
            if (segmentsIntersect(edge_start, edge_end, first.position, second.position)) {
                return false;
            }
        }
    }

    return true;
}

}  // namespace ccrrt
