/**
 * @file legacy_collision_checker.cpp
 * @brief Deterministic collision checks matching Multiagent CCRRT.py.
 */

#include "ccrrt/legacy_collision_checker.hpp"

#include "ccrrt/geometry.hpp"

namespace ccrrt {

LegacyPythonCollisionChecker::LegacyPythonCollisionChecker(const PlannerConfig& config) : config_(config) {}

bool LegacyPythonCollisionChecker::edgeViolatesStaticObstacles(
    const Vec2& edge_start,
    const Vec2& edge_end,
    double child_variance,
    const std::vector<StaticObstacle>& static_obstacles) const {
    for (const auto& obstacle : static_obstacles) {
        const double distance = pointSegmentDistance(obstacle.center, edge_start, edge_end);
        const double clearance =
            obstacle.radius + config_.legacy_p_safe * (child_variance / 2.0);
        if (distance <= clearance) {
            return true;
        }
    }
    return false;
}

bool LegacyPythonCollisionChecker::edgeViolatesPrediction(
    const Vec2& edge_start,
    const Vec2& edge_end,
    double child_variance,
    const TrajectoryPrediction& prediction,
    int time_index) const {
    if (prediction.nodes.size() < 2) {
        return false;
    }

    const TrajectoryNode first = prediction.nodeAt(static_cast<std::size_t>(time_index));
    const TrajectoryNode second = prediction.nodeAt(static_cast<std::size_t>(time_index + 1));

    const std::vector<TrajectoryNode> endpoints = {first, second};
    for (const auto& endpoint : endpoints) {
        const double distance = pointSegmentDistance(endpoint.position, edge_start, edge_end);
        const double clearance =
            (endpoint.variance / 2.0) + config_.legacy_p_safe * (child_variance / 2.0);
        if (distance <= clearance) {
            return true;
        }
    }

    return segmentsIntersect(
        edge_start,
        edge_end,
        first.position,
        second.position);
}

bool LegacyPythonCollisionChecker::isNodeSafe(
    const GaussianState& robot,
    const std::vector<StaticObstacle>& static_obstacles,
    const std::vector<TrajectoryPrediction>& agent_predictions,
    const std::vector<TrajectoryPrediction>& dynamic_predictions,
    int time_index) const {
    for (const auto& obstacle : static_obstacles) {
        const double distance = robot.mean.distance(obstacle.center);
        const double clearance =
            obstacle.radius + config_.legacy_p_safe * (robot.variance / 2.0);
        if (distance <= clearance) {
            return false;
        }
    }

    for (const auto& prediction : agent_predictions) {
        const TrajectoryNode node = prediction.nodeAt(static_cast<std::size_t>(time_index));
        const double distance = robot.mean.distance(node.position);
        const double clearance =
            (node.variance / 2.0) + config_.legacy_p_safe * (robot.variance / 2.0);
        if (distance <= clearance) {
            return false;
        }
    }

    for (const auto& prediction : dynamic_predictions) {
        const TrajectoryNode node = prediction.nodeAt(static_cast<std::size_t>(time_index));
        const double distance = robot.mean.distance(node.position);
        const double clearance =
            (node.variance / 2.0) + config_.legacy_p_safe * (robot.variance / 2.0);
        if (distance <= clearance) {
            return false;
        }
    }

    return true;
}

bool LegacyPythonCollisionChecker::isEdgeSafe(
    const Vec2& edge_start,
    const Vec2& edge_end,
    double robot_variance,
    const std::vector<StaticObstacle>& static_obstacles,
    const std::vector<TrajectoryPrediction>& agent_predictions,
    const std::vector<TrajectoryPrediction>& dynamic_predictions,
    int time_index) const {
    if (edgeViolatesStaticObstacles(edge_start, edge_end, robot_variance, static_obstacles)) {
        return false;
    }

    for (const auto& prediction : agent_predictions) {
        if (edgeViolatesPrediction(edge_start, edge_end, robot_variance, prediction, time_index)) {
            return false;
        }
    }

    for (const auto& prediction : dynamic_predictions) {
        if (edgeViolatesPrediction(edge_start, edge_end, robot_variance, prediction, time_index)) {
            return false;
        }
    }

    return true;
}

double LegacyPythonCollisionChecker::estimateCollisionProbability(
    const GaussianState& robot,
    const std::vector<StaticObstacle>& static_obstacles,
    const std::vector<TrajectoryPrediction>& agent_predictions,
    const std::vector<TrajectoryPrediction>& dynamic_predictions,
    int time_index) const {
    return isNodeSafe(
               robot,
               static_obstacles,
               agent_predictions,
               dynamic_predictions,
               time_index)
               ? 0.0
               : 1.0;
}

}  // namespace ccrrt
