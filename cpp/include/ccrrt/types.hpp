/**
 * @file types.hpp
 * @brief Core value types: positions, Gaussian states, trajectories, environment, results.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace ccrrt {

/**
 * @brief Two-dimensional position vector in the planning workspace.
 */
struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}

    /** @brief Component-wise vector addition. */
    Vec2 operator+(const Vec2& other) const { return {x + other.x, y + other.y}; }

    /** @brief Component-wise vector subtraction. */
    Vec2 operator-(const Vec2& other) const { return {x - other.x, y - other.y}; }

    /** @brief Scalar multiplication. */
    Vec2 operator*(double scale) const { return {x * scale, y * scale}; }

    /** @brief Euclidean norm (length) of the vector. */
    double norm() const { return std::sqrt(x * x + y * y); }

    /** @brief Euclidean distance to another point. */
    double distance(const Vec2& other) const { return (*this - other).norm(); }
};

/**
 * @brief Gaussian belief over a robot's planar position (isotropic variance).
 *
 * Used for chance-constrained collision checking and Kalman filter updates.
 */
struct GaussianState {
    /** @brief Mean (expected) position. */
    Vec2 mean;

    /** @brief Isotropic position variance (scalar; paper uses 0.2·I). */
    double variance = 0.2;
};

/**
 * @brief A single waypoint on a planned or predicted trajectory.
 */
struct TrajectoryNode {
    /** @brief Mean position at this time step. */
    Vec2 position;

    /** @brief Position variance at this time step. */
    double variance = 0.2;

    /** @brief Discrete time index along the trajectory. */
    int time_step = 0;
};

/**
 * @brief A complete path from start toward goal produced by CC-RRT.
 */
struct Trajectory {
    /** @brief Ordered sequence of nodes from start to goal. */
    std::vector<TrajectoryNode> nodes;

    /** @brief Accumulated obstacle-proximity cost along the path. */
    double total_cost = 0.0;

    /** @brief Returns true if the trajectory contains no nodes. */
    bool empty() const { return nodes.empty(); }

    /** @brief Number of nodes in the trajectory. */
    std::size_t size() const { return nodes.size(); }
};

/**
 * @brief A fixed circular obstacle in the environment.
 */
struct StaticObstacle {
    /** @brief Center of the obstacle disc. */
    Vec2 center;

    /** @brief Radius of the obstacle disc. */
    double radius = 1.0;
};

/**
 * @brief Predicted future trajectory of a moving agent or dynamic obstacle.
 *
 * Each node carries a mean position and growing variance (Eq. 5 in the paper).
 * Lower-priority agents use these predictions for chance-constrained planning.
 */
struct TrajectoryPrediction {
    /** @brief Time-indexed mean positions with associated uncertainties. */
    std::vector<TrajectoryNode> nodes;

    /** @brief Returns true if no prediction nodes are available. */
    bool empty() const { return nodes.empty(); }

    /**
     * @brief Returns the prediction node at a given time index.
     * @param index Time index; clamped to the last node if out of range.
     * @return Trajectory node at @p index, or a default node if empty.
     */
    TrajectoryNode nodeAt(std::size_t index) const;
};

/**
 * @brief Static specification of an agent before simulation starts.
 */
struct AgentSpec {
    /** @brief Unique agent identifier. */
    int id = 0;

    /** @brief Priority order (lower value = higher priority). */
    int priority = 0;

    /** @brief Initial mean start position. */
    Vec2 start;

    /** @brief Goal position. */
    Vec2 goal;

    /** @brief Human-readable label (e.g. "red", "blue"). */
    std::string name;
};

/**
 * @brief Specification of a dynamic obstacle with a known mean path.
 */
struct DynamicObstacleSpec {
    /** @brief Unique obstacle identifier. */
    int id = 0;

    /** @brief Predicted mean waypoints (deterministic path). */
    std::vector<Vec2> waypoints;

    /** @brief Initial position variance at the first waypoint. */
    double initial_variance = 0.2;

    /**
     * @brief Optional per-waypoint variance (overrides initial_variance + process_noise).
     *
     * When non-empty, must match waypoints.size(). Used by python_reference to mirror
     * Multiagent CCRRT.py obstaclePath variance schedule.
     */
    std::vector<double> variance_per_waypoint;
};

/**
 * @brief Complete simulation environment: bounds, obstacles, agents, and dynamic objects.
 */
struct Environment {
    /** @brief Lower bound of the planning workspace. */
    double bounds_min = -2.0;

    /** @brief Upper bound of the planning workspace. */
    double bounds_max = 17.0;

    /** @brief Fixed circular obstacles. */
    std::vector<StaticObstacle> static_obstacles;

    /** @brief Agent start/goal/priority specifications. */
    std::vector<AgentSpec> agents;

    /** @brief Dynamic obstacles with predictable mean trajectories. */
    std::vector<DynamicObstacleSpec> dynamic_obstacles;
};

/**
 * @brief One executed step recorded during receding-horizon simulation.
 */
struct ExecutedStep {
    /** @brief Agent that executed this step. */
    int agent_id = 0;

    /** @brief Global simulation timestep. */
    int timestep = 0;

    /** @brief Actual mean position after the step. */
    Vec2 position;

    /** @brief Position variance after Kalman update. */
    double variance = 0.2;

    /** @brief True if the agent adopted a new plan at this step. */
    bool replanned = false;
};

/**
 * @brief Aggregated output of a multi-agent simulation run.
 */
struct SimulationResult {
    /** @brief Name of the scenario that was executed. */
    std::string scenario_name;

    /** @brief Executed path per agent (index matches sorted priority order). */
    std::vector<std::vector<ExecutedStep>> agent_paths;

    /** @brief Total number of replanning events across all agents. */
    int replan_count = 0;

    /** @brief True if every agent reached its goal within the step limit. */
    bool success = false;

    /** @brief Wall-clock runtime of MultiAgentPlanner::run() in milliseconds. */
    double elapsed_ms = 0.0;

    /** @brief Sum of executed steps across all agents. */
    int total_steps = 0;

    /** @brief Maximum global timestep reached before termination. */
    int max_timestep = 0;
};

/**
 * @brief Mutable per-agent state maintained during simulation (Algorithm 2).
 */
struct AgentRuntime {
    /** @brief Static agent specification. */
    AgentSpec spec;

    /** @brief Current Gaussian belief over position. */
    GaussianState state;

    /** @brief Active planned trajectory (receding horizon). */
    Trajectory planned;

    /** @brief History of executed steps for export/visualization. */
    std::vector<ExecutedStep> executed;

    /** @brief True once the agent has reached its goal. */
    bool at_goal = false;
};

inline TrajectoryNode TrajectoryPrediction::nodeAt(std::size_t index) const {
    if (nodes.empty()) {
        return {};
    }
    if (index >= nodes.size()) {
        return nodes.back();
    }
    return nodes[index];
}

/**
 * @brief Builds a TrajectoryPrediction from a sequence of mean waypoints.
 *
 * Variance grows linearly with each step: P_{j} = P_{j-1} + process_noise (Eq. 5).
 *
 * @param waypoints Ordered mean positions of the moving object.
 * @param initial_variance Variance at the first waypoint.
 * @param process_noise Variance increment per prediction step.
 * @return Time-indexed prediction with growing uncertainty.
 */
inline TrajectoryPrediction makePredictionFromWaypoints(
    const std::vector<Vec2>& waypoints,
    double initial_variance,
    double process_noise) {
    TrajectoryPrediction prediction;
    double variance = initial_variance;
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
        TrajectoryNode node;
        node.position = waypoints[i];
        node.variance = variance;
        node.time_step = static_cast<int>(i);
        prediction.nodes.push_back(node);
        variance += process_noise;
    }
    return prediction;
}

/**
 * @brief Builds a TrajectoryPrediction with explicit variance at each waypoint.
 */
inline TrajectoryPrediction makePredictionFromWaypointsWithVariances(
    const std::vector<Vec2>& waypoints,
    const std::vector<double>& variances) {
    TrajectoryPrediction prediction;
    const std::size_t count = std::min(waypoints.size(), variances.size());
    for (std::size_t i = 0; i < count; ++i) {
        TrajectoryNode node;
        node.position = waypoints[i];
        node.variance = variances[i];
        node.time_step = static_cast<int>(i);
        prediction.nodes.push_back(node);
    }
    return prediction;
}

}  // namespace ccrrt
