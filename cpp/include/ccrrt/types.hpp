#pragma once

#include <cmath>
#include <string>
#include <vector>

namespace ccrrt {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}

    Vec2 operator+(const Vec2& other) const { return {x + other.x, y + other.y}; }
    Vec2 operator-(const Vec2& other) const { return {x - other.x, y - other.y}; }
    Vec2 operator*(double scale) const { return {x * scale, y * scale}; }

    double norm() const { return std::sqrt(x * x + y * y); }
    double distance(const Vec2& other) const { return (*this - other).norm(); }
};

struct GaussianState {
    Vec2 mean;
    double variance = 0.2;
};

struct TrajectoryNode {
    Vec2 position;
    double variance = 0.2;
    int time_step = 0;
};

struct Trajectory {
    std::vector<TrajectoryNode> nodes;
    double total_cost = 0.0;

    bool empty() const { return nodes.empty(); }
    std::size_t size() const { return nodes.size(); }
};

struct StaticObstacle {
    Vec2 center;
    double radius = 1.0;
};

struct TrajectoryPrediction {
    std::vector<TrajectoryNode> nodes;

    bool empty() const { return nodes.empty(); }
    TrajectoryNode nodeAt(std::size_t index) const;
};

struct AgentSpec {
    int id = 0;
    int priority = 0;
    Vec2 start;
    Vec2 goal;
    std::string name;
};

struct DynamicObstacleSpec {
    int id = 0;
    std::vector<Vec2> waypoints;
    double initial_variance = 0.2;
};

struct Environment {
    double bounds_min = -2.0;
    double bounds_max = 17.0;
    std::vector<StaticObstacle> static_obstacles;
    std::vector<AgentSpec> agents;
    std::vector<DynamicObstacleSpec> dynamic_obstacles;
};

struct ExecutedStep {
    int agent_id = 0;
    int timestep = 0;
    Vec2 position;
    double variance = 0.2;
    bool replanned = false;
};

struct SimulationResult {
    std::string scenario_name;
    std::vector<std::vector<ExecutedStep>> agent_paths;
    int replan_count = 0;
    bool success = false;
};

struct AgentRuntime {
    AgentSpec spec;
    GaussianState state;
    Trajectory planned;
    std::vector<ExecutedStep> executed;
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

}  // namespace ccrrt
