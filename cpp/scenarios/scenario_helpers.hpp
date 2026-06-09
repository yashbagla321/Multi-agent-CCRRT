/**
 * @file scenario_helpers.hpp
 * @brief Shared utilities for building test and performance environments.
 */

#pragma once

#include "ccrrt/types.hpp"

#include <cmath>
#include <vector>

namespace ccrrt {
namespace scenario_helpers {

inline Environment emptyEnvironment(double bounds_min = -2.0, double bounds_max = 17.0) {
    Environment env;
    env.bounds_min = bounds_min;
    env.bounds_max = bounds_max;
    return env;
}

inline AgentSpec makeAgent(int id, int priority, const std::string& name, Vec2 start, Vec2 goal) {
    AgentSpec agent;
    agent.id = id;
    agent.priority = priority;
    agent.name = name;
    agent.start = start;
    agent.goal = goal;
    return agent;
}

inline std::vector<Vec2> lineWaypoints(Vec2 from, Vec2 to, double step) {
    std::vector<Vec2> waypoints;
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double length = std::sqrt(dx * dx + dy * dy);
    if (length <= 1e-9) {
        waypoints.push_back(from);
        return waypoints;
    }
    const int count = static_cast<int>(std::ceil(length / step));
    for (int i = 0; i <= count; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(count);
        waypoints.push_back({from.x + t * dx, from.y + t * dy});
    }
    return waypoints;
}

inline std::vector<Vec2> verticalPath(double x, double y_start, double y_end, double step) {
    return lineWaypoints({x, y_start}, {x, y_end}, step);
}

inline std::vector<Vec2> horizontalPath(double y, double x_start, double x_end, double step) {
    return lineWaypoints({x_start, y}, {x_end, y}, step);
}

inline DynamicObstacleSpec makeDynamicObstacle(int id, const std::vector<Vec2>& waypoints) {
    DynamicObstacleSpec obstacle;
    obstacle.id = id;
    obstacle.waypoints = waypoints;
    obstacle.initial_variance = 0.2;
    return obstacle;
}

}  // namespace scenario_helpers
}  // namespace ccrrt
