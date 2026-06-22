/**
 * @file paper_figures.cpp
 * @brief Implementation of paper Section 5 simulation scenarios.
 *
 * Coordinates are hand-tuned approximations of Figures 5–7 in a 17×17 workspace.
 * Static obstacles are circular; dynamic obstacles follow vertical mean paths.
 */

#include "scenarios/paper_figures.hpp"

#include "scenarios/scenario_helpers.hpp"

namespace ccrrt {

using scenario_helpers::makeAgent;

namespace {

/**
 * @brief Shared static layout used by all three figure scenarios.
 * @return Environment with bounds and four circular static obstacles only.
 */
Environment baseEnvironment() {
    Environment env;
    env.bounds_min = -2.0;
    env.bounds_max = 17.0;
    env.static_obstacles = {
        {{7.0, 4.0}, 2.0},
        {{3.0, 6.0}, 1.5},
        {{5.0, 10.0}, 2.0},
        {{13.0, 10.0}, 2.0},
    };
    return env;
}

/**
 * @brief Generates equally spaced waypoints along a vertical line.
 *
 * Used to define the predictable mean trajectory of a dynamic obstacle.
 *
 * @param x Fixed x-coordinate of the vertical path.
 * @param y_start Starting y (inclusive).
 * @param y_end Ending y (inclusive).
 * @param step Spacing between consecutive waypoints.
 * @return Ordered waypoints from y_start toward y_end.
 */
std::vector<Vec2> verticalDynamicObstacle(double x, double y_start, double y_end, double step) {
    std::vector<Vec2> waypoints;
    if (y_end >= y_start) {
        for (double y = y_start; y <= y_end + 1e-9; y += step) {
            waypoints.push_back({x, y});
        }
    } else {
        for (double y = y_start; y >= y_end - 1e-9; y -= step) {
            waypoints.push_back({x, y});
        }
    }
    return waypoints;
}

}  // namespace

Environment makeFigure5Scenario() {
    Environment env = baseEnvironment();

    env.agents = {
        makeAgent(0, 0, "red", {4.0, 0.0}, {0.0, 13.0}),
        makeAgent(1, 1, "blue", {2.0, 2.0}, {12.0, 0.0}),
    };

    DynamicObstacleSpec dynamic_obstacle;
    dynamic_obstacle.id = 0;
    // Finer steps so tree-depth time aligns with the obstacle's vertical motion.
    dynamic_obstacle.waypoints = verticalDynamicObstacle(10.0, 12.0, 0.0, 0.2);
    dynamic_obstacle.initial_variance = 0.1;
    env.dynamic_obstacles = {dynamic_obstacle};

    return env;
}

Environment makeFigure6Scenario() {
    Environment env = baseEnvironment();
    env.agents = {
        makeAgent(0, 0, "red", {1.0, 14.0}, {14.0, 14.0}),
        makeAgent(1, 1, "blue", {14.0, 1.0}, {1.0, 1.0}),
    };
    // Fig. 6 tests priority-based agent crossing only (no dynamic obstacle).
    return env;
}

Environment makeFigure7Scenario() {
    Environment env = baseEnvironment();
    env.static_obstacles.push_back({{8.0, 7.0}, 1.5});
    env.agents = {
        makeAgent(0, 0, "red", {1.0, 1.0}, {15.0, 15.0}),
        makeAgent(1, 1, "blue", {15.0, 2.0}, {2.0, 15.0}),
    };

    DynamicObstacleSpec dynamic_obstacle;
    dynamic_obstacle.id = 0;
    // Dynamic obstacle moves downward, forcing blue to wait before crossing.
    dynamic_obstacle.waypoints = verticalDynamicObstacle(9.0, 15.0, 1.0, 0.2);
    dynamic_obstacle.initial_variance = 0.1;
    env.dynamic_obstacles = {dynamic_obstacle};
    return env;
}

}  // namespace ccrrt
