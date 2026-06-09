/**
 * @file python_reference.cpp
 * @brief Environment factory aligned with Multiagent CCRRT.py main().
 */

#include "scenarios/python_reference.hpp"

#include "scenarios/scenario_helpers.hpp"

namespace ccrrt {

using scenario_helpers::makeAgent;

Environment makePythonReferenceScenario() {
    Environment env;
    env.bounds_min = -2.0;
    env.bounds_max = 15.0;

    // obstacleList in Python: (x, y, diameter)
    env.static_obstacles = {
        {{7.0, 4.0}, 2.0},
        {{3.0, 6.0}, 1.5},
        {{5.0, 10.0}, 2.0},
        {{13.0, 10.0}, 2.0},
    };

    // start1 / goal1 and start2 / goal2 from Python main().
    env.agents = {
        makeAgent(0, 0, "red", {4.0, 0.0}, {0.0, 13.0}),
        makeAgent(1, 1, "blue", {2.0, 2.0}, {12.0, 0.0}),
    };

    DynamicObstacleSpec dynamic_obstacle;
    dynamic_obstacle.id = 0;
    for (int y = 0; y <= 12; ++y) {
        dynamic_obstacle.waypoints.push_back({10.0, static_cast<double>(y)});
    }
    // obstaclePath in Python: variance 0.2 at y = 0, then 0.4 for all later steps.
    dynamic_obstacle.variance_per_waypoint.assign(13, 0.4);
    dynamic_obstacle.variance_per_waypoint.front() = 0.2;
    env.dynamic_obstacles = {dynamic_obstacle};

    return env;
}

}  // namespace ccrrt
