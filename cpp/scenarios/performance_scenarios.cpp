/**
 * @file performance_scenarios.cpp
 * @brief Performance benchmark environments for multi-agent CC-RRT.
 *
 * Each scenario targets a different computational bottleneck:
 * collision checks in clutter, multi-agent coordination, narrow passages,
 * long horizons, multiple dynamic obstacles, and combined stress.
 */

#include "scenarios/performance_scenarios.hpp"

#include "scenarios/scenario_helpers.hpp"

namespace ccrrt {

using scenario_helpers::emptyEnvironment;
using scenario_helpers::horizontalPath;
using scenario_helpers::makeAgent;
using scenario_helpers::makeDynamicObstacle;
using scenario_helpers::verticalPath;

Environment makePerfClutteredScenario() {
    Environment env = emptyEnvironment();
    env.static_obstacles = {
        {{4.0, 3.0}, 1.2},  {{8.0, 2.5}, 1.4},  {{12.0, 4.0}, 1.3},
        {{3.0, 7.0}, 1.5},  {{7.0, 6.5}, 1.6},  {{11.0, 7.5}, 1.4},
        {{5.0, 11.0}, 1.5}, {{9.0, 10.0}, 1.3}, {{13.0, 12.0}, 1.5},
        {{6.0, 14.0}, 1.2},
    };
    env.agents = {
        makeAgent(0, 0, "agent_a", {1.0, 1.0}, {14.0, 14.0}),
        makeAgent(1, 1, "agent_b", {14.0, 1.0}, {1.0, 14.0}),
    };
    env.dynamic_obstacles = {
        makeDynamicObstacle(0, verticalPath(10.5, 0.5, 14.5, 0.2)),
    };
    return env;
}

Environment makePerfFourAgentsScenario() {
    Environment env = emptyEnvironment();
    env.static_obstacles = {
        {{7.0, 4.0}, 2.0},
        {{3.0, 6.0}, 1.5},
        {{5.0, 10.0}, 2.0},
        {{13.0, 10.0}, 2.0},
        {{8.5, 7.5}, 1.2},
    };
    env.agents = {
        makeAgent(0, 0, "agent_0", {0.5, 0.5}, {14.5, 14.5}),
        makeAgent(1, 1, "agent_1", {14.5, 0.5}, {0.5, 14.5}),
        makeAgent(2, 2, "agent_2", {0.5, 14.5}, {14.5, 0.5}),
        makeAgent(3, 3, "agent_3", {7.5, 0.5}, {7.5, 14.5}),
    };
    env.dynamic_obstacles = {
        makeDynamicObstacle(0, horizontalPath(8.0, 1.0, 14.0, 0.2)),
    };
    return env;
}

Environment makePerfNarrowPassageScenario() {
    Environment env = emptyEnvironment();
    // Gap near y = 8, x in [6, 10] — forces detours and frequent replans.
    env.static_obstacles = {
        {{4.0, 8.0}, 2.5},
        {{6.0, 8.0}, 2.5},
        {{10.0, 8.0}, 2.5},
        {{12.0, 8.0}, 2.5},
        {{3.0, 12.0}, 1.5},
        {{11.0, 4.0}, 1.5},
    };
    env.agents = {
        makeAgent(0, 0, "west", {1.0, 8.0}, {14.0, 8.0}),
        makeAgent(1, 1, "south", {8.0, 1.0}, {8.0, 14.0}),
    };
    env.dynamic_obstacles = {
        makeDynamicObstacle(0, verticalPath(8.0, 14.0, 1.0, 0.2)),
    };
    return env;
}

Environment makePerfLongPathsScenario() {
    Environment env = emptyEnvironment();
    env.static_obstacles = {
        {{7.0, 5.0}, 1.8},
        {{9.0, 9.0}, 1.8},
        {{5.0, 11.0}, 1.5},
    };
    env.agents = {
        makeAgent(0, 0, "alpha", {0.0, 0.0}, {15.0, 15.0}),
        makeAgent(1, 1, "beta", {15.0, 0.0}, {0.0, 15.0}),
        makeAgent(2, 2, "gamma", {0.0, 15.0}, {15.0, 0.0}),
    };
    return env;
}

Environment makePerfMultiDynamicScenario() {
    Environment env = emptyEnvironment();
    env.static_obstacles = {
        {{6.0, 6.0}, 1.5},
        {{10.0, 6.0}, 1.5},
        {{8.0, 10.0}, 1.5},
    };
    env.agents = {
        makeAgent(0, 0, "north", {8.0, 1.0}, {8.0, 14.0}),
        makeAgent(1, 1, "east", {14.0, 8.0}, {1.0, 8.0}),
    };
    env.dynamic_obstacles = {
        makeDynamicObstacle(0, verticalPath(5.0, 1.0, 14.0, 0.2)),
        makeDynamicObstacle(1, verticalPath(11.0, 14.0, 1.0, 0.2)),
        makeDynamicObstacle(2, horizontalPath(8.0, 1.0, 14.0, 0.2)),
    };
    return env;
}

Environment makePerfStressScenario() {
    Environment env = makePerfClutteredScenario();
    env.agents.push_back(makeAgent(2, 2, "agent_c", {7.5, 0.5}, {7.5, 14.5}));
    env.dynamic_obstacles.push_back(
        makeDynamicObstacle(1, horizontalPath(7.0, 0.5, 14.5, 0.2)));
    env.dynamic_obstacles.push_back(
        makeDynamicObstacle(2, verticalPath(3.0, 14.0, 2.0, 0.2)));
    return env;
}

}  // namespace ccrrt
