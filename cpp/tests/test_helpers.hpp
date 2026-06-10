#pragma once

#include "ccrrt/config.hpp"
#include "ccrrt/types.hpp"

namespace ccrrt {
namespace test_helpers {

inline PlannerConfig fastTestPlannerConfig() {
    PlannerConfig config;
    config.expand_distance = 0.5;
    config.collision_bound_M = 0.2;
    config.mc_samples = 200;
    config.max_iterations = 8000;
    config.max_timesteps = 300;
    config.goal_sample_rate = 20;
    config.bounds_min = 0.0;
    config.bounds_max = 10.0;
    config.rng_seed = 42;
    config.use_legacy_collision = false;
    return config;
}

inline Environment openEnvironment() {
    Environment env;
    env.bounds_min = 0.0;
    env.bounds_max = 10.0;
    return env;
}

inline Environment singleAgentEnvironment(Vec2 start, Vec2 goal) {
    Environment env = openEnvironment();
    AgentSpec agent;
    agent.id = 0;
    agent.priority = 0;
    agent.name = "solo";
    agent.start = start;
    agent.goal = goal;
    env.agents = {agent};
    return env;
}

inline Environment twoAgentOpenEnvironment() {
    Environment env = openEnvironment();
    AgentSpec a;
    a.id = 0;
    a.priority = 0;
    a.name = "a";
    a.start = {1.0, 1.0};
    a.goal = {9.0, 9.0};
    AgentSpec b;
    b.id = 1;
    b.priority = 1;
    b.name = "b";
    b.start = {9.0, 1.0};
    b.goal = {1.0, 9.0};
    env.agents = {a, b};
    return env;
}

inline Environment blockedGoalEnvironment() {
    Environment env = openEnvironment();
    env.static_obstacles = {{{5.0, 5.0}, 4.5}};
    AgentSpec agent;
    agent.id = 0;
    agent.priority = 0;
    agent.name = "blocked";
    agent.start = {1.0, 1.0};
    agent.goal = {9.0, 9.0};
    env.agents = {agent};
    return env;
}

inline Environment staticObstacleOnlyEnvironment() {
    Environment env = openEnvironment();
    env.static_obstacles = {{{5.0, 5.0}, 1.0}};
    return env;
}

}  // namespace test_helpers
}  // namespace ccrrt
