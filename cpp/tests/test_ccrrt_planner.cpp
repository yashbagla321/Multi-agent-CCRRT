#include "ccrrt/ccrrt_planner.hpp"
#include "ccrrt/collision_checker.hpp"
#include "tests/test_helpers.hpp"

#include <gtest/gtest.h>
#include <random>

using ccrrt::CCRRTPlanner;
using ccrrt::GaussianState;
using ccrrt::MonteCarloCollisionChecker;
using ccrrt::test_helpers::fastTestPlannerConfig;
using ccrrt::test_helpers::openEnvironment;
using ccrrt::test_helpers::singleAgentEnvironment;

TEST(CCRRTPlanner, PlansPathInOpenEnvironment) {
    auto config = fastTestPlannerConfig();
    std::mt19937 rng(config.rng_seed);
    MonteCarloCollisionChecker checker(config, rng);
    CCRRTPlanner planner(config, checker, rng);

    GaussianState start;
    start.mean = {1.0, 1.0};
    start.variance = config.initial_variance;

    const auto env = singleAgentEnvironment({1, 1}, {9, 9});
    const auto trajectory = planner.plan(start, {9.0, 9.0}, env.static_obstacles, {}, {}, 0);

    ASSERT_FALSE(trajectory.empty());
    EXPECT_GE(trajectory.nodes.size(), 2u);
    EXPECT_NEAR(trajectory.nodes.back().position.x, 9.0, 1e-6);
    EXPECT_NEAR(trajectory.nodes.back().position.y, 9.0, 1e-6);
}

TEST(CCRRTPlanner, FailsWhenGoalFullyBlocked) {
    auto config = fastTestPlannerConfig();
    config.max_iterations = 500;
    std::mt19937 rng(config.rng_seed);
    MonteCarloCollisionChecker checker(config, rng);
    CCRRTPlanner planner(config, checker, rng);

    GaussianState start;
    start.mean = {1.0, 1.0};
    start.variance = config.initial_variance;

    const auto env = ccrrt::test_helpers::blockedGoalEnvironment();
    const auto trajectory =
        planner.plan(start, env.agents[0].goal, env.static_obstacles, {}, {}, 0);

    EXPECT_TRUE(trajectory.empty());
}
