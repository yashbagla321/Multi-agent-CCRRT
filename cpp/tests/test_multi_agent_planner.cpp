#include "ccrrt/multi_agent_planner.hpp"
#include "tests/test_helpers.hpp"

#include <gtest/gtest.h>

using ccrrt::MultiAgentPlanner;
using ccrrt::test_helpers::fastTestPlannerConfig;
using ccrrt::test_helpers::openEnvironment;
using ccrrt::test_helpers::singleAgentEnvironment;
using ccrrt::test_helpers::twoAgentOpenEnvironment;

TEST(MultiAgentPlanner, SingleAgentReachesGoal) {
    auto config = fastTestPlannerConfig();
    MultiAgentPlanner planner(config);

    const auto env = singleAgentEnvironment({1, 1}, {9, 9});
    const auto result = planner.run(env, "single_agent_open");

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.total_steps, 0);
    EXPECT_GE(result.elapsed_ms, 0.0);
    ASSERT_EQ(result.agent_paths.size(), 1u);
    EXPECT_FALSE(result.agent_paths[0].empty());
}

TEST(MultiAgentPlanner, TwoAgentsBothReachGoals) {
    auto config = fastTestPlannerConfig();
    MultiAgentPlanner planner(config);

    const auto result = planner.run(twoAgentOpenEnvironment(), "two_agent_open");

    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.agent_paths.size(), 2u);
}

TEST(MultiAgentPlanner, RecordsReplansAndTimesteps) {
    auto config = fastTestPlannerConfig();
    MultiAgentPlanner planner(config);

    const auto result = planner.run(twoAgentOpenEnvironment(), "metrics");

    EXPECT_GE(result.max_timestep, 0);
    EXPECT_GE(result.replan_count, 0);
}

TEST(MultiAgentPlanner, LegacyModeRunsWithoutCrash) {
    auto config = fastTestPlannerConfig();
    config.use_legacy_collision = true;
    config.expand_distance = 1.0;
    config.max_iterations = 500;
    config.bounds_max = 15.0;
    MultiAgentPlanner planner(config);

    const auto env = singleAgentEnvironment({4, 0}, {0, 13});
    env.bounds_min = -2.0;
    env.bounds_max = 15.0;
    const auto result = planner.run(env, "legacy_smoke");

    EXPECT_GE(result.elapsed_ms, 0.0);
}
