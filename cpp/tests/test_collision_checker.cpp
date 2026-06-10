#include "ccrrt/collision_checker.hpp"
#include "tests/test_helpers.hpp"

#include <gtest/gtest.h>
#include <random>

using ccrrt::GaussianState;
using ccrrt::MonteCarloCollisionChecker;
using ccrrt::StaticObstacle;
using ccrrt::Vec2;
using ccrrt::test_helpers::fastTestPlannerConfig;
using ccrrt::test_helpers::staticObstacleOnlyEnvironment;

TEST(MonteCarloCollisionChecker, HighCollisionInsideStaticObstacle) {
    auto config = fastTestPlannerConfig();
    config.mc_samples = 500;
    std::mt19937 rng(config.rng_seed);
    MonteCarloCollisionChecker checker(config, rng);

    GaussianState robot;
    robot.mean = {5.0, 5.0};
    robot.variance = 0.01;

    const std::vector<StaticObstacle> obstacles = {{{5.0, 5.0}, 1.0}};
    const double probability = checker.estimateCollisionProbability(robot, obstacles, {}, {}, 0);

    EXPECT_GT(probability, 0.5);
    EXPECT_FALSE(checker.isNodeSafe(robot, obstacles, {}, {}, 0));
}

TEST(MonteCarloCollisionChecker, LowCollisionInOpenSpace) {
    auto config = fastTestPlannerConfig();
    config.mc_samples = 500;
    std::mt19937 rng(config.rng_seed);
    MonteCarloCollisionChecker checker(config, rng);

    GaussianState robot;
    robot.mean = {1.0, 1.0};
    robot.variance = 0.05;

    const double probability =
        checker.estimateCollisionProbability(robot, {}, {}, {}, 0);

    EXPECT_LT(probability, config.collision_bound_M);
    EXPECT_TRUE(checker.isNodeSafe(robot, {}, {}, {}, 0));
}

TEST(MonteCarloCollisionChecker, EdgeThroughStaticObstacleRejected) {
    auto config = fastTestPlannerConfig();
    std::mt19937 rng(config.rng_seed);
    MonteCarloCollisionChecker checker(config, rng);

    const auto env = staticObstacleOnlyEnvironment();
    const bool safe = checker.isEdgeSafe(
        {1.0, 5.0},
        {9.0, 5.0},
        0.2,
        env.static_obstacles,
        {},
        {},
        0);

    EXPECT_FALSE(safe);
}

TEST(MonteCarloCollisionChecker, EdgeAroundObstacleAccepted) {
    auto config = fastTestPlannerConfig();
    std::mt19937 rng(config.rng_seed);
    MonteCarloCollisionChecker checker(config, rng);

    const auto env = staticObstacleOnlyEnvironment();
    const bool safe = checker.isEdgeSafe(
        {1.0, 1.0},
        {1.0, 9.0},
        0.2,
        env.static_obstacles,
        {},
        {},
        0);

    EXPECT_TRUE(safe);
}

TEST(MonteCarloCollisionChecker, SpanEdgeRejectsObstacleCrossingMidHorizon) {
    auto config = fastTestPlannerConfig();
    std::mt19937 rng(config.rng_seed);
    MonteCarloCollisionChecker checker(config, rng);

    const auto env = staticObstacleOnlyEnvironment();
    const std::vector<double> variances = {0.2, 0.2, 0.2, 0.2};

    const bool unsafe_span = isSpanEdgeSafe(
        checker,
        {1.0, 5.0},
        {9.0, 5.0},
        0,
        3,
        variances,
        env.static_obstacles,
        {},
        {});

    EXPECT_FALSE(unsafe_span);

    const bool unsafe_single_step = checker.isEdgeSafe(
        {1.0, 5.0},
        {9.0, 5.0},
        0.2,
        env.static_obstacles,
        {},
        {},
        0);

    EXPECT_FALSE(unsafe_single_step);
}
