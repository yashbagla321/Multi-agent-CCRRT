#include "ccrrt/legacy_collision_checker.hpp"
#include "tests/test_helpers.hpp"

#include <gtest/gtest.h>

using ccrrt::GaussianState;
using ccrrt::LegacyPythonCollisionChecker;
using ccrrt::PlannerConfig;
using ccrrt::test_helpers::staticObstacleOnlyEnvironment;

TEST(LegacyPythonCollisionChecker, NodeInsideObstacleIsUnsafe) {
    PlannerConfig config = ccrrt::pythonCompatPlannerConfig();
    LegacyPythonCollisionChecker checker(config);

    GaussianState robot;
    robot.mean = {5.0, 5.0};
    robot.variance = 0.2;

    const auto env = staticObstacleOnlyEnvironment();
    EXPECT_FALSE(checker.isNodeSafe(robot, env.static_obstacles, {}, {}, 0));
}

TEST(LegacyPythonCollisionChecker, EdgeThroughObstacleRejected) {
    PlannerConfig config = ccrrt::pythonCompatPlannerConfig();
    LegacyPythonCollisionChecker checker(config);

    const auto env = staticObstacleOnlyEnvironment();
    EXPECT_FALSE(checker.isEdgeSafe(
        {1.0, 5.0}, {9.0, 5.0}, 0.2, env.static_obstacles, {}, {}, 0));
}

TEST(LegacyPythonCollisionChecker, OpenEdgeAccepted) {
    PlannerConfig config = ccrrt::pythonCompatPlannerConfig();
    LegacyPythonCollisionChecker checker(config);

    const auto env = staticObstacleOnlyEnvironment();
    EXPECT_TRUE(checker.isEdgeSafe(
        {1.0, 1.0}, {1.0, 9.0}, 0.2, env.static_obstacles, {}, {}, 0));
}
