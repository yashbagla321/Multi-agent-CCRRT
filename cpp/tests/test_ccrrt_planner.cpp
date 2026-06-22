#include "ccrrt/ccrrt_planner.hpp"
#include "ccrrt/collision_checker.hpp"
#include "tests/test_helpers.hpp"

#include <gtest/gtest.h>
#include <random>

using ccrrt::CCRRTPlanner;
using ccrrt::GaussianState;
using ccrrt::MonteCarloCollisionChecker;
using ccrrt::Vec2;
using ccrrt::test_helpers::fastTestPlannerConfig;
using ccrrt::test_helpers::openEnvironment;
using ccrrt::test_helpers::singleAgentEnvironment;

namespace {

class RejectGoalEdgeChecker final : public ccrrt::ICollisionChecker {
public:
    explicit RejectGoalEdgeChecker(Vec2 goal) : goal_(goal) {}

    bool isNodeSafe(
        const GaussianState&,
        const std::vector<ccrrt::StaticObstacle>&,
        const std::vector<ccrrt::TrajectoryPrediction>&,
        const std::vector<ccrrt::TrajectoryPrediction>&,
        int) const override {
        return true;
    }

    bool isEdgeSafe(
        const Vec2&,
        const Vec2& edge_end,
        double,
        const std::vector<ccrrt::StaticObstacle>&,
        const std::vector<ccrrt::TrajectoryPrediction>&,
        const std::vector<ccrrt::TrajectoryPrediction>&,
        int) const override {
        return edge_end.distance(goal_) > 1e-9;
    }

    double estimateCollisionProbability(
        const GaussianState&,
        const std::vector<ccrrt::StaticObstacle>&,
        const std::vector<ccrrt::TrajectoryPrediction>&,
        const std::vector<ccrrt::TrajectoryPrediction>&,
        int) const override {
        return 0.0;
    }

private:
    Vec2 goal_;
};

class AlwaysSafeChecker final : public ccrrt::ICollisionChecker {
public:
    bool isNodeSafe(
        const GaussianState&,
        const std::vector<ccrrt::StaticObstacle>&,
        const std::vector<ccrrt::TrajectoryPrediction>&,
        const std::vector<ccrrt::TrajectoryPrediction>&,
        int) const override {
        return true;
    }

    bool isEdgeSafe(
        const Vec2&,
        const Vec2&,
        double,
        const std::vector<ccrrt::StaticObstacle>&,
        const std::vector<ccrrt::TrajectoryPrediction>&,
        const std::vector<ccrrt::TrajectoryPrediction>&,
        int) const override {
        return true;
    }

    double estimateCollisionProbability(
        const GaussianState&,
        const std::vector<ccrrt::StaticObstacle>&,
        const std::vector<ccrrt::TrajectoryPrediction>&,
        const std::vector<ccrrt::TrajectoryPrediction>&,
        int) const override {
        return 0.0;
    }
};

}  // namespace

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

TEST(CCRRTPlanner, RejectsUnsafeFinalGoalEdge) {
    auto config = fastTestPlannerConfig();
    config.expand_distance = 0.5;
    config.goal_sample_rate = 100;
    config.max_iterations = 20;
    std::mt19937 rng(config.rng_seed);
    const Vec2 goal{1.0, 0.0};
    RejectGoalEdgeChecker checker(goal);
    CCRRTPlanner planner(config, checker, rng);

    GaussianState start;
    start.mean = {0.0, 0.0};
    start.variance = config.initial_variance;

    const auto trajectory = planner.plan(start, goal, {}, {}, {}, 0);

    EXPECT_TRUE(trajectory.empty());
}

TEST(CCRRTPlanner, PathSmoothingFlagPreservesDiscreteHorizon) {
    auto unsmoothed_config = fastTestPlannerConfig();
    unsmoothed_config.expand_distance = 0.5;
    unsmoothed_config.goal_sample_rate = 100;
    unsmoothed_config.max_iterations = 20;
    unsmoothed_config.enable_path_smoothing = false;

    auto smoothed_config = unsmoothed_config;
    smoothed_config.enable_path_smoothing = true;

    const Vec2 goal{3.0, 0.0};
    GaussianState start;
    start.mean = {0.0, 0.0};
    start.variance = unsmoothed_config.initial_variance;

    AlwaysSafeChecker unsmoothed_checker;
    std::mt19937 unsmoothed_rng(unsmoothed_config.rng_seed);
    CCRRTPlanner unsmoothed_planner(unsmoothed_config, unsmoothed_checker, unsmoothed_rng);
    const auto unsmoothed = unsmoothed_planner.plan(start, goal, {}, {}, {}, 0);

    AlwaysSafeChecker smoothed_checker;
    std::mt19937 smoothed_rng(smoothed_config.rng_seed);
    CCRRTPlanner smoothed_planner(smoothed_config, smoothed_checker, smoothed_rng);
    const auto smoothed = smoothed_planner.plan(start, goal, {}, {}, {}, 0);

    ASSERT_FALSE(unsmoothed.empty());
    ASSERT_FALSE(smoothed.empty());
    EXPECT_GT(unsmoothed.nodes.size(), 2u);
    EXPECT_EQ(smoothed.nodes.size(), unsmoothed.nodes.size());
    EXPECT_DOUBLE_EQ(smoothed.total_cost, unsmoothed.total_cost);
    for (std::size_t i = 0; i < smoothed.nodes.size(); ++i) {
        EXPECT_EQ(smoothed.nodes[i].time_step, static_cast<int>(i));
    }
    EXPECT_NEAR(smoothed.nodes.back().position.x, goal.x, 1e-9);
    EXPECT_NEAR(smoothed.nodes.back().position.y, goal.y, 1e-9);
}
