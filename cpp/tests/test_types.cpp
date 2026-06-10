#include "ccrrt/types.hpp"

#include <gtest/gtest.h>

using ccrrt::TrajectoryPrediction;
using ccrrt::Vec2;
using ccrrt::makePredictionFromWaypoints;

TEST(Types, MakePredictionFromWaypoints_GrowsVariance) {
    const auto prediction = makePredictionFromWaypoints({{0, 0}, {1, 0}, {2, 0}}, 0.2, 0.1);
    ASSERT_EQ(prediction.nodes.size(), 3u);
    EXPECT_DOUBLE_EQ(prediction.nodes[0].variance, 0.2);
    EXPECT_DOUBLE_EQ(prediction.nodes[1].variance, 0.3);
    EXPECT_DOUBLE_EQ(prediction.nodes[2].variance, 0.4);
}

TEST(Types, TrajectoryPrediction_NodeAtClampsPastEnd) {
    TrajectoryPrediction prediction;
    prediction.nodes.push_back({{1, 1}, 0.2, 0});
    prediction.nodes.push_back({{2, 2}, 0.3, 1});

    const auto node = prediction.nodeAt(99);
    EXPECT_DOUBLE_EQ(node.position.x, 2.0);
    EXPECT_DOUBLE_EQ(node.variance, 0.3);
}

TEST(Types, Vec2Distance) {
    EXPECT_NEAR(Vec2{0, 0}.distance({3, 4}), 5.0, 1e-9);
}
