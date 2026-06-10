#include "ccrrt/geometry.hpp"

#include <gtest/gtest.h>

using ccrrt::Vec2;
using ccrrt::chiSquaredThreshold2D;
using ccrrt::pointSegmentDistance;
using ccrrt::segmentsIntersect;

TEST(Geometry, PointSegmentDistance_OnSegment) {
    EXPECT_NEAR(pointSegmentDistance({2, 0}, {0, 0}, {4, 0}), 0.0, 1e-9);
}

TEST(Geometry, PointSegmentDistance_Perpendicular) {
    EXPECT_NEAR(pointSegmentDistance({2, 3}, {0, 0}, {4, 0}), 3.0, 1e-9);
}

TEST(Geometry, PointSegmentDistance_DegenerateSegment) {
    EXPECT_NEAR(pointSegmentDistance({3, 4}, {1, 1}, {1, 1}), 5.0, 1e-9);
}

TEST(Geometry, SegmentsIntersect_Crossing) {
    EXPECT_TRUE(segmentsIntersect({0, 0}, {4, 4}, {0, 4}, {4, 0}));
}

TEST(Geometry, SegmentsIntersect_Parallel) {
    EXPECT_FALSE(segmentsIntersect({0, 0}, {4, 0}, {0, 2}, {4, 2}));
}

TEST(Geometry, SegmentsIntersect_TouchingEndpoint) {
    EXPECT_TRUE(segmentsIntersect({0, 0}, {2, 0}, {2, 0}, {2, 2}));
}

TEST(Geometry, ChiSquaredThreshold2D_KnownValues) {
    EXPECT_NEAR(chiSquaredThreshold2D(0.99), 9.21034, 1e-4);
    EXPECT_NEAR(chiSquaredThreshold2D(0.95), 5.99146, 1e-4);
}
