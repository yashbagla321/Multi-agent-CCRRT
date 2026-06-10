#include "ccrrt/kalman_filter.hpp"
#include "ccrrt/config.hpp"

#include <gtest/gtest.h>
#include <random>

using ccrrt::GaussianState;
using ccrrt::KalmanFilter;
using ccrrt::PlannerConfig;
using ccrrt::Vec2;

TEST(KalmanFilter, PredictIncreasesVariance) {
    PlannerConfig config;
    KalmanFilter filter(config);

    GaussianState state;
    state.mean = {1.0, 2.0};
    state.variance = 0.2;

    const GaussianState predicted = filter.predict(state);
    EXPECT_DOUBLE_EQ(predicted.mean.x, 1.0);
    EXPECT_DOUBLE_EQ(predicted.mean.y, 2.0);
    EXPECT_DOUBLE_EQ(predicted.variance, 0.4);
}

TEST(KalmanFilter, UpdateReducesVariance) {
    PlannerConfig config;
    KalmanFilter filter(config);

    GaussianState predicted;
    predicted.mean = {0.0, 0.0};
    predicted.variance = 0.4;

    const GaussianState updated = filter.update(predicted, {0.1, -0.1});
    EXPECT_LT(updated.variance, predicted.variance);
    EXPECT_GT(updated.variance, 0.0);
}

TEST(KalmanFilter, SimulateMeasurementIsDeterministicWithFixedSeed) {
    PlannerConfig config;
    KalmanFilter filter(config);
    std::mt19937 rng(12345);

    const Vec2 a = filter.simulateMeasurement({5.0, 5.0}, rng);
    std::mt19937 rng2(12345);
    const Vec2 b = filter.simulateMeasurement({5.0, 5.0}, rng2);

    EXPECT_DOUBLE_EQ(a.x, b.x);
    EXPECT_DOUBLE_EQ(a.y, b.y);
}
