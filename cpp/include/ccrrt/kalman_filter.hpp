#pragma once

#include "ccrrt/config.hpp"
#include "ccrrt/types.hpp"

#include <random>

namespace ccrrt {

class KalmanFilter {
public:
    explicit KalmanFilter(const PlannerConfig& config);

    GaussianState predict(const GaussianState& state) const;
    GaussianState update(const GaussianState& predicted, const Vec2& measurement) const;
    Vec2 simulateMeasurement(const Vec2& true_position, std::mt19937& rng) const;

private:
    PlannerConfig config_;
};

}  // namespace ccrrt
