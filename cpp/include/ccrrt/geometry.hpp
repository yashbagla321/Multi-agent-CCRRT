#pragma once

#include "ccrrt/types.hpp"

namespace ccrrt {

double pointSegmentDistance(const Vec2& point, const Vec2& seg_start, const Vec2& seg_end);

bool segmentsIntersect(
    const Vec2& a0,
    const Vec2& a1,
    const Vec2& b0,
    const Vec2& b1);

double chiSquaredThreshold2D(double alpha);

}  // namespace ccrrt
