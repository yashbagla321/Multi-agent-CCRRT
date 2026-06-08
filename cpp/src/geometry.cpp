/**
 * @file geometry.cpp
 * @brief 2D geometric primitives for collision checking and edge validation.
 *
 * @see ccrrt/geometry.hpp
 */

#include "ccrrt/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace ccrrt {

double pointSegmentDistance(const Vec2& point, const Vec2& seg_start, const Vec2& seg_end) {
    const Vec2 segment = seg_end - seg_start;
    const Vec2 to_point = point - seg_start;
    const double seg_len_sq = segment.x * segment.x + segment.y * segment.y;

    // Degenerate segment: treat as a point.
    if (seg_len_sq <= 1e-12) {
        return point.distance(seg_start);
    }

    // Project onto the segment and clamp to [0, 1].
    double t = (to_point.x * segment.x + to_point.y * segment.y) / seg_len_sq;
    t = std::clamp(t, 0.0, 1.0);
    const Vec2 projection = seg_start + segment * t;
    return point.distance(projection);
}

namespace {

/** @brief Signed 2D cross product (b - a) x (c - a); used for orientation tests. */
double cross(const Vec2& a, const Vec2& b, const Vec2& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

/** @brief True if point @p p lies on segment [a, b] (within epsilon tolerance). */
bool onSegment(const Vec2& a, const Vec2& b, const Vec2& p) {
    return std::min(a.x, b.x) - 1e-9 <= p.x && p.x <= std::max(a.x, b.x) + 1e-9 &&
           std::min(a.y, b.y) - 1e-9 <= p.y && p.y <= std::max(a.y, b.y) + 1e-9;
}

}  // namespace

bool segmentsIntersect(
    const Vec2& a0,
    const Vec2& a1,
    const Vec2& b0,
    const Vec2& b1) {
    // Standard segment intersection using cross-product orientation tests.
    const double d1 = cross(b0, b1, a0);
    const double d2 = cross(b0, b1, a1);
    const double d3 = cross(a0, a1, b0);
    const double d4 = cross(a0, a1, b1);

    if (((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) &&
        ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))) {
        return true;
    }

    // Collinear / endpoint overlap cases.
    if (std::abs(d1) <= 1e-9 && onSegment(b0, b1, a0)) {
        return true;
    }
    if (std::abs(d2) <= 1e-9 && onSegment(b0, b1, a1)) {
        return true;
    }
    if (std::abs(d3) <= 1e-9 && onSegment(a0, a1, b0)) {
        return true;
    }
    if (std::abs(d4) <= 1e-9 && onSegment(a0, a1, b1)) {
        return true;
    }

    return false;
}

double chiSquaredThreshold2D(double alpha) {
    // Isotropic 2D Gaussian: alpha-confidence radius = sigma * sqrt(k).
    // k is the chi-squared(2) quantile at level alpha.
    if (alpha >= 0.99) {
        return 9.21034;
    }
    if (alpha >= 0.95) {
        return 5.99146;
    }
    return 5.99146;
}

}  // namespace ccrrt
