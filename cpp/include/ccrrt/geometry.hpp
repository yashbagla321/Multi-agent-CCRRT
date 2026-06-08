/**
 * @file geometry.hpp
 * @brief 2D geometric utilities for distance and intersection tests.
 */

#pragma once

#include "ccrrt/types.hpp"

namespace ccrrt {

/**
 * @brief Shortest Euclidean distance from a point to a line segment.
 *
 * Used for static obstacle edge checks and proximity cost evaluation.
 *
 * @param point Query point.
 * @param seg_start Start of the line segment.
 * @param seg_end End of the line segment.
 * @return Minimum distance from @p point to the closed segment [seg_start, seg_end].
 */
double pointSegmentDistance(const Vec2& point, const Vec2& seg_start, const Vec2& seg_end);

/**
 * @brief Tests whether two line segments intersect in 2D.
 *
 * Used to reject RRT edges that cross a higher-priority agent's broadcast path
 * (paper Section 4.1).
 *
 * @param a0 Start of segment A.
 * @param a1 End of segment A.
 * @param b0 Start of segment B.
 * @param b1 End of segment B.
 * @return True if segments A and B share at least one interior or endpoint intersection.
 */
bool segmentsIntersect(
    const Vec2& a0,
    const Vec2& a1,
    const Vec2& b0,
    const Vec2& b1);

/**
 * @brief Chi-squared threshold k for an isotropic 2D alpha-confidence disc.
 *
 * For isotropic Gaussian variance sigma^2, the alpha-confidence radius is
 * sigma * sqrt(k). Supports common alpha values used in the paper (0.99, 0.95).
 *
 * @param alpha Desired confidence level in (0, 1).
 * @return k such that P(||X - mu||^2 / sigma^2 <= k) >= alpha for 2D isotropic Gaussian.
 */
double chiSquaredThreshold2D(double alpha);

}  // namespace ccrrt
