/**
 * @file python_reference.hpp
 * @brief Scenario matching the working test case in Multiagent CCRRT.py.
 */

#pragma once

#include "ccrrt/types.hpp"

namespace ccrrt {

/**
 * @brief Recreates the geometry and obstacle layout from Multiagent CCRRT.py main().
 *
 * Two agents (red priority 0, blue priority 1), four static discs, and one
 * vertical dynamic obstacle at x = 10 with unit-spaced waypoints y = 0..12.
 */
Environment makePythonReferenceScenario();

}  // namespace ccrrt
