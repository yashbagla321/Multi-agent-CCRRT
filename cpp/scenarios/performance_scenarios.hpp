/**
 * @file performance_scenarios.hpp
 * @brief Benchmark scenarios that stress planner scaling and collision checking.
 */

#pragma once

#include "ccrrt/types.hpp"

namespace ccrrt {

/** @brief Dense static obstacle field for Monte Carlo collision-check load. */
Environment makePerfClutteredScenario();

/** @brief Four-agent priority-chain scenario for coordination scaling. */
Environment makePerfFourAgentsScenario();

/** @brief Narrow corridor scenario that stresses replanning around tight gaps. */
Environment makePerfNarrowPassageScenario();

/** @brief Long-horizon three-agent scenario with corner-to-corner routes. */
Environment makePerfLongPathsScenario();

/** @brief Scenario with multiple dynamic obstacles and two planning agents. */
Environment makePerfMultiDynamicScenario();

/** @brief Combined clutter, multi-agent, and multi-dynamic stress scenario. */
Environment makePerfStressScenario();

}  // namespace ccrrt
