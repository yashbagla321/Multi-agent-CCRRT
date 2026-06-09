/**
 * @file performance_scenarios.hpp
 * @brief Benchmark scenarios that stress planner scaling and collision checking.
 */

#pragma once

#include "ccrrt/types.hpp"

namespace ccrrt {

Environment makePerfClutteredScenario();
Environment makePerfFourAgentsScenario();
Environment makePerfNarrowPassageScenario();
Environment makePerfLongPathsScenario();
Environment makePerfMultiDynamicScenario();
Environment makePerfStressScenario();

}  // namespace ccrrt
