/**
 * @file scenarios.hpp
 * @brief Unified registry of paper figures and performance benchmark scenarios.
 */

#pragma once

#include "ccrrt/types.hpp"

#include <string>
#include <vector>

namespace ccrrt {

/**
 * @brief Category tag for grouping scenarios in listings and benchmarks.
 */
enum class ScenarioCategory {
    Paper,
    Performance,
};

/**
 * @brief Named scenario with optional description for benchmarks.
 */
struct ScenarioEntry {
    std::string name;
    std::string description;
    ScenarioCategory category = ScenarioCategory::Paper;
    Environment environment;
};

/** @brief Paper Section 5 scenarios (Figures 5–7). */
std::vector<ScenarioEntry> paperScenarios();

/** @brief Stress / scaling scenarios for timing and success-rate comparison. */
std::vector<ScenarioEntry> performanceScenarios();

/** @brief All scenarios (paper + performance). */
std::vector<ScenarioEntry> allScenarios();

/** @brief Performance scenarios only (for --benchmark-all). */
std::vector<ScenarioEntry> benchmarkScenarios();

}  // namespace ccrrt
