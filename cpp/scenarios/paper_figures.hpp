/**
 * @file paper_figures.hpp
 * @brief Scenario definitions matching paper Section 5 (Figures 5–7).
 *
 * Each factory builds an Environment with static obstacles, agent start/goal/priority,
 * and dynamic obstacles whose mean paths are predictable but carry growing uncertainty
 * once converted to TrajectoryPrediction inside MultiAgentPlanner.
 */

#pragma once

#include "ccrrt/types.hpp"

#include <string>
#include <vector>

namespace ccrrt {

/**
 * @brief Named scenario paired with its environment configuration.
 */
struct ScenarioEntry {
    /** @brief CLI identifier (e.g. "figure5"). */
    std::string name;

    /** @brief Full environment passed to MultiAgentPlanner::run(). */
    Environment environment;
};

/**
 * @brief Returns all built-in paper scenarios for CLI selection.
 * @return Vector of {name, environment} entries for figure5, figure6, figure7.
 *
 * Preview without simulation:
 * @code
 * multi_agent_ccrrt --scenario figure5 --preview
 * multi_agent_ccrrt --preview-all
 * @endcode
 */
std::vector<ScenarioEntry> allScenarios();

/**
 * @brief Figure 5: 4 static obstacles, 2 robots, 1 vertical dynamic obstacle.
 *
 * Red robot (priority 0) plans first; blue robot (priority 1) routes around red
 * and the dynamic obstacle.
 */
Environment makeFigure5Scenario();

/**
 * @brief Figure 6: priority-assignment / time-separated crossing scenario.
 *
 * Green robot has higher priority; agents may cross paths at different times.
 */
Environment makeFigure6Scenario();

/**
 * @brief Figure 7: 5 static obstacles; blue robot waits for dynamic obstacle.
 *
 * Blue robot (lower priority) exhibits waiting/zig-zag behavior while the dynamic
 * obstacle passes through a shared corridor.
 */
Environment makeFigure7Scenario();

}  // namespace ccrrt
