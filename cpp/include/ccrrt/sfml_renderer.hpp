/**
 * @file sfml_renderer.hpp
 * @brief Optional SFML visualization (requires CCRRT_HAS_SFML).
 */

#pragma once

#include "ccrrt/types.hpp"

namespace ccrrt {

#if CCRRT_HAS_SFML

/**
 * @brief Optional SFML-based 2D visualization for environments and simulation results.
 *
 * Available only when the project is built with CCRRT_ENABLE_VISUALIZATION=ON
 * and SFML is found. Opens an interactive window; close it to continue/exit.
 */
class SFMLRenderer {
public:
    /**
     * @brief Displays a scenario layout without running the planner.
     *
     * Draws static obstacles, agent start/goal pairs (with intended path hints),
     * dynamic obstacle mean trajectories, and initial uncertainty discs.
     * Close the window to exit.
     *
     * @param environment Environment from paper_figures (or custom).
     * @param scenario_name Optional title suffix for the window (e.g. "figure5").
     */
    void renderScenarioPreview(const Environment& environment, const std::string& scenario_name = "") const;

    /**
     * @brief Alias for renderScenarioPreview (legacy name).
     * @param environment Environment to draw.
     */
    void renderEnvironment(const Environment& environment) const;

    /**
     * @brief Displays executed agent paths and confidence discs after a simulation.
     *
     * Draws static obstacles, dynamic obstacle waypoints, agent trajectories,
     * and variance-scaled confidence regions.
     *
     * @param environment Original environment (obstacles and goals).
     * @param result Simulation output containing executed steps per agent.
     */
    void renderSimulationResult(const Environment& environment, const SimulationResult& result) const;
};

#endif

}  // namespace ccrrt
