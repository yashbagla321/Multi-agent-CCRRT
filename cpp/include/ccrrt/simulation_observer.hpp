/**
 * @file simulation_observer.hpp
 * @brief Per-timestep simulation snapshots for live visualization.
 */

#pragma once

#include "ccrrt/types.hpp"

#include <string>
#include <vector>

namespace ccrrt {

/**
 * @brief Snapshot of one agent at a single simulation instant.
 */
struct AgentSnapshot {
    AgentSpec spec;
    Vec2 position;
    double variance = 0.2;
    bool at_goal = false;
    bool replanned_this_step = false;

    /** @brief Steps executed so far (includes current position history). */
    std::vector<ExecutedStep> executed;

    /** @brief Active receding-horizon plan from the current state. */
    Trajectory planned;
};

/**
 * @brief Full world state at one visualization frame.
 */
struct SimulationFrame {
    std::string scenario_name;
    int timestep = 0;

    /** @brief True after Algorithm 1 completes and before the first execution step. */
    bool initial_plan_ready = false;

    /** @brief True when simulation finished (success or step limit). */
    bool simulation_complete = false;

    Environment environment;
    std::vector<AgentSnapshot> agents;
    std::vector<TrajectoryPrediction> dynamic_predictions;
};

/**
 * @brief Receives simulation frames during MultiAgentPlanner::run().
 *
 * Return false to abort the simulation early (e.g. user closed the window).
 */
class ISimulationObserver {
public:
    virtual ~ISimulationObserver() = default;
    virtual bool onFrame(const SimulationFrame& frame) = 0;
};

}  // namespace ccrrt
