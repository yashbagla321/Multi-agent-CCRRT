/**
 * @file simulation_observer.hpp
 * @brief Per-timestep simulation snapshots for replay export and visualization.
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

    /** @brief Collision probability estimate for each node in @ref planned. */
    std::vector<double> planned_collision_probabilities;

    /** @brief Maximum collision probability across the active planned horizon. */
    double max_collision_probability = 0.0;
};

/**
 * @brief Full world state at one replay frame.
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

    /** @brief Maximum collision probability across all active agent plans. */
    double max_collision_probability = 0.0;
};

/**
 * @brief Receives simulation frames during MultiAgentPlanner::run().
 *
 * Return false to abort the simulation early.
 */
class ISimulationObserver {
public:
    virtual ~ISimulationObserver() = default;

    /** @brief Handles one simulation frame; return false to stop the run. */
    virtual bool onFrame(const SimulationFrame& frame) = 0;
};

}  // namespace ccrrt
