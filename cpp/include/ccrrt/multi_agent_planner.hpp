/**
 * @file multi_agent_planner.hpp
 * @brief Multi-agent receding-horizon coordinator (paper Algorithms 1 & 2).
 */

#pragma once

#include "ccrrt/ccrrt_planner.hpp"
#include "ccrrt/collision_checker.hpp"
#include "ccrrt/config.hpp"
#include "ccrrt/kalman_filter.hpp"
#include "ccrrt/simulation_observer.hpp"
#include "ccrrt/types.hpp"

#include <memory>

namespace ccrrt {

/**
 * @brief Multi-agent receding-horizon CC-RRT coordinator (paper Algorithms 1 & 2).
 *
 * Orchestrates priority-ordered initial planning, trajectory broadcasting,
 * Kalman measurement updates, lazy collision checks, and replanning for all agents.
 */
class MultiAgentPlanner {
public:
    /**
     * @brief Constructs the coordinator and its internal planner components.
     * @param config Planner, filter, and collision-check parameters.
     */
    explicit MultiAgentPlanner(PlannerConfig config);

    /**
     * @brief Runs a full multi-agent simulation for the given environment.
     *
     * Executes Algorithm 1 (initial priority-based planning) followed by
     * Algorithm 2 (receding horizon loop) until all agents reach their goals
     * or the step limit is reached.
     *
     * @param environment Static/dynamic obstacles and agent specifications.
     * @param scenario_name Label stored in the result for export.
     * @return Executed paths, replan count, and success flag.
     */
    SimulationResult run(const Environment& environment, const std::string& scenario_name);

    /**
     * @brief Runs simulation with optional per-timestep observer (live visualization).
     * @param observer Receives frames after initial planning and each timestep; may abort.
     */
    SimulationResult run(
        const Environment& environment,
        const std::string& scenario_name,
        ISimulationObserver* observer);

private:
    SimulationFrame buildFrame(
        const Environment& environment,
        const std::string& scenario_name,
        int timestep,
        const std::vector<AgentRuntime>& agents,
        const std::vector<TrajectoryPrediction>& dynamic_predictions,
        bool initial_plan_ready,
        bool simulation_complete) const;

    bool notifyObserver(
        ISimulationObserver* observer,
        const SimulationFrame& frame) const;

    /** @brief Creates runtime agent states sorted by ascending priority. */
    std::vector<AgentRuntime> initializeAgents(const Environment& environment) const;

    /**
     * @brief Algorithm 1: plans initial trajectories in priority order.
     * @return False if any agent fails to find an initial path.
     */
    bool planInitialTrajectories(
        std::vector<AgentRuntime>& agents,
        const Environment& environment,
        std::vector<TrajectoryPrediction>& dynamic_predictions);

    /**
     * @brief Lazy feasibility check on the agent's immediate next edge (Section 4.2.1).
     * @return True if the next edge is sufficiently safe; false triggers replanning.
     */
    bool lazyCheck(
        const AgentRuntime& agent,
        const std::vector<AgentRuntime>& agents,
        const std::vector<StaticObstacle>& static_obstacles,
        const std::vector<TrajectoryPrediction>& dynamic_predictions) const;

    /** @brief Broadcast prediction anchored at the agent's current state. */
    TrajectoryPrediction makeAgentPrediction(const AgentRuntime& agent) const;

    /** @brief Drops the first node, advancing a prediction one time step. */
    TrajectoryPrediction advancePrediction(const TrajectoryPrediction& prediction) const;

    /**
     * @brief Collects broadcast predictions from all agents except @p exclude_id.
     * @param exclude_id Agent ID to omit (typically the planning agent itself).
     */
    std::vector<TrajectoryPrediction> collectAgentPredictions(
        const std::vector<AgentRuntime>& agents,
        int exclude_id) const;

    /** @brief Returns true when every agent has reached its goal. */
    bool allAgentsAtGoal(const std::vector<AgentRuntime>& agents) const;

    /** @brief Removes the first node from a trajectory (receding horizon shift). */
    void shiftTrajectory(Trajectory& trajectory);

    PlannerConfig config_;
    std::mt19937 rng_;
    KalmanFilter kalman_;
    std::unique_ptr<ICollisionChecker> collision_checker_;
    CCRRTPlanner planner_;
};

}  // namespace ccrrt
