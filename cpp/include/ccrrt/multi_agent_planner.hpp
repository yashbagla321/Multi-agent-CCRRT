#pragma once

#include "ccrrt/ccrrt_planner.hpp"
#include "ccrrt/collision_checker.hpp"
#include "ccrrt/config.hpp"
#include "ccrrt/kalman_filter.hpp"
#include "ccrrt/types.hpp"

namespace ccrrt {

class MultiAgentPlanner {
public:
    explicit MultiAgentPlanner(PlannerConfig config);

    SimulationResult run(const Environment& environment, const std::string& scenario_name);

private:
    std::vector<AgentRuntime> initializeAgents(const Environment& environment) const;
    bool planInitialTrajectories(
        std::vector<AgentRuntime>& agents,
        const Environment& environment,
        std::vector<TrajectoryPrediction>& dynamic_predictions);
    bool lazyCheck(
        const AgentRuntime& agent,
        const std::vector<AgentRuntime>& agents,
        const std::vector<StaticObstacle>& static_obstacles,
        const std::vector<TrajectoryPrediction>& dynamic_predictions) const;
    TrajectoryPrediction makeAgentPrediction(const Trajectory& trajectory) const;
    TrajectoryPrediction advancePrediction(const TrajectoryPrediction& prediction) const;
    std::vector<TrajectoryPrediction> collectAgentPredictions(
        const std::vector<AgentRuntime>& agents,
        int exclude_id) const;
    bool allAgentsAtGoal(const std::vector<AgentRuntime>& agents) const;
    void shiftTrajectory(Trajectory& trajectory);

    PlannerConfig config_;
    std::mt19937 rng_;
    KalmanFilter kalman_;
    MonteCarloCollisionChecker collision_checker_;
    CCRRTPlanner planner_;
};

}  // namespace ccrrt
