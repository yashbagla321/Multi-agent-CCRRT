/**
 * @file multi_agent_planner.cpp
 * @brief Multi-agent receding-horizon CC-RRT simulation (paper Algorithms 1 & 2).
 *
 * Algorithm 1: priority-ordered initial planning with trajectory broadcast.
 * Algorithm 2: per-step execution, Kalman update, lazy check, optional replan,
 *              and dynamic obstacle prediction advance.
 *
 * @see ccrrt/multi_agent_planner.hpp
 */

#include "ccrrt/multi_agent_planner.hpp"

#include <algorithm>
#include <iostream>

namespace ccrrt {

MultiAgentPlanner::MultiAgentPlanner(PlannerConfig config)
    : config_(config),
      rng_(config.rng_seed),
      kalman_(config_),
      collision_checker_(config_, rng_),
      planner_(config_, collision_checker_, rng_) {}

std::vector<AgentRuntime> MultiAgentPlanner::initializeAgents(const Environment& environment) const {
    std::vector<AgentRuntime> agents;
    agents.reserve(environment.agents.size());
    for (const auto& spec : environment.agents) {
        AgentRuntime runtime;
        runtime.spec = spec;
        runtime.state.mean = spec.start;
        runtime.state.variance = config_.initial_variance;
        runtime.at_goal = false;
        agents.push_back(runtime);
    }

    std::sort(agents.begin(), agents.end(), [](const AgentRuntime& a, const AgentRuntime& b) {
        return a.spec.priority < b.spec.priority;
    });
    return agents;
}

TrajectoryPrediction MultiAgentPlanner::makeAgentPrediction(const Trajectory& trajectory) const {
    TrajectoryPrediction prediction;
    prediction.nodes = trajectory.nodes;
    return prediction;
}

TrajectoryPrediction MultiAgentPlanner::advancePrediction(const TrajectoryPrediction& prediction) const {
    if (prediction.nodes.size() <= 1) {
        return prediction;
    }
    TrajectoryPrediction advanced;
    advanced.nodes.assign(prediction.nodes.begin() + 1, prediction.nodes.end());
    for (std::size_t i = 0; i < advanced.nodes.size(); ++i) {
        advanced.nodes[i].time_step = static_cast<int>(i);
    }
    return advanced;
}

std::vector<TrajectoryPrediction> MultiAgentPlanner::collectAgentPredictions(
    const std::vector<AgentRuntime>& agents,
    int exclude_id) const {
    std::vector<TrajectoryPrediction> predictions;
    for (const auto& agent : agents) {
        if (agent.spec.id == exclude_id || agent.planned.empty()) {
            continue;
        }
        predictions.push_back(makeAgentPrediction(agent.planned));
    }
    return predictions;
}

bool MultiAgentPlanner::allAgentsAtGoal(const std::vector<AgentRuntime>& agents) const {
    return std::all_of(agents.begin(), agents.end(), [](const AgentRuntime& agent) {
        return agent.at_goal;
    });
}

void MultiAgentPlanner::shiftTrajectory(Trajectory& trajectory) {
    if (trajectory.nodes.size() > 1) {
        trajectory.nodes.erase(trajectory.nodes.begin());
        for (std::size_t i = 0; i < trajectory.nodes.size(); ++i) {
            trajectory.nodes[i].time_step = static_cast<int>(i);
        }
    }
}

bool MultiAgentPlanner::planInitialTrajectories(
    std::vector<AgentRuntime>& agents,
    const Environment& environment,
    std::vector<TrajectoryPrediction>& dynamic_predictions) {
    // Algorithm 1: each agent plans in priority order, seeing higher-priority broadcasts.
    for (auto& agent : agents) {
        const auto agent_predictions = collectAgentPredictions(agents, agent.spec.id);
        Trajectory planned = planner_.plan(
            agent.state,
            agent.spec.goal,
            environment.static_obstacles,
            agent_predictions,
            dynamic_predictions,
            0);

        if (planned.empty()) {
            std::cerr << "Failed to find initial plan for agent " << agent.spec.name << '\n';
            return false;
        }

        agent.planned = planned;
    }
    return true;
}

bool MultiAgentPlanner::lazyCheck(
    const AgentRuntime& agent,
    const std::vector<AgentRuntime>& agents,
    const std::vector<StaticObstacle>& static_obstacles,
    const std::vector<TrajectoryPrediction>& dynamic_predictions) const {
    if (agent.planned.nodes.size() < 2) {
        return true;
    }

    const Vec2 edge_start = agent.planned.nodes[0].position;
    const Vec2 edge_end = agent.planned.nodes[1].position;
    const double variance = agent.planned.nodes[1].variance;

    const auto agent_predictions = collectAgentPredictions(agents, agent.spec.id);
    return collision_checker_.isEdgeSafe(
        edge_start,
        edge_end,
        variance,
        static_obstacles,
        agent_predictions,
        dynamic_predictions,
        0);
}

SimulationResult MultiAgentPlanner::run(const Environment& environment, const std::string& scenario_name) {
    SimulationResult result;
    result.scenario_name = scenario_name;

    config_.bounds_min = environment.bounds_min;
    config_.bounds_max = environment.bounds_max;

    std::vector<AgentRuntime> agents = initializeAgents(environment);

    // Build dynamic obstacle predictions with growing variance along mean waypoints.
    std::vector<TrajectoryPrediction> dynamic_predictions;
    dynamic_predictions.reserve(environment.dynamic_obstacles.size());
    for (const auto& obstacle : environment.dynamic_obstacles) {
        dynamic_predictions.push_back(makePredictionFromWaypoints(
            obstacle.waypoints,
            obstacle.initial_variance,
            config_.process_noise));
    }

    if (!planInitialTrajectories(agents, environment, dynamic_predictions)) {
        result.success = false;
        return result;
    }

    int timestep = 0;
    const int max_timesteps = 500;

    // Algorithm 2: receding horizon execution loop.
    while (!allAgentsAtGoal(agents) && timestep < max_timesteps) {
        for (auto& agent : agents) {
            if (agent.at_goal) {
                continue;
            }

            if (agent.planned.nodes.size() < 2) {
                agent.at_goal = agent.state.mean.distance(agent.spec.goal) <= config_.expand_distance;
                continue;
            }

            // Record executed step at current position.
            ExecutedStep step;
            step.agent_id = agent.spec.id;
            step.timestep = timestep;
            step.position = agent.state.mean;
            step.variance = agent.state.variance;
            agent.executed.push_back(step);

            // Navigate to next planned node; simulate GPS and Kalman update.
            const Vec2 next_position = agent.planned.nodes[1].position;
            const Vec2 measurement = kalman_.simulateMeasurement(next_position, rng_);
            agent.state.mean = next_position;
            agent.state = kalman_.update(kalman_.predict(agent.state), measurement);

            shiftTrajectory(agent.planned);

            // Lazy check + optional replan if unsafe or shorter path found.
            const bool lazy_safe =
                lazyCheck(agent, agents, environment.static_obstacles, dynamic_predictions);
            const auto agent_predictions = collectAgentPredictions(agents, agent.spec.id);
            Trajectory replanned = planner_.plan(
                agent.state,
                agent.spec.goal,
                environment.static_obstacles,
                agent_predictions,
                dynamic_predictions,
                0);

            const bool adopt_replan =
                !lazy_safe ||
                (!replanned.empty() && replanned.nodes.size() < agent.planned.nodes.size());

            if (adopt_replan && !replanned.empty()) {
                agent.planned = replanned;
                ++result.replan_count;
                if (!agent.executed.empty()) {
                    agent.executed.back().replanned = true;
                }
            }

            if (agent.state.mean.distance(agent.spec.goal) <= config_.expand_distance) {
                agent.at_goal = true;
            }
        }

        // Advance all dynamic obstacle predictions one timestep.
        for (auto& prediction : dynamic_predictions) {
            prediction = advancePrediction(prediction);
        }

        ++timestep;
    }

    result.success = allAgentsAtGoal(agents);
    result.agent_paths.resize(agents.size());
    for (std::size_t i = 0; i < agents.size(); ++i) {
        result.agent_paths[i] = agents[i].executed;
    }
    return result;
}

}  // namespace ccrrt
