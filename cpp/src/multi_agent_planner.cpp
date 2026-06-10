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
#include "ccrrt/simulation_observer.hpp"

#include "ccrrt/collision_checker.hpp"
#include "ccrrt/legacy_collision_checker.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>

namespace ccrrt {

namespace {

double effectiveMotionStep(const PlannerConfig& config) {
    if (config.motion_step <= 0.0) {
        return config.expand_distance;
    }
    return std::min(config.motion_step, config.expand_distance);
}

Vec2 advanceToward(const Vec2& from, const Vec2& to, double step_size) {
    const Vec2 delta = to - from;
    const double distance = delta.norm();
    if (distance <= step_size || distance <= 1e-9) {
        return to;
    }
    return from + delta * (step_size / distance);
}

std::vector<double> variancesAlongPlannedEdge(
    const TrajectoryNode& start_node,
    const TrajectoryNode& end_node) {
    const int time_start = start_node.time_step;
    const int time_end = end_node.time_step;
    const int span = time_end - time_start;
    if (span <= 0) {
        return {end_node.variance};
    }

    std::vector<double> variances;
    variances.reserve(static_cast<std::size_t>(span + 1));
    for (int time_index = time_start; time_index <= time_end; ++time_index) {
        const double alpha = static_cast<double>(time_index - time_start) / static_cast<double>(span);
        variances.push_back(start_node.variance * (1.0 - alpha) + end_node.variance * alpha);
    }
    return variances;
}

}  // namespace

MultiAgentPlanner::MultiAgentPlanner(PlannerConfig config)
    : config_(config),
      rng_(config.rng_seed),
      kalman_(config_),
      collision_checker_(config.use_legacy_collision
                               ? std::unique_ptr<ICollisionChecker>(
                                     std::make_unique<LegacyPythonCollisionChecker>(config))
                               : std::unique_ptr<ICollisionChecker>(
                                     std::make_unique<MonteCarloCollisionChecker>(config, rng_))),
      planner_(config_, *collision_checker_, rng_) {}

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
        advanced.nodes[i].variance = config_.measurement_noise;
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
    if (trajectory.nodes.size() <= 1) {
        return;
    }
    const int time_delta = trajectory.nodes[1].time_step - trajectory.nodes[0].time_step;
    trajectory.nodes.erase(trajectory.nodes.begin());
    for (auto& node : trajectory.nodes) {
        node.time_step -= time_delta;
    }
}

bool MultiAgentPlanner::planInitialTrajectories(
    std::vector<AgentRuntime>& agents,
    const Environment& environment,
    std::vector<TrajectoryPrediction>& dynamic_predictions) {
    // Algorithm 1: each agent plans in priority order, seeing higher-priority broadcasts.
    const int default_max_iterations = config_.max_iterations;
    const int default_goal_sample_rate = config_.goal_sample_rate;

    for (std::size_t agent_index = 0; agent_index < agents.size(); ++agent_index) {
        auto& agent = agents[agent_index];

        // Lower-priority agents route around prior broadcasts and need extra search budget.
        if (agent_index > 0) {
            config_.max_iterations = default_max_iterations * 2;
            config_.goal_sample_rate = std::min(35, default_goal_sample_rate + 15);
        } else {
            config_.max_iterations = default_max_iterations;
            config_.goal_sample_rate = default_goal_sample_rate;
        }

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
            config_.max_iterations = default_max_iterations;
            config_.goal_sample_rate = default_goal_sample_rate;
            return false;
        }

        agent.planned = planned;
    }

    config_.max_iterations = default_max_iterations;
    config_.goal_sample_rate = default_goal_sample_rate;
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

    const Vec2 edge_start = agent.state.mean;
    const Vec2 edge_end = agent.planned.nodes[1].position;
    const int time_start = agent.planned.nodes[0].time_step;
    const int time_end = agent.planned.nodes[1].time_step;
    const auto variances =
        variancesAlongPlannedEdge(agent.planned.nodes[0], agent.planned.nodes[1]);
    std::vector<double> edge_variances = variances;
    if (!edge_variances.empty()) {
        edge_variances.front() = agent.state.variance;
    }

    const auto agent_predictions = collectAgentPredictions(agents, agent.spec.id);
    return isSpanEdgeSafe(
        *collision_checker_,
        edge_start,
        edge_end,
        time_start,
        time_end,
        edge_variances,
        static_obstacles,
        agent_predictions,
        dynamic_predictions);
}

SimulationResult MultiAgentPlanner::run(
    const Environment& environment,
    const std::string& scenario_name) {
    return run(environment, scenario_name, nullptr);
}

SimulationFrame MultiAgentPlanner::buildFrame(
    const Environment& environment,
    const std::string& scenario_name,
    int timestep,
    const std::vector<AgentRuntime>& agents,
    const std::vector<TrajectoryPrediction>& dynamic_predictions,
    bool initial_plan_ready,
    bool simulation_complete) const {
    SimulationFrame frame;
    frame.scenario_name = scenario_name;
    frame.timestep = timestep;
    frame.initial_plan_ready = initial_plan_ready;
    frame.simulation_complete = simulation_complete;
    frame.environment = environment;
    frame.dynamic_predictions = dynamic_predictions;
    frame.agents.reserve(agents.size());

    for (const auto& agent : agents) {
        AgentSnapshot snapshot;
        snapshot.spec = agent.spec;
        snapshot.position = agent.state.mean;
        snapshot.variance = agent.state.variance;
        snapshot.at_goal = agent.at_goal;
        snapshot.executed = agent.executed;
        snapshot.planned = agent.planned;
        if (!agent.executed.empty()) {
            snapshot.replanned_this_step = agent.executed.back().replanned;
        }
        frame.agents.push_back(std::move(snapshot));
    }
    return frame;
}

bool MultiAgentPlanner::notifyObserver(
    ISimulationObserver* observer,
    const SimulationFrame& frame) const {
    if (observer == nullptr) {
        return true;
    }
    return observer->onFrame(frame);
}

SimulationResult MultiAgentPlanner::run(
    const Environment& environment,
    const std::string& scenario_name,
    ISimulationObserver* observer) {
    const auto clock_start = std::chrono::steady_clock::now();

    SimulationResult result;
    result.scenario_name = scenario_name;

    config_.bounds_min = environment.bounds_min;
    config_.bounds_max = environment.bounds_max;

    if (environment.agents.empty()) {
        std::cerr << "Scenario has no agents.\n";
        return result;
    }

    std::vector<AgentRuntime> agents = initializeAgents(environment);

    // Dynamic obstacles: constant per-step variance (measurement noise on nominal path).
    std::vector<TrajectoryPrediction> dynamic_predictions;
    dynamic_predictions.reserve(environment.dynamic_obstacles.size());
    for (const auto& obstacle : environment.dynamic_obstacles) {
        dynamic_predictions.push_back(makePredictionFromWaypoints(
            obstacle.waypoints,
            config_.measurement_noise));
    }

    if (!planInitialTrajectories(agents, environment, dynamic_predictions)) {
        result.success = false;
        return result;
    }

    if (!notifyObserver(
            observer,
            buildFrame(
                environment,
                scenario_name,
                0,
                agents,
                dynamic_predictions,
                true,
                false))) {
        result.success = false;
        return result;
    }

    int timestep = 0;

    // Algorithm 2: receding horizon execution loop.
    while (!allAgentsAtGoal(agents) && timestep < config_.max_timesteps) {
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

            const Vec2 target = agent.planned.nodes[1].position;
            const double step_size = effectiveMotionStep(config_);
            const Vec2 next_position = advanceToward(agent.state.mean, target, step_size);
            const bool reached_waypoint =
                next_position.distance(target) <= 1e-9 ||
                agent.state.mean.distance(target) <= step_size;

            const Vec2 measurement = kalman_.simulateMeasurement(next_position, rng_);
            agent.state = kalman_.measurementUpdate(agent.state, measurement);
            agent.state.mean = next_position;

            if (reached_waypoint) {
                shiftTrajectory(agent.planned);
            }

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

        if (!notifyObserver(
                observer,
                buildFrame(
                    environment,
                    scenario_name,
                    timestep,
                    agents,
                    dynamic_predictions,
                    false,
                    allAgentsAtGoal(agents)))) {
            break;
        }
    }

    result.success = allAgentsAtGoal(agents);
    result.agent_paths.resize(agents.size());
    for (std::size_t i = 0; i < agents.size(); ++i) {
        result.agent_paths[i] = agents[i].executed;
        result.total_steps += static_cast<int>(agents[i].executed.size());
        for (const auto& step : agents[i].executed) {
            result.max_timestep = std::max(result.max_timestep, step.timestep);
        }
    }

    const auto clock_end = std::chrono::steady_clock::now();
    result.elapsed_ms = std::chrono::duration<double, std::milli>(clock_end - clock_start).count();

    notifyObserver(
        observer,
        buildFrame(
            environment,
            scenario_name,
            timestep,
            agents,
            dynamic_predictions,
            false,
            true));

    return result;
}

}  // namespace ccrrt
