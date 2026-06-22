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
#include <cstdint>
#include <iostream>
#include <memory>

namespace ccrrt {

namespace {

/** @brief Monte Carlo sample cap used only for replay risk estimates. */
constexpr int kVisualizationRiskSamples = 100;

/** @brief Returns the physical motion step used for one execution tick. */
double effectiveMotionStep(const PlannerConfig& config) {
    if (config.motion_step <= 0.0) {
        return config.expand_distance;
    }
    return std::min(config.motion_step, config.expand_distance);
}

/** @brief Moves from @p from toward @p to by at most @p step_size. */
Vec2 advanceToward(const Vec2& from, const Vec2& to, double step_size) {
    const Vec2 delta = to - from;
    const double distance = delta.norm();
    if (distance <= step_size || distance <= 1e-9) {
        return to;
    }
    return from + delta * (step_size / distance);
}

/** @brief Interpolates variance at every integer timestep along a planned edge. */
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

/** @brief Creates a lower-cost config for replay-only collision probability estimates. */
PlannerConfig visualizationRiskConfig(const PlannerConfig& config) {
    PlannerConfig risk_config = config;
    risk_config.mc_samples = std::min(config.mc_samples, kVisualizationRiskSamples);
    return risk_config;
}

/** @brief Mixes frame/agent/node ids into a deterministic seed for replay risk sampling. */
std::uint32_t mixSeed(std::uint32_t seed, int timestep, int agent_id, int node_time_step) {
    std::uint32_t value = seed + 0x9e3779b9u;
    value ^= static_cast<std::uint32_t>(timestep + 0x7f4a7c15) + (value << 6) + (value >> 2);
    value ^= static_cast<std::uint32_t>(agent_id + 0x85ebca6b) + (value << 6) + (value >> 2);
    value ^= static_cast<std::uint32_t>(node_time_step + 0xc2b2ae35) + (value << 6) + (value >> 2);
    return value;
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

TrajectoryPrediction MultiAgentPlanner::makeAgentPrediction(const AgentRuntime& agent) const {
    TrajectoryPrediction prediction;
    if (agent.planned.empty()) {
        return prediction;
    }
    prediction.nodes = agent.planned.nodes;
    prediction.nodes.front().position = agent.state.mean;
    prediction.nodes.front().variance = agent.state.variance;
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
    int exclude_id,
    int priority_limit) const {
    std::vector<TrajectoryPrediction> predictions;
    for (const auto& agent : agents) {
        if (agent.spec.id == exclude_id ||
            agent.spec.priority >= priority_limit ||
            agent.planned.empty()) {
            continue;
        }
        predictions.push_back(makeAgentPrediction(agent));
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

    const auto agent_predictions =
        collectAgentPredictions(agents, agent.spec.id, agent.spec.priority);
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
    const PlannerConfig risk_config = visualizationRiskConfig(config_);

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

        const auto agent_predictions = collectAgentPredictions(agents, agent.spec.id);
        snapshot.planned_collision_probabilities.reserve(snapshot.planned.nodes.size());
        for (const auto& node : snapshot.planned.nodes) {
            std::mt19937 risk_rng(mixSeed(
                static_cast<std::uint32_t>(config_.rng_seed),
                timestep,
                agent.spec.id,
                node.time_step));
            MonteCarloCollisionChecker risk_checker(risk_config, risk_rng);
            const GaussianState robot{node.position, node.variance};
            const double probability = risk_checker.estimateCollisionProbability(
                robot,
                environment.static_obstacles,
                agent_predictions,
                dynamic_predictions,
                node.time_step);
            snapshot.planned_collision_probabilities.push_back(probability);
            snapshot.max_collision_probability =
                std::max(snapshot.max_collision_probability, probability);
        }
        frame.max_collision_probability =
            std::max(frame.max_collision_probability, snapshot.max_collision_probability);
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

    auto finishResult = [&](bool success) {
        result.success = success && allAgentsAtGoal(agents);
        result.agent_paths.resize(agents.size());
        result.total_steps = 0;
        result.max_timestep = 0;
        for (std::size_t i = 0; i < agents.size(); ++i) {
            result.agent_paths[i] = agents[i].executed;
            result.total_steps += static_cast<int>(agents[i].executed.size());
            for (const auto& step : agents[i].executed) {
                result.max_timestep = std::max(result.max_timestep, step.timestep);
            }
        }

        const auto clock_end = std::chrono::steady_clock::now();
        result.elapsed_ms =
            std::chrono::duration<double, std::milli>(clock_end - clock_start).count();
    };

    // Algorithm 2: receding horizon execution loop.
    while (!allAgentsAtGoal(agents) && timestep < config_.max_timesteps) {
        for (auto& agent : agents) {
            if (agent.at_goal) {
                continue;
            }

            if (agent.planned.nodes.size() < 2) {
                agent.at_goal = agent.state.mean.distance(agent.spec.goal) <= config_.expand_distance;
                if (!agent.at_goal) {
                    const auto agent_predictions =
                        collectAgentPredictions(agents, agent.spec.id, agent.spec.priority);
                    agent.planned = planner_.plan(
                        agent.state,
                        agent.spec.goal,
                        environment.static_obstacles,
                        agent_predictions,
                        dynamic_predictions,
                        0);
                    if (agent.planned.empty()) {
                        std::cerr << "Agent " << agent.spec.name
                                  << " exhausted its trajectory and no feasible replan was found.\n";
                        finishResult(false);
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
                    ++result.replan_count;
                }
                continue;
            }

            const auto agent_predictions =
                collectAgentPredictions(agents, agent.spec.id, agent.spec.priority);
            Trajectory replanned = planner_.plan(
                agent.state,
                agent.spec.goal,
                environment.static_obstacles,
                agent_predictions,
                dynamic_predictions,
                0);

            const bool lazy_safe =
                lazyCheck(agent, agents, environment.static_obstacles, dynamic_predictions);

            if (!lazy_safe && replanned.empty()) {
                const int default_max_iterations = config_.max_iterations;
                const int default_goal_sample_rate = config_.goal_sample_rate;
                config_.max_iterations = default_max_iterations * 4;
                config_.goal_sample_rate = std::min(50, default_goal_sample_rate + 20);
                replanned = planner_.plan(
                    agent.state,
                    agent.spec.goal,
                    environment.static_obstacles,
                    agent_predictions,
                    dynamic_predictions,
                    0);
                config_.max_iterations = default_max_iterations;
                config_.goal_sample_rate = default_goal_sample_rate;
            }

            if (!lazy_safe && replanned.empty()) {
                std::cerr << "Unsafe path detected for agent " << agent.spec.name
                          << " and no feasible replan was found.\n";
                finishResult(false);
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

            const bool adopt_replan =
                !lazy_safe ||
                (!replanned.empty() && replanned.nodes.size() < agent.planned.nodes.size());

            if (adopt_replan && !replanned.empty()) {
                agent.planned = replanned;
                ++result.replan_count;
            }

            const Vec2 target = agent.planned.nodes[1].position;
            const double step_size = effectiveMotionStep(config_);
            const Vec2 next_position = advanceToward(agent.state.mean, target, step_size);
            const bool reached_goal =
                next_position.distance(agent.spec.goal) <= config_.expand_distance;
            const Vec2 executed_position = reached_goal ? agent.spec.goal : next_position;
            const bool reached_waypoint =
                next_position.distance(target) <= 1e-9 ||
                agent.state.mean.distance(target) <= step_size;

            const Vec2 measurement = kalman_.simulateMeasurement(executed_position, rng_);
            agent.state = kalman_.measurementUpdate(agent.state, measurement);
            agent.state.mean = executed_position;

            ExecutedStep step;
            step.agent_id = agent.spec.id;
            step.timestep = timestep;
            step.position = agent.state.mean;
            step.variance = agent.state.variance;
            step.replanned = adopt_replan && !replanned.empty();
            agent.executed.push_back(step);

            if (!agent.planned.empty()) {
                agent.planned.nodes.front().position = agent.state.mean;
                agent.planned.nodes.front().variance = agent.state.variance;
            }

            if (reached_waypoint) {
                shiftTrajectory(agent.planned);
            }

            if (reached_goal) {
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

    finishResult(true);

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
