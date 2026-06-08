/**
 * @file ccrrt.hpp
 * @brief Umbrella header for the multi-agent CC-RRT library.
 *
 * @mainpage Multi-Agent Chance-Constrained RRT
 *
 * C++17 implementation of the receding-horizon multi-agent CC-RRT algorithm from:
 * Bagla & Srivastava, "On Receding Horizon Chance Constraint Motion Planning
 * for Uncertain Multi-Agent Systems", DSCC 2019.
 *
 * ## Architecture
 *
 * ```
 * main.cpp / scenarios
 *        |
 *        v
 * MultiAgentPlanner  ---- Algorithm 1 (initial) + Algorithm 2 (receding horizon)
 *        |
 *        +-- CCRRTPlanner          (single-agent tree growth)
 *        +-- MonteCarloCollisionChecker  (chance constraints)
 *        +-- KalmanFilter          (measurement updates)
 *        +-- TrajectoryExporter / SFMLRenderer
 * ```
 *
 * ## Typical usage
 *
 * @code
 * ccrrt::PlannerConfig config;
 * ccrrt::MultiAgentPlanner planner(config);
 * ccrrt::SimulationResult result = planner.run(environment, "figure5");
 * @endcode
 *
 * @see MultiAgentPlanner
 * @see CCRRTPlanner
 * @see MonteCarloCollisionChecker
 */

#pragma once

#include "ccrrt/ccrrt_planner.hpp"
#include "ccrrt/collision_checker.hpp"
#include "ccrrt/config.hpp"
#include "ccrrt/geometry.hpp"
#include "ccrrt/kalman_filter.hpp"
#include "ccrrt/multi_agent_planner.hpp"
#include "ccrrt/sfml_renderer.hpp"
#include "ccrrt/trajectory_exporter.hpp"
#include "ccrrt/types.hpp"
