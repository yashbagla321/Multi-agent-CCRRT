/**
 * @file trajectory_exporter.hpp
 * @brief CSV/JSON export of simulation trajectories and run summaries.
 */

#pragma once

#include "ccrrt/simulation_observer.hpp"
#include "ccrrt/types.hpp"

#include <string>

namespace ccrrt {

/**
 * @brief Writes simulation results to CSV and JSON files on disk.
 */
class TrajectoryExporter {
public:
    /**
     * @brief Exports executed agent steps to trajectories.csv.
     *
     * Columns: agent_id, timestep, x, y, variance, replanned.
     * Creates @p output_directory if it does not exist.
     *
     * @param result Simulation output to serialize.
     * @param output_directory Target directory path.
     * @return True on successful write.
     */
    bool exportCsv(const SimulationResult& result, const std::string& output_directory) const;

    /**
     * @brief Exports a run summary to summary.json.
     *
     * Includes scenario name, success flag, replan count, and per-agent step counts.
     * Creates @p output_directory if it does not exist.
     *
     * @param result Simulation output to serialize.
     * @param output_directory Target directory path.
     * @return True on successful write.
     */
    bool exportSummaryJson(const SimulationResult& result, const std::string& output_directory) const;

    /**
     * @brief Exports the scenario geometry used by a run to scenario.json.
     *
     * Keeping geometry next to trajectories.csv makes replay folders self-contained.
     *
     * @param scenario_name Name of the scenario that produced the replay.
     * @param description Human-readable scenario description.
     * @param environment Bounds, agents, and obstacles used by the planner.
     * @param output_directory Target directory path.
     * @return True on successful write.
     */
    bool exportScenarioJson(
        const std::string& scenario_name,
        const std::string& description,
        const Environment& environment,
        const std::string& output_directory) const;

    /**
     * @brief Exports per-timestep replay frames with active plans and risk estimates.
     *
     * The browser replay viewer uses this to show receding-horizon trajectories,
     * covariance at future nodes, replans, and max collision probability over time.
     *
     * @param frames Ordered frames captured during MultiAgentPlanner::run().
     * @param output_directory Target directory path.
     * @return True on successful write.
     */
    bool exportReplayFramesJson(
        const std::vector<SimulationFrame>& frames,
        const std::string& output_directory) const;

    /**
     * @brief Writes a benchmark summary table (one row per prior run) to benchmark.csv.
     * @param rows Collection of results to append as rows.
     * @param output_directory Target directory path.
     * @return True on successful write.
     */
    bool exportBenchmarkCsv(
        const std::vector<SimulationResult>& rows,
        const std::string& output_directory) const;
};

}  // namespace ccrrt
