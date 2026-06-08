/**
 * @file trajectory_exporter.hpp
 * @brief CSV/JSON export of simulation trajectories and run summaries.
 */

#pragma once

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
};

}  // namespace ccrrt
