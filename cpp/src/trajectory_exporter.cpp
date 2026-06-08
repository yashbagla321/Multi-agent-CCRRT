/**
 * @file trajectory_exporter.cpp
 * @brief CSV and JSON export of simulation results.
 *
 * @see ccrrt/trajectory_exporter.hpp
 */

#include "ccrrt/trajectory_exporter.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace ccrrt {

namespace {

/** @brief Creates @p path and parent directories if they do not exist. */
bool ensureDirectory(const std::string& path) {
    std::error_code error;
    std::filesystem::create_directories(path, error);
    if (error) {
        std::cerr << "Failed to create directory: " << path << " (" << error.message() << ")\n";
        return false;
    }
    return true;
}

}  // namespace

bool TrajectoryExporter::exportCsv(const SimulationResult& result, const std::string& output_directory) const {
    if (!ensureDirectory(output_directory)) {
        return false;
    }

    std::ostringstream filename;
    filename << output_directory << "/trajectories.csv";
    std::ofstream file(filename.str());
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename.str() << " for writing\n";
        return false;
    }

    file << "agent_id,timestep,x,y,variance,replanned\n";
    file << std::fixed << std::setprecision(4);
    for (std::size_t agent_index = 0; agent_index < result.agent_paths.size(); ++agent_index) {
        for (const auto& step : result.agent_paths[agent_index]) {
            file << step.agent_id << ',' << step.timestep << ',' << step.position.x << ','
                 << step.position.y << ',' << step.variance << ','
                 << (step.replanned ? "1" : "0") << '\n';
        }
    }
    return true;
}

bool TrajectoryExporter::exportSummaryJson(const SimulationResult& result, const std::string& output_directory) const {
    if (!ensureDirectory(output_directory)) {
        return false;
    }

    std::ostringstream filename;
    filename << output_directory << "/summary.json";
    std::ofstream file(filename.str());
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename.str() << " for writing\n";
        return false;
    }

    file << std::fixed << std::setprecision(4);
    file << "{\n";
    file << "  \"scenario\": \"" << result.scenario_name << "\",\n";
    file << "  \"success\": " << (result.success ? "true" : "false") << ",\n";
    file << "  \"replan_count\": " << result.replan_count << ",\n";
    file << "  \"agents\": [\n";

    for (std::size_t i = 0; i < result.agent_paths.size(); ++i) {
        file << "    {\n";
        file << "      \"agent_index\": " << i << ",\n";
        file << "      \"steps\": " << result.agent_paths[i].size() << "\n";
        file << "    }";
        if (i + 1 < result.agent_paths.size()) {
            file << ',';
        }
        file << '\n';
    }

    file << "  ]\n";
    file << "}\n";
    return true;
}

}  // namespace ccrrt
