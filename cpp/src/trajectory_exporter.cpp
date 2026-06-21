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
#include <ostream>
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

std::string escapeJsonString(const std::string& value) {
    std::ostringstream out;
    for (const char ch : value) {
        switch (ch) {
            case '\\':
                out << "\\\\";
                break;
            case '"':
                out << "\\\"";
                break;
            case '\n':
                out << "\\n";
                break;
            case '\r':
                out << "\\r";
                break;
            case '\t':
                out << "\\t";
                break;
            default:
                out << ch;
                break;
        }
    }
    return out.str();
}

void writePoint(std::ostream& out, const Vec2& point) {
    out << '[' << point.x << ", " << point.y << ']';
}

void writeTrajectoryNode(
    std::ostream& out,
    const TrajectoryNode& node,
    const double* collision_probability = nullptr) {
    out << "{ \"time_step\": " << node.time_step
        << ", \"x\": " << node.position.x
        << ", \"y\": " << node.position.y
        << ", \"variance\": " << node.variance;
    if (collision_probability != nullptr) {
        out << ", \"collision_probability\": " << *collision_probability;
    }
    out << " }";
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
    file << "  \"elapsed_ms\": " << result.elapsed_ms << ",\n";
    file << "  \"total_steps\": " << result.total_steps << ",\n";
    file << "  \"max_timestep\": " << result.max_timestep << ",\n";
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

bool TrajectoryExporter::exportScenarioJson(
    const std::string& scenario_name,
    const std::string& description,
    const Environment& environment,
    const std::string& output_directory) const {
    if (!ensureDirectory(output_directory)) {
        return false;
    }

    std::ostringstream filename;
    filename << output_directory << "/scenario.json";
    std::ofstream file(filename.str());
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename.str() << " for writing\n";
        return false;
    }

    file << std::fixed << std::setprecision(4);
    file << "{\n";
    file << "  \"name\": \"" << escapeJsonString(scenario_name) << "\",\n";
    file << "  \"description\": \"" << escapeJsonString(description) << "\",\n";
    file << "  \"bounds\": { \"min\": " << environment.bounds_min
         << ", \"max\": " << environment.bounds_max << " },\n";

    file << "  \"static_obstacles\": [\n";
    for (std::size_t i = 0; i < environment.static_obstacles.size(); ++i) {
        const auto& obstacle = environment.static_obstacles[i];
        file << "    { \"center\": ";
        writePoint(file, obstacle.center);
        file << ", \"radius\": " << obstacle.radius << " }";
        if (i + 1 < environment.static_obstacles.size()) {
            file << ',';
        }
        file << '\n';
    }
    file << "  ],\n";

    file << "  \"agents\": [\n";
    for (std::size_t i = 0; i < environment.agents.size(); ++i) {
        const auto& agent = environment.agents[i];
        file << "    { \"id\": " << agent.id
             << ", \"priority\": " << agent.priority
             << ", \"name\": \"" << escapeJsonString(agent.name) << "\", \"start\": ";
        writePoint(file, agent.start);
        file << ", \"goal\": ";
        writePoint(file, agent.goal);
        file << " }";
        if (i + 1 < environment.agents.size()) {
            file << ',';
        }
        file << '\n';
    }
    file << "  ],\n";

    file << "  \"dynamic_obstacles\": [\n";
    for (std::size_t i = 0; i < environment.dynamic_obstacles.size(); ++i) {
        const auto& obstacle = environment.dynamic_obstacles[i];
        file << "    {\n";
        file << "      \"id\": " << obstacle.id << ",\n";
        file << "      \"initial_variance\": " << obstacle.initial_variance << ",\n";
        file << "      \"waypoints\": [\n";
        for (std::size_t j = 0; j < obstacle.waypoints.size(); ++j) {
            file << "        ";
            writePoint(file, obstacle.waypoints[j]);
            if (j + 1 < obstacle.waypoints.size()) {
                file << ',';
            }
            file << '\n';
        }
        file << "      ]";
        if (!obstacle.variance_per_waypoint.empty()) {
            file << ",\n";
            file << "      \"variance_per_waypoint\": [";
            for (std::size_t j = 0; j < obstacle.variance_per_waypoint.size(); ++j) {
                if (j > 0) {
                    file << ", ";
                }
                file << obstacle.variance_per_waypoint[j];
            }
            file << "]\n";
        } else {
            file << '\n';
        }
        file << "    }";
        if (i + 1 < environment.dynamic_obstacles.size()) {
            file << ',';
        }
        file << '\n';
    }
    file << "  ]\n";
    file << "}\n";
    return true;
}

bool TrajectoryExporter::exportReplayFramesJson(
    const std::vector<SimulationFrame>& frames,
    const std::string& output_directory) const {
    if (!ensureDirectory(output_directory)) {
        return false;
    }

    std::ostringstream filename;
    filename << output_directory << "/replay_frames.json";
    std::ofstream file(filename.str());
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename.str() << " for writing\n";
        return false;
    }

    file << std::fixed << std::setprecision(6);
    file << "{\n";
    file << "  \"frames\": [\n";
    for (std::size_t frame_index = 0; frame_index < frames.size(); ++frame_index) {
        const auto& frame = frames[frame_index];
        file << "    {\n";
        file << "      \"scenario\": \"" << escapeJsonString(frame.scenario_name) << "\",\n";
        file << "      \"timestep\": " << frame.timestep << ",\n";
        file << "      \"initial_plan_ready\": "
             << (frame.initial_plan_ready ? "true" : "false") << ",\n";
        file << "      \"simulation_complete\": "
             << (frame.simulation_complete ? "true" : "false") << ",\n";
        file << "      \"max_collision_probability\": "
             << frame.max_collision_probability << ",\n";

        file << "      \"agents\": [\n";
        for (std::size_t agent_index = 0; agent_index < frame.agents.size(); ++agent_index) {
            const auto& agent = frame.agents[agent_index];
            file << "        {\n";
            file << "          \"id\": " << agent.spec.id << ",\n";
            file << "          \"priority\": " << agent.spec.priority << ",\n";
            file << "          \"name\": \"" << escapeJsonString(agent.spec.name) << "\",\n";
            file << "          \"x\": " << agent.position.x << ",\n";
            file << "          \"y\": " << agent.position.y << ",\n";
            file << "          \"variance\": " << agent.variance << ",\n";
            file << "          \"at_goal\": " << (agent.at_goal ? "true" : "false") << ",\n";
            file << "          \"replanned_this_step\": "
                 << (agent.replanned_this_step ? "true" : "false") << ",\n";
            file << "          \"max_collision_probability\": "
                 << agent.max_collision_probability << ",\n";
            file << "          \"planned\": [\n";
            for (std::size_t node_index = 0; node_index < agent.planned.nodes.size(); ++node_index) {
                file << "            ";
                const double* probability = nullptr;
                if (node_index < agent.planned_collision_probabilities.size()) {
                    probability = &agent.planned_collision_probabilities[node_index];
                }
                writeTrajectoryNode(file, agent.planned.nodes[node_index], probability);
                if (node_index + 1 < agent.planned.nodes.size()) {
                    file << ',';
                }
                file << '\n';
            }
            file << "          ]\n";
            file << "        }";
            if (agent_index + 1 < frame.agents.size()) {
                file << ',';
            }
            file << '\n';
        }
        file << "      ],\n";

        file << "      \"dynamic_predictions\": [\n";
        for (std::size_t prediction_index = 0;
             prediction_index < frame.dynamic_predictions.size();
             ++prediction_index) {
            const auto& prediction = frame.dynamic_predictions[prediction_index];
            file << "        {\n";
            file << "          \"nodes\": [\n";
            for (std::size_t node_index = 0; node_index < prediction.nodes.size(); ++node_index) {
                file << "            ";
                writeTrajectoryNode(file, prediction.nodes[node_index]);
                if (node_index + 1 < prediction.nodes.size()) {
                    file << ',';
                }
                file << '\n';
            }
            file << "          ]\n";
            file << "        }";
            if (prediction_index + 1 < frame.dynamic_predictions.size()) {
                file << ',';
            }
            file << '\n';
        }
        file << "      ]\n";
        file << "    }";
        if (frame_index + 1 < frames.size()) {
            file << ',';
        }
        file << '\n';
    }
    file << "  ]\n";
    file << "}\n";
    return true;
}

bool TrajectoryExporter::exportBenchmarkCsv(
    const std::vector<SimulationResult>& rows,
    const std::string& output_directory) const {
    if (!ensureDirectory(output_directory)) {
        return false;
    }

    std::ostringstream filename;
    filename << output_directory << "/benchmark.csv";
    std::ofstream file(filename.str());
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename.str() << " for writing\n";
        return false;
    }

    file << "scenario,success,elapsed_ms,replan_count,total_steps,max_timestep\n";
    file << std::fixed << std::setprecision(2);
    for (const auto& row : rows) {
        file << row.scenario_name << ','
             << (row.success ? "1" : "0") << ','
             << row.elapsed_ms << ','
             << row.replan_count << ','
             << row.total_steps << ','
             << row.max_timestep << '\n';
    }
    return true;
}

}  // namespace ccrrt
