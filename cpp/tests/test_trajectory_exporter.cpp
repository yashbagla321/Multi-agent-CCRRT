#include "ccrrt/trajectory_exporter.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace {

std::string readFile(const std::string& path) {
    std::ifstream file(path);
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

ccrrt::SimulationResult sampleResult() {
    ccrrt::SimulationResult result;
    result.scenario_name = "unit_test";
    result.success = true;
    result.replan_count = 2;
    result.elapsed_ms = 123.45;
    result.total_steps = 10;
    result.max_timestep = 9;

    ccrrt::ExecutedStep step;
    step.agent_id = 0;
    step.timestep = 0;
    step.position = {1.0, 2.0};
    step.variance = 0.2;
    step.replanned = false;
    result.agent_paths = {{step}};
    return result;
}

}  // namespace

TEST(TrajectoryExporter, WritesCsvAndJson) {
    const auto temp_dir = std::filesystem::temp_directory_path() / "ccrrt_test_export";
    std::filesystem::remove_all(temp_dir);

    const ccrrt::SimulationResult result = sampleResult();
    ccrrt::TrajectoryExporter exporter;

    ASSERT_TRUE(exporter.exportCsv(result, temp_dir.string()));
    ASSERT_TRUE(exporter.exportSummaryJson(result, temp_dir.string()));

    const std::string csv = readFile((temp_dir / "trajectories.csv").string());
    EXPECT_NE(csv.find("agent_id,timestep,x,y,variance,replanned"), std::string::npos);
    EXPECT_NE(csv.find("0,0,1"), std::string::npos);

    const std::string json = readFile((temp_dir / "summary.json").string());
    EXPECT_NE(json.find("\"scenario\": \"unit_test\""), std::string::npos);
    EXPECT_NE(json.find("\"elapsed_ms\": 123.45"), std::string::npos);
    EXPECT_NE(json.find("\"replan_count\": 2"), std::string::npos);

    std::filesystem::remove_all(temp_dir);
}

TEST(TrajectoryExporter, WritesBenchmarkCsv) {
    const auto temp_dir = std::filesystem::temp_directory_path() / "ccrrt_test_benchmark";
    std::filesystem::remove_all(temp_dir);

    ccrrt::TrajectoryExporter exporter;
    const std::vector<ccrrt::SimulationResult> rows = {sampleResult(), sampleResult()};

    ASSERT_TRUE(exporter.exportBenchmarkCsv(rows, temp_dir.string()));

    const std::string csv = readFile((temp_dir / "benchmark.csv").string());
    EXPECT_NE(csv.find("scenario,success,elapsed_ms"), std::string::npos);
    EXPECT_NE(csv.find("unit_test"), std::string::npos);

    std::filesystem::remove_all(temp_dir);
}

TEST(TrajectoryExporter, WritesScenarioJson) {
    const auto temp_dir = std::filesystem::temp_directory_path() / "ccrrt_test_scenario_export";
    std::filesystem::remove_all(temp_dir);

    ccrrt::Environment environment;
    environment.bounds_min = -4.0;
    environment.bounds_max = 17.0;
    environment.static_obstacles.push_back({{7.0, 4.0}, 2.0});
    environment.agents.push_back({0, 0, {5.0, -1.0}, {0.0, 13.0}, "red"});
    ccrrt::DynamicObstacleSpec dynamic;
    dynamic.id = 3;
    dynamic.initial_variance = 0.15;
    dynamic.waypoints = {{10.0, 0.0}, {10.0, 1.0}};
    dynamic.variance_per_waypoint = {0.2, 0.4};
    environment.dynamic_obstacles.push_back(dynamic);

    ccrrt::TrajectoryExporter exporter;
    ASSERT_TRUE(exporter.exportScenarioJson("figure5", "Paper Fig. 5", environment, temp_dir.string()));

    const std::string json = readFile((temp_dir / "scenario.json").string());
    EXPECT_NE(json.find("\"name\": \"figure5\""), std::string::npos);
    EXPECT_NE(json.find("\"static_obstacles\""), std::string::npos);
    EXPECT_NE(json.find("\"dynamic_obstacles\""), std::string::npos);
    EXPECT_NE(json.find("\"variance_per_waypoint\": [0.2000, 0.4000]"), std::string::npos);

    std::filesystem::remove_all(temp_dir);
}

TEST(TrajectoryExporter, WritesReplayFramesJson) {
    const auto temp_dir = std::filesystem::temp_directory_path() / "ccrrt_test_replay_frames";
    std::filesystem::remove_all(temp_dir);

    ccrrt::SimulationFrame frame;
    frame.scenario_name = "figure5";
    frame.timestep = 4;
    frame.initial_plan_ready = false;
    frame.simulation_complete = false;
    frame.max_collision_probability = 0.125;

    ccrrt::AgentSnapshot agent;
    agent.spec = {0, 0, {0.0, 0.0}, {3.0, 3.0}, "red"};
    agent.position = {1.0, 1.0};
    agent.variance = 0.2;
    agent.replanned_this_step = true;
    agent.max_collision_probability = 0.125;
    agent.planned.nodes.push_back({{1.0, 1.0}, 0.2, 0});
    agent.planned.nodes.push_back({{2.0, 2.0}, 0.3, 1});
    agent.planned_collision_probabilities = {0.025, 0.125};
    frame.agents.push_back(agent);

    ccrrt::TrajectoryPrediction dynamic_prediction;
    dynamic_prediction.nodes.push_back({{4.0, 4.0}, 0.2, 0});
    frame.dynamic_predictions.push_back(dynamic_prediction);

    ccrrt::TrajectoryExporter exporter;
    ASSERT_TRUE(exporter.exportReplayFramesJson({frame}, temp_dir.string()));

    const std::string json = readFile((temp_dir / "replay_frames.json").string());
    EXPECT_NE(json.find("\"frames\""), std::string::npos);
    EXPECT_NE(json.find("\"max_collision_probability\": 0.125000"), std::string::npos);
    EXPECT_NE(json.find("\"replanned_this_step\": true"), std::string::npos);
    EXPECT_NE(json.find("\"collision_probability\": 0.125000"), std::string::npos);
    EXPECT_NE(json.find("\"dynamic_predictions\""), std::string::npos);

    std::filesystem::remove_all(temp_dir);
}
