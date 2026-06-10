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
