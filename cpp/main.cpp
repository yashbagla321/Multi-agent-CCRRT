/**
 * @file main.cpp
 * @brief Command-line entry point for multi-agent CC-RRT simulations.
 */

#include "ccrrt/config.hpp"
#include "ccrrt/multi_agent_planner.hpp"
#include "ccrrt/runtime_config.hpp"
#include "ccrrt/simulation_observer.hpp"
#include "ccrrt/trajectory_exporter.hpp"

#include <iostream>
#include <iomanip>
#include <optional>
#include <string>
#include <vector>

namespace {

/** @brief Collects replay frames during a single scenario run for JSON export. */
class ReplayFrameCollector final : public ccrrt::ISimulationObserver {
public:
    /** @brief Stores @p frame and allows the simulation to continue. */
    bool onFrame(const ccrrt::SimulationFrame& frame) override {
        frames.push_back(frame);
        return true;
    }

    std::vector<ccrrt::SimulationFrame> frames;
};

/** @brief Prints command-line usage and supported runtime options. */
void printUsage() {
    std::cout << "Usage: multi_agent_ccrrt [options]\n"
              << "\nConfiguration (no rebuild needed):\n"
              << "  --config <file>    Load ccrrt.json (default: config/ccrrt.json if present)\n"
              << "  Scenarios live in config/scenarios.json (see scenarios_file in ccrrt.json).\n"
              << "\nOptions:\n"
              << "  --scenario <name>  Scenario to run (overrides config)\n"
              << "  --list-scenarios   Print available scenarios and descriptions\n"
              << "  --run-all          Run every scenario twice: normal and smoothed\n"
              << "  --run-all-normal   Run every scenario with smoothing disabled\n"
              << "  --run-all-smooth   Run every scenario with smoothing enabled\n"
              << "  --benchmark-all    Run all performance scenarios; write benchmark.csv\n"
              << "  --output <dir>     Output directory/root (default: output/<scenario>)\n"
              << "  --seed <n>         RNG seed (default: 42)\n"
              << "  --mc-samples <n>   Monte Carlo samples (default: 1000)\n"
              << "  --path-smoothing   Enable RRT shortcut path smoothing\n"
              << "  --no-path-smoothing Disable RRT shortcut path smoothing\n"
              << "  --python-compat    Use Multiagent CCRRT.py planner settings\n"
              << "\nVisualization:\n"
              << "  Run output is written as CSV/JSON. Open tools/replay_viewer.html and\n"
              << "  select the output folder to replay paths, plans, obstacles, covariance,\n"
              << "  replans, and current/next/max collision probability.\n"
              << "\nSee CONFIG.md for the full schema; config/ccrrt.json has per-field descriptions.\n";
}

/** @brief Prints scenarios grouped by paper and performance categories. */
void printScenarioList(const ccrrt::ScenarioRegistry& registry) {
    std::cout << std::left;
    std::cout << "\nPaper scenarios:\n";
    for (const auto& scenario : registry.paperScenarios()) {
        std::cout << "  " << std::setw(22) << scenario.name << scenario.description << '\n';
    }
    std::cout << "\nPerformance scenarios:\n";
    for (const auto& scenario : registry.performanceScenarios()) {
        std::cout << "  " << std::setw(22) << scenario.name << scenario.description << '\n';
    }
    std::cout << '\n';
}

/** @brief Prints the compact run summary shown after each scenario. */
void printResultSummary(const ccrrt::SimulationResult& result) {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Success:      " << (result.success ? "yes" : "no") << '\n';
    std::cout << "Elapsed:      " << result.elapsed_ms << " ms\n";
    std::cout << "Replans:      " << result.replan_count << '\n';
    std::cout << "Total steps:  " << result.total_steps << '\n';
    std::cout << "Max timestep: " << result.max_timestep << '\n';
}

/** @brief Loads the JSON config and applies command-line overrides. */
ccrrt::AppConfig loadConfiguration(int argc, char* argv[]) {
    ccrrt::AppConfig app;

    bool explicit_config = false;
    const std::string config_path = ccrrt::resolveConfigFilePath(argc, argv, explicit_config);
    if (!config_path.empty()) {
        const std::optional<ccrrt::AppConfig> loaded = ccrrt::loadAppConfigFromFile(config_path);
        if (loaded.has_value()) {
            app = *loaded;
            std::cout << "Loaded config: " << config_path << '\n';
        } else if (explicit_config) {
            std::cerr << "Aborting due to config load failure.\n";
            std::exit(1);
        }
    } else {
        std::cerr << "No config file found. Create config/ccrrt.json or pass --config <file>.\n";
        std::exit(1);
    }

    ccrrt::applyCommandLineOverrides(app, argc, argv);
    return app;
}

/** @brief Default replay output directory for a scenario and smoothing mode. */
std::string scenarioOutputDirectory(const std::string& scenario_name, bool smoothing_enabled) {
    return "output/" + scenario_name + (smoothing_enabled ? "_smooth" : "");
}

/** @brief Default benchmark output directory for a smoothing mode. */
std::string benchmarkOutputDirectory(bool smoothing_enabled) {
    return std::string("output/benchmark") + (smoothing_enabled ? "_smooth" : "");
}

/** @brief Runs one selected scenario and writes all replay artifacts. */
int runSingleScenario(
    const ccrrt::ScenarioEntry& scenario,
    ccrrt::PlannerConfig config,
    const std::string& output_directory,
    ccrrt::SimulationResult* result_out = nullptr) {
    std::cout << "Running scenario: " << scenario.name << '\n';
    if (!scenario.description.empty()) {
        std::cout << "  " << scenario.description << '\n';
    }

    ccrrt::MultiAgentPlanner planner(config);
    ReplayFrameCollector replay_collector;
    const ccrrt::SimulationResult result =
        planner.run(scenario.environment, scenario.name, &replay_collector);

    ccrrt::TrajectoryExporter exporter;
    if (!exporter.exportCsv(result, output_directory)) {
        return 1;
    }
    if (!exporter.exportSummaryJson(result, output_directory)) {
        return 1;
    }
    if (!exporter.exportScenarioJson(
            scenario.name,
            scenario.description,
            scenario.environment,
            output_directory)) {
        return 1;
    }
    if (!exporter.exportReplayFramesJson(replay_collector.frames, output_directory)) {
        return 1;
    }

    if (result_out != nullptr) {
        *result_out = result;
    }

    printResultSummary(result);
    std::cout << "Output:       " << output_directory << '\n';
    std::cout << "Replay:       open tools/replay_viewer.html and select this output folder\n";

    return result.success ? 0 : 2;
}

/** @brief Runs all benchmark scenarios and writes per-scenario outputs plus benchmark.csv. */
int runBenchmarkAll(
    const ccrrt::ScenarioRegistry& registry,
    ccrrt::PlannerConfig config,
    const std::string& output_directory) {
    const auto scenarios = registry.benchmarkScenarios();
    std::vector<ccrrt::SimulationResult> results;
    results.reserve(scenarios.size());

    std::cout << "Running " << scenarios.size() << " performance scenarios (no visualization)\n\n";

    ccrrt::TrajectoryExporter exporter;

    std::cout << std::left << std::fixed << std::setprecision(2);
    std::cout << std::setw(22) << "Scenario"
              << std::setw(10) << "Success"
              << std::setw(12) << "Time(ms)"
              << std::setw(10) << "Replans"
              << std::setw(12) << "Steps"
              << "MaxT\n";
    std::cout << std::string(72, '-') << '\n';

    for (const auto& scenario : scenarios) {
        ccrrt::MultiAgentPlanner planner(config);
        ccrrt::SimulationResult result = planner.run(scenario.environment, scenario.name);
        results.push_back(result);

        const std::string per_scenario_dir = output_directory + "/" + scenario.name;
        exporter.exportCsv(result, per_scenario_dir);
        exporter.exportSummaryJson(result, per_scenario_dir);
        exporter.exportScenarioJson(
            scenario.name,
            scenario.description,
            scenario.environment,
            per_scenario_dir);

        std::cout << std::setw(22) << scenario.name
                  << std::setw(10) << (result.success ? "yes" : "no")
                  << std::setw(12) << result.elapsed_ms
                  << std::setw(10) << result.replan_count
                  << std::setw(12) << result.total_steps
                  << result.max_timestep << '\n';
    }

    if (!exporter.exportBenchmarkCsv(results, output_directory)) {
        return 1;
    }

    std::cout << "\nBenchmark summary: " << output_directory << "/benchmark.csv\n";
    return 0;
}

/** @brief Runs all scenarios in normal and smoothed modes with separate output folders. */
int runAllScenarios(
    const ccrrt::ScenarioRegistry& registry,
    ccrrt::PlannerConfig config,
    const std::string& output_root,
    const std::vector<bool>& smoothing_modes) {
    const auto scenarios = registry.all();
    const std::string root_prefix = output_root.empty() ? "output" : output_root;
    ccrrt::TrajectoryExporter exporter;
    std::vector<ccrrt::SimulationResult> benchmark_normal;
    std::vector<ccrrt::SimulationResult> benchmark_smooth;
    int status = 0;

    for (const bool smoothing_enabled : smoothing_modes) {
        ccrrt::PlannerConfig mode_config = config;
        mode_config.enable_path_smoothing = smoothing_enabled;
        std::cout << "\nRunning all scenarios ("
                  << (smoothing_enabled ? "smoothed" : "normal")
                  << ")\n";

        for (const auto& scenario : scenarios) {
            const std::string suffix = smoothing_enabled ? "_smooth" : "";
            const std::string output_directory = root_prefix + "/" + scenario.name + suffix;
            ccrrt::SimulationResult result;
            const int result_code =
                runSingleScenario(scenario, mode_config, output_directory, &result);
            if (result_code != 0 && status == 0) {
                status = result_code;
            }

            if (scenario.category == ccrrt::ScenarioCategory::Performance) {
                if (smoothing_enabled) {
                    benchmark_smooth.push_back(result);
                } else {
                    benchmark_normal.push_back(result);
                }
            }
        }
    }

    if (!benchmark_normal.empty()) {
        const std::string benchmark_dir = root_prefix + "/benchmark";
        if (!exporter.exportBenchmarkCsv(benchmark_normal, benchmark_dir) && status == 0) {
            status = 1;
        }
        std::cout << "Benchmark summary: " << benchmark_dir << "/benchmark.csv\n";
    }
    if (!benchmark_smooth.empty()) {
        const std::string benchmark_dir = root_prefix + "/benchmark_smooth";
        if (!exporter.exportBenchmarkCsv(benchmark_smooth, benchmark_dir) && status == 0) {
            status = 1;
        }
        std::cout << "Benchmark summary: " << benchmark_dir << "/benchmark.csv\n";
    }

    return status;
}

/** @brief Returns true when the argv list contains --help or -h. */
bool hasHelpFlag(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            return true;
        }
    }
    return false;
}

}  // namespace

int main(int argc, char* argv[]) {
    if (hasHelpFlag(argc, argv)) {
        printUsage();
        return 0;
    }

    const ccrrt::AppConfig app = loadConfiguration(argc, argv);
    const ccrrt::ScenarioRegistry registry = ccrrt::makeScenarioRegistry(app);

    if (app.scenarios.empty()) {
        std::cerr << "No scenarios loaded. Check scenarios_file in your config.\n";
        return 1;
    }

    if (app.run.list_scenarios) {
        printScenarioList(registry);
        return 0;
    }

    if (app.run.run_all || app.run.run_all_normal || app.run.run_all_smooth) {
        std::vector<bool> smoothing_modes;
        if (app.run.run_all || app.run.run_all_normal) {
            smoothing_modes.push_back(false);
        }
        if (app.run.run_all || app.run.run_all_smooth) {
            smoothing_modes.push_back(true);
        }
        return runAllScenarios(registry, app.planner, app.run.output_directory, smoothing_modes);
    }

    if (app.run.benchmark_all) {
        std::string output_directory = app.run.output_directory;
        if (output_directory.empty()) {
            output_directory = benchmarkOutputDirectory(app.planner.enable_path_smoothing);
        }
        return runBenchmarkAll(registry, app.planner, output_directory);
    }

    const std::optional<ccrrt::ScenarioEntry> selected = registry.find(app.run.scenario);
    if (!selected.has_value()) {
        std::cerr << "Unknown scenario: " << app.run.scenario << '\n';
        printUsage();
        return 1;
    }

    std::string output_directory = app.run.output_directory;
    if (output_directory.empty()) {
        output_directory =
            scenarioOutputDirectory(app.run.scenario, app.planner.enable_path_smoothing);
    }

    if (app.run.python_compat) {
        std::cout << "Python-compat mode: legacy collision, expand=1.0, max_iter=500\n";
    }

    return runSingleScenario(
        *selected,
        app.planner,
        output_directory);
}
