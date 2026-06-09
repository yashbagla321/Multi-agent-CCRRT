/**
 * @file main.cpp
 * @brief Command-line entry point for multi-agent CC-RRT simulations.
 */

#include "ccrrt/config.hpp"
#include "ccrrt/multi_agent_planner.hpp"
#include "ccrrt/sfml_renderer.hpp"
#include "ccrrt/trajectory_exporter.hpp"
#include "scenarios/scenarios.hpp"

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

namespace {

void printUsage() {
    std::cout << "Usage: multi_agent_ccrrt --scenario <name> [options]\n"
              << "\nPaper scenarios:    figure5, figure6, figure7\n"
              << "Performance tests:    perf_cluttered, perf_four_agents, perf_narrow_passage,\n"
              << "                      perf_long_paths, perf_multi_dynamic, perf_stress\n"
              << "\nOptions:\n"
              << "  --preview          Visualize scenario layout only (no simulation)\n"
              << "  --preview-all      Preview all scenarios in sequence\n"
              << "  --list-scenarios   Print available scenarios and descriptions\n"
              << "  --benchmark-all    Run all performance scenarios; write benchmark.csv\n"
              << "  --no-viz           Disable SFML visualization after simulation\n"
              << "  --output <dir>     Output directory (default: output/<scenario>)\n"
              << "  --seed <n>         RNG seed (default: 42)\n"
              << "  --mc-samples <n>   Monte Carlo samples (default: 1000)\n";
}

const ccrrt::ScenarioEntry* findScenario(const std::string& name) {
    for (const auto& scenario : ccrrt::allScenarios()) {
        if (scenario.name == name) {
            return &scenario;
        }
    }
    return nullptr;
}

void printScenarioList() {
    std::cout << std::left;
    std::cout << "\nPaper scenarios:\n";
    for (const auto& scenario : ccrrt::paperScenarios()) {
        std::cout << "  " << std::setw(22) << scenario.name << scenario.description << '\n';
    }
    std::cout << "\nPerformance scenarios:\n";
    for (const auto& scenario : ccrrt::performanceScenarios()) {
        std::cout << "  " << std::setw(22) << scenario.name << scenario.description << '\n';
    }
    std::cout << '\n';
}

void printResultSummary(const ccrrt::SimulationResult& result) {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Success:      " << (result.success ? "yes" : "no") << '\n';
    std::cout << "Elapsed:      " << result.elapsed_ms << " ms\n";
    std::cout << "Replans:      " << result.replan_count << '\n';
    std::cout << "Total steps:  " << result.total_steps << '\n';
    std::cout << "Max timestep: " << result.max_timestep << '\n';
}

#if CCRRT_HAS_SFML
void previewScenario(const ccrrt::ScenarioEntry& scenario) {
    ccrrt::SFMLRenderer renderer;
    renderer.renderScenarioPreview(scenario.environment, scenario.name);
}
#endif

int runSingleScenario(
    const ccrrt::ScenarioEntry& scenario,
    ccrrt::PlannerConfig config,
    const std::string& output_directory,
    bool enable_visualization) {
    std::cout << "Running scenario: " << scenario.name << '\n';
    if (!scenario.description.empty()) {
        std::cout << "  " << scenario.description << '\n';
    }

    ccrrt::MultiAgentPlanner planner(config);
    const ccrrt::SimulationResult result = planner.run(scenario.environment, scenario.name);

    ccrrt::TrajectoryExporter exporter;
    if (!exporter.exportCsv(result, output_directory)) {
        return 1;
    }
    if (!exporter.exportSummaryJson(result, output_directory)) {
        return 1;
    }

    printResultSummary(result);
    std::cout << "Output:       " << output_directory << '\n';

#if CCRRT_HAS_SFML
    if (enable_visualization) {
        ccrrt::SFMLRenderer renderer;
        renderer.renderSimulationResult(scenario.environment, result);
    }
#else
    if (enable_visualization) {
        std::cout << "Visualization requested but SFML was not available at build time.\n";
    }
#endif

    return result.success ? 0 : 2;
}

int runBenchmarkAll(ccrrt::PlannerConfig config, const std::string& output_directory) {
    const auto scenarios = ccrrt::benchmarkScenarios();
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

}  // namespace

int main(int argc, char* argv[]) {
    std::string scenario_name = "figure5";
    std::string output_directory;
    bool enable_visualization = true;
    bool preview_only = false;
    bool preview_all = false;
    bool list_scenarios = false;
    bool benchmark_all = false;
    ccrrt::PlannerConfig config;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--scenario" && i + 1 < argc) {
            scenario_name = argv[++i];
        } else if (arg == "--preview") {
            preview_only = true;
        } else if (arg == "--preview-all") {
            preview_all = true;
        } else if (arg == "--list-scenarios") {
            list_scenarios = true;
        } else if (arg == "--benchmark-all") {
            benchmark_all = true;
            enable_visualization = false;
        } else if (arg == "--no-viz") {
            enable_visualization = false;
        } else if (arg == "--output" && i + 1 < argc) {
            output_directory = argv[++i];
        } else if (arg == "--seed" && i + 1 < argc) {
            config.rng_seed = static_cast<unsigned int>(std::stoul(argv[++i]));
        } else if (arg == "--mc-samples" && i + 1 < argc) {
            config.mc_samples = std::stoi(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            printUsage();
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << '\n';
            printUsage();
            return 1;
        }
    }

    if (list_scenarios) {
        printScenarioList();
        return 0;
    }

    if (preview_all) {
#if CCRRT_HAS_SFML
        for (const auto& scenario : ccrrt::allScenarios()) {
            previewScenario(scenario);
        }
        return 0;
#else
        std::cerr << "Preview requires SFML.\n";
        return 1;
#endif
    }

    if (benchmark_all) {
        if (output_directory.empty()) {
            output_directory = "output/benchmark";
        }
        return runBenchmarkAll(config, output_directory);
    }

    const ccrrt::ScenarioEntry* selected = findScenario(scenario_name);
    if (selected == nullptr) {
        std::cerr << "Unknown scenario: " << scenario_name << '\n';
        printUsage();
        return 1;
    }

    if (preview_only) {
#if CCRRT_HAS_SFML
        previewScenario(*selected);
        return 0;
#else
        std::cerr << "Preview requires SFML.\n";
        return 1;
#endif
    }

    if (output_directory.empty()) {
        output_directory = "output/" + scenario_name;
    }

    return runSingleScenario(*selected, config, output_directory, enable_visualization);
}
