/**
 * @file main.cpp
 * @brief Command-line entry point for multi-agent CC-RRT simulations.
 */

#include "ccrrt/config.hpp"
#include "ccrrt/multi_agent_planner.hpp"
#include "ccrrt/runtime_config.hpp"
#include "ccrrt/sfml_renderer.hpp"
#if CCRRT_HAS_SFML
#include "ccrrt/sfml_live_visualizer.hpp"
#endif
#include "ccrrt/trajectory_exporter.hpp"

#include <iostream>
#include <iomanip>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {

void printUsage() {
    std::cout << "Usage: multi_agent_ccrrt [options]\n"
              << "\nConfiguration (no rebuild needed):\n"
              << "  --config <file>    Load ccrrt.json (default: config/ccrrt.json if present)\n"
              << "  Scenarios live in config/scenarios.json (see scenarios_file in ccrrt.json).\n"
              << "\nOptions:\n"
              << "  --scenario <name>  Scenario to run (overrides config)\n"
              << "  --preview          Visualize scenario layout only (no simulation)\n"
              << "  --preview-all      Preview all scenarios in sequence\n"
              << "  --list-scenarios   Print available scenarios and descriptions\n"
              << "  --benchmark-all    Run all performance scenarios; write benchmark.csv\n"
              << "  --no-viz           Disable all SFML visualization\n"
              << "  --no-live-viz      Disable live step-by-step viz (keep post-run view)\n"
              << "  --viz-delay-ms <n> Delay between live viz frames in ms (default: 150)\n"
              << "  --output <dir>     Output directory (default: output/<scenario>)\n"
              << "  --seed <n>         RNG seed (default: 42)\n"
              << "  --mc-samples <n>   Monte Carlo samples (default: 1000)\n"
              << "  --python-compat    Use Multiagent CCRRT.py planner settings\n"
              << "\nSee CONFIG.md for the full schema; config/ccrrt.json has per-field descriptions.\n";
}

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

void printResultSummary(const ccrrt::SimulationResult& result) {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Success:      " << (result.success ? "yes" : "no") << '\n';
    std::cout << "Elapsed:      " << result.elapsed_ms << " ms\n";
    std::cout << "Replans:      " << result.replan_count << '\n';
    std::cout << "Total steps:  " << result.total_steps << '\n';
    std::cout << "Max timestep: " << result.max_timestep << '\n';
}

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
    bool enable_visualization,
    bool live_visualization,
    int viz_step_delay_ms) {
    std::cout << "Running scenario: " << scenario.name << '\n';
    if (!scenario.description.empty()) {
        std::cout << "  " << scenario.description << '\n';
    }

    ccrrt::MultiAgentPlanner planner(config);

#if CCRRT_HAS_SFML
    std::unique_ptr<ccrrt::SFMLLiveVisualizer> live_viz;
    ccrrt::ISimulationObserver* observer = nullptr;
    if (enable_visualization && live_visualization) {
        live_viz = std::make_unique<ccrrt::SFMLLiveVisualizer>(scenario.name, viz_step_delay_ms);
        observer = live_viz.get();
    }
    const ccrrt::SimulationResult result =
        planner.run(scenario.environment, scenario.name, observer);
#else
    const ccrrt::SimulationResult result = planner.run(scenario.environment, scenario.name);
#endif

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
    if (enable_visualization && !live_visualization) {
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

    if (app.run.preview_all) {
#if CCRRT_HAS_SFML
        for (const auto& scenario : registry.all()) {
            previewScenario(scenario);
        }
        return 0;
#else
        std::cerr << "Preview requires SFML.\n";
        return 1;
#endif
    }

    if (app.run.benchmark_all) {
        std::string output_directory = app.run.output_directory;
        if (output_directory.empty()) {
            output_directory = "output/benchmark";
        }
        return runBenchmarkAll(registry, app.planner, output_directory);
    }

    const std::optional<ccrrt::ScenarioEntry> selected = registry.find(app.run.scenario);
    if (!selected.has_value()) {
        std::cerr << "Unknown scenario: " << app.run.scenario << '\n';
        printUsage();
        return 1;
    }

    if (app.run.preview_only) {
#if CCRRT_HAS_SFML
        previewScenario(*selected);
        return 0;
#else
        std::cerr << "Preview requires SFML.\n";
        return 1;
#endif
    }

    std::string output_directory = app.run.output_directory;
    if (output_directory.empty()) {
        output_directory = "output/" + app.run.scenario;
    }

    if (app.run.python_compat) {
        std::cout << "Python-compat mode: legacy collision, expand=1.0, max_iter=500\n";
    }

    return runSingleScenario(
        *selected,
        app.planner,
        output_directory,
        app.run.enable_visualization,
        app.run.live_visualization,
        app.run.viz_step_delay_ms);
}
