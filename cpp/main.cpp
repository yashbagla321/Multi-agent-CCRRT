/**
 * @file main.cpp
 * @brief Command-line entry point for multi-agent CC-RRT simulations.
 *
 * Parses CLI arguments, selects a paper scenario, runs MultiAgentPlanner,
 * exports trajectories to CSV/JSON, and optionally opens an SFML result window.
 */

#include "ccrrt/config.hpp"
#include "ccrrt/multi_agent_planner.hpp"
#include "ccrrt/sfml_renderer.hpp"
#include "ccrrt/trajectory_exporter.hpp"
#include "scenarios/paper_figures.hpp"

#include <iostream>
#include <string>

namespace {

/** @brief Prints CLI usage help to stdout. */
void printUsage() {
    std::cout << "Usage: multi_agent_ccrrt --scenario <figure5|figure6|figure7> [options]\n"
              << "Options:\n"
              << "  --no-viz           Disable SFML visualization\n"
              << "  --output <dir>     Output directory (default: output/<scenario>)\n"
              << "  --seed <n>         RNG seed (default: 42)\n"
              << "  --mc-samples <n>   Monte Carlo samples (default: 1000)\n";
}

}  // namespace

/**
 * @brief Program entry point.
 * @return 0 on success, 1 on CLI/IO error, 2 if simulation did not reach all goals.
 */
int main(int argc, char* argv[]) {
    std::string scenario_name = "figure5";
    std::string output_directory;
    bool enable_visualization = true;
    ccrrt::PlannerConfig config;

    // --- Parse command-line arguments ---
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--scenario" && i + 1 < argc) {
            scenario_name = argv[++i];
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

    // --- Resolve scenario by name ---
    const auto scenarios = ccrrt::allScenarios();
    const ccrrt::ScenarioEntry* selected = nullptr;
    for (const auto& scenario : scenarios) {
        if (scenario.name == scenario_name) {
            selected = &scenario;
            break;
        }
    }

    if (selected == nullptr) {
        std::cerr << "Unknown scenario: " << scenario_name << '\n';
        printUsage();
        return 1;
    }

    if (output_directory.empty()) {
        output_directory = "output/" + scenario_name;
    }

    // --- Run simulation ---
    std::cout << "Running scenario: " << selected->name << '\n';
    ccrrt::MultiAgentPlanner planner(config);
    const ccrrt::SimulationResult result = planner.run(selected->environment, selected->name);

    // --- Export results ---
    ccrrt::TrajectoryExporter exporter;
    if (!exporter.exportCsv(result, output_directory)) {
        return 1;
    }
    if (!exporter.exportSummaryJson(result, output_directory)) {
        return 1;
    }

    std::cout << "Success: " << (result.success ? "yes" : "no") << '\n';
    std::cout << "Replans: " << result.replan_count << '\n';
    std::cout << "Output: " << output_directory << '\n';

#if CCRRT_HAS_SFML
    if (enable_visualization) {
        ccrrt::SFMLRenderer renderer;
        renderer.renderSimulationResult(selected->environment, result);
    }
#else
    if (enable_visualization) {
        std::cout << "Visualization requested but SFML was not available at build time.\n";
    }
#endif

    return result.success ? 0 : 2;
}
