/**
 * @file runtime_config.hpp
 * @brief JSON runtime configuration (no rebuild required to change parameters).
 */

#pragma once

#include "ccrrt/config.hpp"
#include "ccrrt/types.hpp"

#include <optional>
#include <string>
#include <vector>

namespace ccrrt {

/**
 * @brief CLI / run-time options that are not part of the planner algorithm itself.
 */
struct RunSettings {
    std::string scenario = "figure5";
    std::string output_directory;
    bool enable_visualization = true;
    bool live_visualization = true;
    int viz_step_delay_ms = 150;
    bool preview_only = false;
    bool preview_all = false;
    bool list_scenarios = false;
    bool benchmark_all = false;
    bool python_compat = false;
    std::string loaded_config_path;
};

/**
 * @brief Category tag for grouping scenarios in listings and benchmarks.
 */
enum class ScenarioCategory {
    Paper,
    Performance,
};

/**
 * @brief Named scenario with environment geometry (loaded from JSON).
 */
struct ScenarioEntry {
    std::string name;
    std::string description;
    ScenarioCategory category = ScenarioCategory::Paper;
    Environment environment;
};

/** @brief Scenario block as stored in ccrrt.json / scenarios.json. */
struct ConfigScenario {
    std::string name;
    std::string description;
    ScenarioCategory category = ScenarioCategory::Paper;
    Environment environment;
};

struct AppConfig {
    RunSettings run;
    PlannerConfig planner;
    std::vector<ConfigScenario> scenarios;
};

/**
 * @brief Lookup and listing over scenarios loaded from the config file.
 */
class ScenarioRegistry {
public:
    explicit ScenarioRegistry(std::vector<ConfigScenario> scenarios);

    std::optional<ScenarioEntry> find(const std::string& name) const;
    std::vector<ScenarioEntry> all() const;
    std::vector<ScenarioEntry> paperScenarios() const;
    std::vector<ScenarioEntry> performanceScenarios() const;
    std::vector<ScenarioEntry> benchmarkScenarios() const;

private:
    std::vector<ConfigScenario> scenarios_;
};

/**
 * @brief Loads configuration from a JSON file.
 * @param path Path to ccrrt.json (or compatible file).
 * @return Parsed config, or nullopt on I/O / parse errors (message written to stderr).
 */
std::optional<AppConfig> loadAppConfigFromFile(const std::string& path);

/**
 * @brief Resolves a config file path: explicit --config, then common default locations.
 * @param argc argv from main.
 * @param argv argv from main.
 * @param explicit_path_out Set when --config was passed on the command line.
 * @return Path to use, or empty if no config file should be loaded.
 */
std::string resolveConfigFilePath(int argc, char* argv[], bool& explicit_path_out);

/**
 * @brief Applies command-line flags on top of a loaded config (CLI wins over file).
 */
void applyCommandLineOverrides(AppConfig& config, int argc, char* argv[]);

/** @brief Builds a registry from parsed config scenarios. */
ScenarioRegistry makeScenarioRegistry(const AppConfig& config);

}  // namespace ccrrt
