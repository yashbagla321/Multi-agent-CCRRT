/**
 * @file runtime_config.cpp
 * @brief JSON runtime configuration loader.
 */

#include "ccrrt/runtime_config.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

namespace ccrrt {

namespace {

using json = nlohmann::json;

Vec2 parseVec2(const json& value) {
    if (!value.is_array() || value.size() < 2) {
        throw std::runtime_error("expected [x, y] array");
    }
    return {value[0].get<double>(), value[1].get<double>()};
}

void parsePlannerConfig(const json& root, PlannerConfig& config) {
    if (!root.is_object()) {
        return;
    }

    if (root.contains("expand_distance")) {
        config.expand_distance = root["expand_distance"].get<double>();
    }
    if (root.contains("collision_bound_M")) {
        config.collision_bound_M = root["collision_bound_M"].get<double>();
    }
    if (root.contains("confidence_alpha")) {
        config.confidence_alpha = root["confidence_alpha"].get<double>();
    }
    if (root.contains("mc_samples")) {
        config.mc_samples = root["mc_samples"].get<int>();
    }
    if (root.contains("max_iterations")) {
        config.max_iterations = root["max_iterations"].get<int>();
    }
    if (root.contains("max_timesteps")) {
        config.max_timesteps = root["max_timesteps"].get<int>();
    }
    if (root.contains("goal_sample_rate")) {
        config.goal_sample_rate = root["goal_sample_rate"].get<int>();
    }
    if (root.contains("initial_variance")) {
        config.initial_variance = root["initial_variance"].get<double>();
    }
    if (root.contains("process_noise")) {
        config.process_noise = root["process_noise"].get<double>();
    }
    if (root.contains("max_prediction_variance")) {
        config.max_prediction_variance = root["max_prediction_variance"].get<double>();
    }
    if (root.contains("measurement_noise")) {
        config.measurement_noise = root["measurement_noise"].get<double>();
    }
    if (root.contains("bounds_min")) {
        config.bounds_min = root["bounds_min"].get<double>();
    }
    if (root.contains("bounds_max")) {
        config.bounds_max = root["bounds_max"].get<double>();
    }
    if (root.contains("rng_seed")) {
        config.rng_seed = root["rng_seed"].get<unsigned int>();
    }
    if (root.contains("use_legacy_collision")) {
        config.use_legacy_collision = root["use_legacy_collision"].get<bool>();
    }
    if (root.contains("legacy_p_safe")) {
        config.legacy_p_safe = root["legacy_p_safe"].get<double>();
    }
    if (root.contains("variance_growth_alpha")) {
        config.variance_growth_alpha = root["variance_growth_alpha"].get<double>();
    }
}

void parseRunSettings(const json& root, RunSettings& run) {
    if (!root.is_object()) {
        return;
    }

    if (root.contains("scenario")) {
        run.scenario = root["scenario"].get<std::string>();
    }
    if (root.contains("output_directory")) {
        run.output_directory = root["output_directory"].get<std::string>();
    }
    if (root.contains("enable_visualization")) {
        run.enable_visualization = root["enable_visualization"].get<bool>();
    }
    if (root.contains("live_visualization")) {
        run.live_visualization = root["live_visualization"].get<bool>();
    }
    if (root.contains("viz_step_delay_ms")) {
        run.viz_step_delay_ms = root["viz_step_delay_ms"].get<int>();
    }
    if (root.contains("preview_only")) {
        run.preview_only = root["preview_only"].get<bool>();
    }
    if (root.contains("preview_all")) {
        run.preview_all = root["preview_all"].get<bool>();
    }
    if (root.contains("list_scenarios")) {
        run.list_scenarios = root["list_scenarios"].get<bool>();
    }
    if (root.contains("benchmark_all")) {
        run.benchmark_all = root["benchmark_all"].get<bool>();
    }
    if (root.contains("python_compat")) {
        run.python_compat = root["python_compat"].get<bool>();
    }
}

std::vector<Vec2> generateLineWaypoints(Vec2 from, Vec2 to, double step) {
    std::vector<Vec2> waypoints;
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double length = std::sqrt(dx * dx + dy * dy);
    if (length <= 1e-9) {
        waypoints.push_back(from);
        return waypoints;
    }
    const int count = static_cast<int>(std::ceil(length / step));
    for (int i = 0; i <= count; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(count);
        waypoints.push_back({from.x + t * dx, from.y + t * dy});
    }
    return waypoints;
}

std::vector<Vec2> generateVerticalPath(double x, double y_start, double y_end, double step) {
    return generateLineWaypoints({x, y_start}, {x, y_end}, step);
}

std::vector<Vec2> generateHorizontalPath(double y, double x_start, double x_end, double step) {
    return generateLineWaypoints({x_start, y}, {x_end, y}, step);
}

std::vector<Vec2> parseDynamicPath(const json& path_spec) {
    const std::string type = path_spec.value("type", "");
    if (type == "vertical") {
        return generateVerticalPath(
            path_spec.at("x").get<double>(),
            path_spec.at("y_start").get<double>(),
            path_spec.at("y_end").get<double>(),
            path_spec.at("step").get<double>());
    }
    if (type == "horizontal") {
        return generateHorizontalPath(
            path_spec.at("y").get<double>(),
            path_spec.at("x_start").get<double>(),
            path_spec.at("x_end").get<double>(),
            path_spec.at("step").get<double>());
    }
    if (type == "line") {
        return generateLineWaypoints(
            parseVec2(path_spec.at("from")),
            parseVec2(path_spec.at("to")),
            path_spec.at("step").get<double>());
    }
    throw std::runtime_error("dynamic obstacle path type must be vertical, horizontal, or line");
}

ScenarioCategory parseScenarioCategory(const json& root) {
    const std::string category = root.value("category", "paper");
    if (category == "performance" || category == "perf") {
        return ScenarioCategory::Performance;
    }
    return ScenarioCategory::Paper;
}

Environment parseEnvironment(const json& root) {
    Environment env;

    if (root.contains("bounds") && root["bounds"].is_object()) {
        const auto& bounds = root["bounds"];
        env.bounds_min = bounds.value("min", env.bounds_min);
        env.bounds_max = bounds.value("max", env.bounds_max);
    }

    if (root.contains("static_obstacles") && root["static_obstacles"].is_array()) {
        for (const auto& item : root["static_obstacles"]) {
            StaticObstacle obstacle;
            if (item.contains("center")) {
                obstacle.center = parseVec2(item["center"]);
            }
            if (item.contains("radius")) {
                obstacle.radius = item["radius"].get<double>();
            }
            env.static_obstacles.push_back(obstacle);
        }
    }

    if (root.contains("agents") && root["agents"].is_array()) {
        for (const auto& item : root["agents"]) {
            AgentSpec agent;
            agent.id = item.value("id", 0);
            agent.priority = item.value("priority", 0);
            agent.name = item.value("name", "agent");
            if (item.contains("start")) {
                agent.start = parseVec2(item["start"]);
            }
            if (item.contains("goal")) {
                agent.goal = parseVec2(item["goal"]);
            }
            env.agents.push_back(agent);
        }
    }

    if (root.contains("dynamic_obstacles") && root["dynamic_obstacles"].is_array()) {
        for (const auto& item : root["dynamic_obstacles"]) {
            DynamicObstacleSpec obstacle;
            obstacle.id = item.value("id", 0);
            obstacle.initial_variance = item.value("initial_variance", 0.2);

            if (item.contains("path") && item["path"].is_object()) {
                obstacle.waypoints = parseDynamicPath(item["path"]);
            } else if (item.contains("waypoints") && item["waypoints"].is_array()) {
                for (const auto& waypoint : item["waypoints"]) {
                    obstacle.waypoints.push_back(parseVec2(waypoint));
                }
            }

            if (item.contains("variance_per_waypoint") && item["variance_per_waypoint"].is_array()) {
                for (const auto& variance : item["variance_per_waypoint"]) {
                    obstacle.variance_per_waypoint.push_back(variance.get<double>());
                }
            }

            env.dynamic_obstacles.push_back(obstacle);
        }
    }

    return env;
}

ConfigScenario parseConfigScenario(const std::string& name, const json& root) {
    ConfigScenario scenario;
    scenario.name = name;
    scenario.description = root.value("description", "");
    scenario.category = parseScenarioCategory(root);
    scenario.environment = parseEnvironment(root);
    return scenario;
}

void mergeScenario(std::vector<ConfigScenario>& scenarios, ConfigScenario scenario) {
    for (auto& existing : scenarios) {
        if (existing.name == scenario.name) {
            existing = std::move(scenario);
            return;
        }
    }
    scenarios.push_back(std::move(scenario));
}

void parseScenariosObject(
    const json& scenarios_object,
    std::vector<ConfigScenario>& out,
    bool merge_existing) {
    if (!scenarios_object.is_object()) {
        return;
    }
    for (auto it = scenarios_object.begin(); it != scenarios_object.end(); ++it) {
        const std::string scenario_name = it.key();
        if (!scenario_name.empty() && scenario_name.front() == '_') {
            continue;
        }
        ConfigScenario scenario = parseConfigScenario(scenario_name, it.value());
        if (merge_existing) {
            mergeScenario(out, std::move(scenario));
        } else {
            out.push_back(std::move(scenario));
        }
    }
}

std::string resolveSiblingPath(const std::string& config_path, const std::string& relative_path) {
    const std::filesystem::path base = std::filesystem::path(config_path).parent_path();
    return (base / relative_path).lexically_normal().string();
}

bool fileExists(const std::string& path) {
    return !path.empty() && std::filesystem::exists(path);
}

}  // namespace

std::optional<AppConfig> loadAppConfigFromFile(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Failed to open config file: " << path << '\n';
        return std::nullopt;
    }

    json document;
    try {
        file >> document;
    } catch (const json::parse_error& error) {
        std::cerr << "JSON parse error in " << path << ": " << error.what() << '\n';
        return std::nullopt;
    }

    AppConfig app;
    app.run.loaded_config_path = path;

    try {
        if (document.contains("run")) {
            parseRunSettings(document["run"], app.run);
        }

        if (app.run.python_compat) {
            app.planner = pythonCompatPlannerConfig();
        }

        if (document.contains("planner")) {
            parsePlannerConfig(document["planner"], app.planner);
        }

        if (document.contains("scenarios_file")) {
            const std::string scenarios_path =
                resolveSiblingPath(path, document["scenarios_file"].get<std::string>());
            std::ifstream scenarios_file(scenarios_path);
            if (!scenarios_file.is_open()) {
                throw std::runtime_error("failed to open scenarios_file: " + scenarios_path);
            }
            json scenarios_document;
            scenarios_file >> scenarios_document;
            if (scenarios_document.is_object()) {
                parseScenariosObject(scenarios_document, app.scenarios, false);
            } else if (scenarios_document.contains("scenarios")) {
                parseScenariosObject(scenarios_document["scenarios"], app.scenarios, false);
            }
        }

        if (document.contains("scenarios") && document["scenarios"].is_object()) {
            parseScenariosObject(document["scenarios"], app.scenarios, true);
        }
    } catch (const std::exception& error) {
        std::cerr << "Invalid config in " << path << ": " << error.what() << '\n';
        return std::nullopt;
    }

    return app;
}

std::string resolveConfigFilePath(int argc, char* argv[], bool& explicit_path_out) {
    explicit_path_out = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            explicit_path_out = true;
            return argv[i + 1];
        }
    }

    const std::vector<std::string> candidates = {
        "ccrrt.json",
        "config/ccrrt.json",
    };

    for (const auto& candidate : candidates) {
        if (fileExists(candidate)) {
            return candidate;
        }
    }

    if (argc > 0) {
        const std::filesystem::path exe_path = std::filesystem::path(argv[0]).parent_path();
        const std::vector<std::filesystem::path> exe_candidates = {
            exe_path / "ccrrt.json",
            exe_path / "config" / "ccrrt.json",
        };
        for (const auto& candidate : exe_candidates) {
            if (std::filesystem::exists(candidate)) {
                return candidate.string();
            }
        }
    }

    return {};
}

void applyCommandLineOverrides(AppConfig& config, int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--scenario" && i + 1 < argc) {
            config.run.scenario = argv[++i];
        } else if (arg == "--preview") {
            config.run.preview_only = true;
        } else if (arg == "--preview-all") {
            config.run.preview_all = true;
        } else if (arg == "--list-scenarios") {
            config.run.list_scenarios = true;
        } else if (arg == "--benchmark-all") {
            config.run.benchmark_all = true;
            config.run.enable_visualization = false;
        } else if (arg == "--no-viz") {
            config.run.enable_visualization = false;
        } else if (arg == "--no-live-viz") {
            config.run.live_visualization = false;
        } else if (arg == "--viz-delay-ms" && i + 1 < argc) {
            config.run.viz_step_delay_ms = std::stoi(argv[++i]);
        } else if (arg == "--output" && i + 1 < argc) {
            config.run.output_directory = argv[++i];
        } else if (arg == "--seed" && i + 1 < argc) {
            config.planner.rng_seed = static_cast<unsigned int>(std::stoul(argv[++i]));
        } else if (arg == "--mc-samples" && i + 1 < argc) {
            config.planner.mc_samples = std::stoi(argv[++i]);
        } else if (arg == "--python-compat") {
            config.planner = pythonCompatPlannerConfig();
            config.run.python_compat = true;
        } else if (arg == "--config" && i + 1 < argc) {
            ++i;
        }
    }
}

namespace {

ScenarioEntry toScenarioEntry(const ConfigScenario& scenario) {
    ScenarioEntry entry;
    entry.name = scenario.name;
    entry.description = scenario.description;
    entry.category = scenario.category;
    entry.environment = scenario.environment;
    return entry;
}

}  // namespace

ScenarioRegistry::ScenarioRegistry(std::vector<ConfigScenario> scenarios)
    : scenarios_(std::move(scenarios)) {}

std::optional<ScenarioEntry> ScenarioRegistry::find(const std::string& name) const {
    for (const auto& scenario : scenarios_) {
        if (scenario.name == name) {
            return toScenarioEntry(scenario);
        }
    }
    return std::nullopt;
}

std::vector<ScenarioEntry> ScenarioRegistry::all() const {
    std::vector<ScenarioEntry> entries;
    entries.reserve(scenarios_.size());
    for (const auto& scenario : scenarios_) {
        entries.push_back(toScenarioEntry(scenario));
    }
    return entries;
}

std::vector<ScenarioEntry> ScenarioRegistry::paperScenarios() const {
    std::vector<ScenarioEntry> entries;
    for (const auto& scenario : scenarios_) {
        if (scenario.category == ScenarioCategory::Paper) {
            entries.push_back(toScenarioEntry(scenario));
        }
    }
    return entries;
}

std::vector<ScenarioEntry> ScenarioRegistry::performanceScenarios() const {
    std::vector<ScenarioEntry> entries;
    for (const auto& scenario : scenarios_) {
        if (scenario.category == ScenarioCategory::Performance) {
            entries.push_back(toScenarioEntry(scenario));
        }
    }
    return entries;
}

std::vector<ScenarioEntry> ScenarioRegistry::benchmarkScenarios() const {
    return performanceScenarios();
}

ScenarioRegistry makeScenarioRegistry(const AppConfig& config) {
    return ScenarioRegistry(config.scenarios);
}

}  // namespace ccrrt
