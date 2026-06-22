#include "ccrrt/multi_agent_planner.hpp"
#include "ccrrt/runtime_config.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <iterator>
#include <string>
#include <vector>

#ifndef CCRRT_TEST_CONFIG_DIR
#define CCRRT_TEST_CONFIG_DIR "config"
#endif

namespace {

std::string configPath(const char* filename) {
    return (std::filesystem::path(CCRRT_TEST_CONFIG_DIR) / filename).string();
}

}  // namespace

TEST(RuntimeConfig, LoadsProjectConfigFile) {
    const std::string path = configPath("ccrrt.json");
    ASSERT_TRUE(std::filesystem::exists(path)) << "Missing " << path;

    const auto config = ccrrt::loadAppConfigFromFile(path);
    ASSERT_TRUE(config.has_value());

    EXPECT_DOUBLE_EQ(config->planner.expand_distance, 0.5);
    EXPECT_DOUBLE_EQ(config->planner.motion_step, 0.2);
    EXPECT_TRUE(config->planner.enable_path_smoothing);
    EXPECT_DOUBLE_EQ(config->planner.collision_bound_M, 0.2);
    EXPECT_DOUBLE_EQ(config->planner.initial_variance, 0.2);
    EXPECT_DOUBLE_EQ(config->planner.process_noise, 0.2);
    EXPECT_DOUBLE_EQ(config->planner.measurement_noise, 0.2);
    EXPECT_DOUBLE_EQ(config->planner.max_prediction_variance, 0.8);
    EXPECT_EQ(config->run.scenario, "figure5");
    EXPECT_FALSE(config->scenarios.empty());
}

TEST(RuntimeConfig, ScenarioRegistryFindsFigure5) {
    const std::string path = configPath("ccrrt.json");
    const auto config = ccrrt::loadAppConfigFromFile(path);
    ASSERT_TRUE(config.has_value());

    const ccrrt::ScenarioRegistry registry = ccrrt::makeScenarioRegistry(*config);
    const auto scenario = registry.find("figure5");
    ASSERT_TRUE(scenario.has_value());
    EXPECT_EQ(scenario->name, "figure5");
    EXPECT_EQ(scenario->environment.agents.size(), 2u);
    EXPECT_EQ(scenario->environment.static_obstacles.size(), 4u);
    EXPECT_FALSE(scenario->environment.dynamic_obstacles.empty());
}

TEST(RuntimeConfig, PerformanceScenariosPresent) {
    const std::string path = configPath("ccrrt.json");
    const auto config = ccrrt::loadAppConfigFromFile(path);
    ASSERT_TRUE(config.has_value());

    const ccrrt::ScenarioRegistry registry = ccrrt::makeScenarioRegistry(*config);
    const auto perf = registry.performanceScenarios();
    EXPECT_GE(perf.size(), 1u);

    bool found_cluttered = false;
    for (const auto& entry : perf) {
        if (entry.name == "perf_cluttered") {
            found_cluttered = true;
            EXPECT_GE(entry.environment.static_obstacles.size(), 5u);
        }
    }
    EXPECT_TRUE(found_cluttered);
}

TEST(RuntimeConfig, ScenarioAgentNamesFollowVisualizationColorOrder) {
    const std::string path = configPath("ccrrt.json");
    const auto config = ccrrt::loadAppConfigFromFile(path);
    ASSERT_TRUE(config.has_value());

    const ccrrt::ScenarioRegistry registry = ccrrt::makeScenarioRegistry(*config);
    const std::vector<std::string> expected_names = {"red", "blue", "green", "orange"};

    for (const auto& entry : registry.all()) {
        ASSERT_LE(entry.environment.agents.size(), expected_names.size()) << entry.name;
        for (std::size_t i = 0; i < entry.environment.agents.size(); ++i) {
            EXPECT_EQ(entry.environment.agents[i].name, expected_names[i]) << entry.name;
        }
    }
}

TEST(RuntimeConfig, AppliesCommandLineOverrides) {
    ccrrt::AppConfig config;
    config.planner.max_iterations = 1234;

    const char* args[] = {
        "multi_agent_ccrrt",
        "--scenario",
        "figure6",
        "--output",
        "out/run",
        "--seed",
        "7",
        "--mc-samples",
        "321",
        "--path-smoothing",
        "--list-scenarios",
        "--benchmark-all",
        "--run-all",
        "--run-all-normal",
        "--run-all-smooth",
    };
    char** argv = const_cast<char**>(args);

    ccrrt::applyCommandLineOverrides(config, static_cast<int>(std::size(args)), argv);

    EXPECT_EQ(config.run.scenario, "figure6");
    EXPECT_EQ(config.run.output_directory, "out/run");
    EXPECT_EQ(config.planner.rng_seed, 7u);
    EXPECT_EQ(config.planner.mc_samples, 321);
    EXPECT_TRUE(config.planner.enable_path_smoothing);
    EXPECT_TRUE(config.run.list_scenarios);
    EXPECT_TRUE(config.run.benchmark_all);
    EXPECT_TRUE(config.run.run_all);
    EXPECT_TRUE(config.run.run_all_normal);
    EXPECT_TRUE(config.run.run_all_smooth);
    EXPECT_EQ(config.planner.max_iterations, 1234);
}

TEST(RuntimeConfig, PythonCompatFlagIsOrderIndependentBaseProfile) {
    ccrrt::AppConfig config;
    config.planner.max_iterations = 1234;
    config.planner.enable_path_smoothing = false;

    const char* args[] = {
        "multi_agent_ccrrt",
        "--seed",
        "99",
        "--mc-samples",
        "444",
        "--path-smoothing",
        "--python-compat",
    };
    char** argv = const_cast<char**>(args);

    ccrrt::applyCommandLineOverrides(config, static_cast<int>(std::size(args)), argv);

    EXPECT_TRUE(config.run.python_compat);
    EXPECT_TRUE(config.planner.use_legacy_collision);
    EXPECT_DOUBLE_EQ(config.planner.expand_distance, 1.0);
    EXPECT_EQ(config.planner.max_iterations, 500);
    EXPECT_EQ(config.planner.rng_seed, 99u);
    EXPECT_EQ(config.planner.mc_samples, 444);
    EXPECT_TRUE(config.planner.enable_path_smoothing);
}

TEST(RuntimeConfig, NoPathSmoothingFlagDisablesSmoothing) {
    ccrrt::AppConfig config;
    config.planner.enable_path_smoothing = true;

    const char* args[] = {
        "multi_agent_ccrrt",
        "--no-path-smoothing",
    };
    char** argv = const_cast<char**>(args);

    ccrrt::applyCommandLineOverrides(config, static_cast<int>(std::size(args)), argv);

    EXPECT_FALSE(config.planner.enable_path_smoothing);
}

TEST(RuntimeConfig, ResolvesExplicitConfigPath) {
    const char* args[] = {
        "multi_agent_ccrrt",
        "--config",
        "cpp/config/ccrrt.json",
    };
    char** argv = const_cast<char**>(args);
    bool explicit_path = false;

    const auto path =
        ccrrt::resolveConfigFilePath(static_cast<int>(std::size(args)), argv, explicit_path);

    EXPECT_TRUE(explicit_path);
    EXPECT_EQ(path, "cpp/config/ccrrt.json");
}

TEST(RuntimeConfig, IntegrationFigure5CompletesWithFastPlanner) {
    const std::string path = configPath("ccrrt.json");
    const auto loaded = ccrrt::loadAppConfigFromFile(path);
    ASSERT_TRUE(loaded.has_value());

    const ccrrt::ScenarioRegistry registry = ccrrt::makeScenarioRegistry(*loaded);
    const auto scenario = registry.find("figure5");
    ASSERT_TRUE(scenario.has_value());

    ccrrt::PlannerConfig planner = loaded->planner;
    planner.mc_samples = 150;
    planner.max_iterations = 8000;
    planner.max_timesteps = 400;
    planner.rng_seed = 42;

    ccrrt::MultiAgentPlanner multi_planner(planner);
    const auto result = multi_planner.run(scenario->environment, scenario->name);

    EXPECT_TRUE(result.success);
    EXPECT_GT(result.total_steps, 0);
}
