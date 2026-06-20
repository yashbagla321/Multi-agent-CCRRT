#include "ccrrt/multi_agent_planner.hpp"
#include "ccrrt/runtime_config.hpp"

#include <gtest/gtest.h>

#include <filesystem>

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
    EXPECT_FALSE(config->planner.enable_path_smoothing);
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
