#include "scenarios/scenarios.hpp"

#include "scenarios/paper_figures.hpp"
#include "scenarios/performance_scenarios.hpp"

namespace ccrrt {

namespace {

ScenarioEntry paperEntry(const std::string& name, const std::string& description, Environment env) {
    ScenarioEntry entry;
    entry.name = name;
    entry.description = description;
    entry.category = ScenarioCategory::Paper;
    entry.environment = std::move(env);
    return entry;
}

ScenarioEntry perfEntry(const std::string& name, const std::string& description, Environment env) {
    ScenarioEntry entry;
    entry.name = name;
    entry.description = description;
    entry.category = ScenarioCategory::Performance;
    entry.environment = std::move(env);
    return entry;
}

}  // namespace

std::vector<ScenarioEntry> paperScenarios() {
    return {
        paperEntry("figure5", "Paper Fig. 5: 2 agents, 4 static, 1 dynamic", makeFigure5Scenario()),
        paperEntry("figure6", "Paper Fig. 6: priority / time-separated crossing", makeFigure6Scenario()),
        paperEntry("figure7", "Paper Fig. 7: zig-zag wait for dynamic obstacle", makeFigure7Scenario()),
    };
}

std::vector<ScenarioEntry> performanceScenarios() {
    return {
        perfEntry(
            "perf_cluttered",
            "10 static obstacles; tests MC collision cost in dense maps",
            makePerfClutteredScenario()),
        perfEntry(
            "perf_four_agents",
            "4 agents + 5 static; tests priority-chain planning scale",
            makePerfFourAgentsScenario()),
        perfEntry(
            "perf_narrow_passage",
            "narrow gap at y=8; tests replanning in tight corridors",
            makePerfNarrowPassageScenario()),
        perfEntry(
            "perf_long_paths",
            "3 agents, long corner-to-corner paths; tests horizon length",
            makePerfLongPathsScenario()),
        perfEntry(
            "perf_multi_dynamic",
            "2 agents + 3 dynamic obstacles; tests moving-object predictions",
            makePerfMultiDynamicScenario()),
        perfEntry(
            "perf_stress",
            "3 agents, clutter + 3 dynamic; combined stress benchmark",
            makePerfStressScenario()),
    };
}

std::vector<ScenarioEntry> allScenarios() {
    auto scenarios = paperScenarios();
    const auto perf = performanceScenarios();
    scenarios.insert(scenarios.end(), perf.begin(), perf.end());
    return scenarios;
}

std::vector<ScenarioEntry> benchmarkScenarios() {
    return performanceScenarios();
}

}  // namespace ccrrt
