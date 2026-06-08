#pragma once

#include "ccrrt/types.hpp"

#include <string>
#include <vector>

namespace ccrrt {

struct ScenarioEntry {
    std::string name;
    Environment environment;
};

std::vector<ScenarioEntry> allScenarios();
Environment makeFigure5Scenario();
Environment makeFigure6Scenario();
Environment makeFigure7Scenario();

}  // namespace ccrrt
