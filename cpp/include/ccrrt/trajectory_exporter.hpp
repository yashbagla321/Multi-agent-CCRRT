#pragma once

#include "ccrrt/types.hpp"

#include <string>

namespace ccrrt {

class TrajectoryExporter {
public:
    bool exportCsv(const SimulationResult& result, const std::string& output_directory) const;
    bool exportSummaryJson(const SimulationResult& result, const std::string& output_directory) const;
};

}  // namespace ccrrt
