#pragma once

#include "ccrrt/types.hpp"

namespace ccrrt {

#if CCRRT_HAS_SFML

class SFMLRenderer {
public:
    void renderEnvironment(const Environment& environment) const;
    void renderSimulationResult(const Environment& environment, const SimulationResult& result) const;
};

#endif

}  // namespace ccrrt
