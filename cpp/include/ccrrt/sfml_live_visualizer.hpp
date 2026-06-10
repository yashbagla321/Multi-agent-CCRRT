/**
 * @file sfml_live_visualizer.hpp
 * @brief Step-by-step SFML visualization during simulation.
 */

#pragma once

#include "ccrrt/simulation_observer.hpp"

#include <string>

namespace ccrrt {

#if CCRRT_HAS_SFML

/**
 * @brief Live SFML window that updates after each simulation timestep.
 *
 * Shows executed paths (solid), current planned trajectories (faint), agent
 * positions with confidence discs, and dynamic obstacle positions.
 *
 * Controls: Space = pause/resume, N = next step (while paused), close window = abort.
 */
class SFMLLiveVisualizer final : public ISimulationObserver {
public:
    SFMLLiveVisualizer(std::string scenario_name, int step_delay_ms = 150);
    ~SFMLLiveVisualizer() override;

    bool onFrame(const SimulationFrame& frame) override;

private:
    std::string scenario_name_;
    int step_delay_ms_;
    bool paused_ = false;
    bool step_once_ = false;
    class Impl;
    Impl* impl_ = nullptr;
};

#endif

}  // namespace ccrrt
