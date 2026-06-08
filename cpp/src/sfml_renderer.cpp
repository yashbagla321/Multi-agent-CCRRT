/**
 * @file sfml_renderer.cpp
 * @brief SFML-based 2D visualization of environments and executed trajectories.
 *
 * Only compiled when CCRRT_ENABLE_VISUALIZATION=ON and SFML is found.
 * Coordinate system: y-axis is flipped so mathematical y-up maps to screen coordinates.
 *
 * @see ccrrt/sfml_renderer.hpp
 */

#include "ccrrt/sfml_renderer.hpp"

#if CCRRT_HAS_SFML

#include "ccrrt/geometry.hpp"
#include "ccrrt/types.hpp"

#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace ccrrt {

namespace {

/** @brief Maps workspace coordinates to SFML screen pixels (y flipped). */
sf::Vector2f toScreen(const Vec2& point, float scale, float offset) {
    return {static_cast<float>(point.x * scale + offset),
            static_cast<float>(offset - point.y * scale)};
}

/** @brief Draws a filled circle at workspace position @p center. */
void drawDisc(
    sf::RenderWindow& window,
    const Vec2& center,
    double radius,
    const sf::Color& fill,
    float scale,
    float offset) {
    sf::CircleShape circle(static_cast<float>(radius * scale));
    circle.setFillColor(fill);
    circle.setOrigin(static_cast<float>(radius * scale), static_cast<float>(radius * scale));
    const sf::Vector2f screen = toScreen(center, scale, offset);
    circle.setPosition(screen);
    window.draw(circle);
}

/** @brief Draws a ring (outline only) at workspace position @p center. */
void drawRing(
    sf::RenderWindow& window,
    const Vec2& center,
    double radius,
    const sf::Color& outline,
    float scale,
    float offset,
    float thickness = 2.0f) {
    sf::CircleShape circle(static_cast<float>(radius * scale));
    circle.setFillColor(sf::Color::Transparent);
    circle.setOutlineColor(outline);
    circle.setOutlineThickness(thickness);
    circle.setOrigin(static_cast<float>(radius * scale), static_cast<float>(radius * scale));
    circle.setPosition(toScreen(center, scale, offset));
    window.draw(circle);
}

/** @brief Draws an alpha-confidence disc from isotropic variance. */
void drawConfidenceDisc(
    sf::RenderWindow& window,
    const Vec2& center,
    double variance,
    const sf::Color& fill,
    float scale,
    float offset) {
    const double radius = std::sqrt(variance * chiSquaredThreshold2D(0.99));
    drawDisc(window, center, radius, fill, scale, offset);
}

/** @brief Returns a distinct color per agent name for preview rendering. */
sf::Color agentColor(const std::string& name) {
    if (name == "red") {
        return sf::Color(220, 20, 60);
    }
    if (name == "blue") {
        return sf::Color(30, 144, 255);
    }
    if (name == "green") {
        return sf::Color(34, 139, 34);
    }
    return sf::Color(255, 140, 0);
}

/** @brief Connects workspace points with a colored polyline. */
void drawPolyline(
    sf::RenderWindow& window,
    const std::vector<Vec2>& points,
    const sf::Color& color,
    float scale,
    float offset) {
    if (points.size() < 2) {
        return;
    }
    sf::VertexArray line(sf::LineStrip, points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        line[i].position = toScreen(points[i], scale, offset);
        line[i].color = color;
    }
    window.draw(line);
}

/** @brief Prints a text legend to stdout when previewing a scenario. */
void printScenarioLegend(const Environment& environment, const std::string& scenario_name) {
    std::cout << "\n=== Scenario preview: " << (scenario_name.empty() ? "(unnamed)" : scenario_name)
              << " ===\n";
    std::cout << "  Bounds: [" << environment.bounds_min << ", " << environment.bounds_max << "]\n";
    std::cout << "  Static obstacles: " << environment.static_obstacles.size() << " (black discs)\n";
    for (const auto& agent : environment.agents) {
        std::cout << "  Agent \"" << agent.name << "\" (priority " << agent.priority << "): "
                  << "start (" << agent.start.x << ", " << agent.start.y << ") -> "
                  << "goal (" << agent.goal.x << ", " << agent.goal.y << ")\n";
    }
    for (const auto& dyn : environment.dynamic_obstacles) {
        std::cout << "  Dynamic obstacle " << dyn.id << ": " << dyn.waypoints.size()
                  << " waypoints (magenta trail)\n";
    }
    std::cout << "  Legend: filled circle = start, ring = goal, dashed line = start-to-goal hint\n";
    std::cout << "  Close the window to continue.\n\n";
}

}  // namespace

void SFMLRenderer::renderScenarioPreview(
    const Environment& environment,
    const std::string& scenario_name) const {
    printScenarioLegend(environment, scenario_name);

    const float window_size = 800.0f;
    const float scale = window_size / static_cast<float>(environment.bounds_max - environment.bounds_min);
    const float offset = window_size * 0.95f;

    std::string title = "Scenario Preview";
    if (!scenario_name.empty()) {
        title += " — " + scenario_name;
    }

    sf::RenderWindow window(
        sf::VideoMode(static_cast<unsigned>(window_size), static_cast<unsigned>(window_size)),
        title);

    while (window.isOpen()) {
        sf::Event event{};
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        window.clear(sf::Color(245, 245, 245));

        // Workspace border.
        const float span = static_cast<float>(environment.bounds_max - environment.bounds_min);
        sf::RectangleShape border({span * scale, span * scale});
        border.setPosition(toScreen({environment.bounds_min, environment.bounds_min}, scale, offset));
        border.setFillColor(sf::Color::Transparent);
        border.setOutlineColor(sf::Color(180, 180, 180));
        border.setOutlineThickness(1.0f);
        window.draw(border);

        // Static obstacles.
        for (const auto& obstacle : environment.static_obstacles) {
            drawDisc(window, obstacle.center, obstacle.radius, sf::Color(20, 20, 20), scale, offset);
        }

        // Dynamic obstacle mean paths with initial uncertainty.
        for (const auto& dynamic_obstacle : environment.dynamic_obstacles) {
            drawPolyline(window, dynamic_obstacle.waypoints, sf::Color(148, 0, 211), scale, offset);
            for (const auto& waypoint : dynamic_obstacle.waypoints) {
                drawDisc(window, waypoint, 0.12, sf::Color(148, 0, 211, 160), scale, offset);
            }
            if (!dynamic_obstacle.waypoints.empty()) {
                drawConfidenceDisc(
                    window,
                    dynamic_obstacle.waypoints.front(),
                    dynamic_obstacle.initial_variance,
                    sf::Color(148, 0, 211, 50),
                    scale,
                    offset);
            }
        }

        // Agents: start (filled), goal (ring), intended direction (line).
        for (const auto& agent : environment.agents) {
            const sf::Color color = agentColor(agent.name);
            drawPolyline(window, {agent.start, agent.goal}, sf::Color(color.r, color.g, color.b, 100), scale, offset);
            drawDisc(window, agent.start, 0.3, color, scale, offset);
            drawRing(window, agent.goal, 0.3, color, scale, offset);
            drawConfidenceDisc(
                window,
                agent.start,
                0.2,
                sf::Color(color.r, color.g, color.b, 40),
                scale,
                offset);
        }

        window.display();
    }
}

void SFMLRenderer::renderEnvironment(const Environment& environment) const {
    renderScenarioPreview(environment, "");
}

void SFMLRenderer::renderSimulationResult(
    const Environment& environment,
    const SimulationResult& result) const {
    const float window_size = 800.0f;
    const float scale = window_size / static_cast<float>(environment.bounds_max - environment.bounds_min);
    const float offset = window_size * 0.95f;

    std::string title = "Simulation Result";
    if (!result.scenario_name.empty()) {
        title += " — " + result.scenario_name;
    }

    sf::RenderWindow window(
        sf::VideoMode(static_cast<unsigned>(window_size), static_cast<unsigned>(window_size)),
        title);

    const std::vector<sf::Color> path_colors = {
        sf::Color(30, 144, 255),
        sf::Color(50, 205, 50),
        sf::Color(255, 140, 0),
    };

    while (window.isOpen()) {
        sf::Event event{};
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        window.clear(sf::Color(245, 245, 245));

        for (const auto& obstacle : environment.static_obstacles) {
            drawDisc(window, obstacle.center, obstacle.radius, sf::Color(20, 20, 20), scale, offset);
        }

        for (const auto& dynamic_obstacle : environment.dynamic_obstacles) {
            drawPolyline(window, dynamic_obstacle.waypoints, sf::Color(148, 0, 211, 120), scale, offset);
            for (const auto& waypoint : dynamic_obstacle.waypoints) {
                drawDisc(window, waypoint, 0.12, sf::Color(148, 0, 211, 100), scale, offset);
            }
        }

        for (std::size_t i = 0; i < result.agent_paths.size(); ++i) {
            const sf::Color color = path_colors[i % path_colors.size()];
            const auto& path = result.agent_paths[i];
            if (path.size() < 2) {
                continue;
            }

            sf::VertexArray lines(sf::LineStrip, path.size());
            for (std::size_t j = 0; j < path.size(); ++j) {
                const sf::Vector2f screen = toScreen(path[j].position, scale, offset);
                lines[j].position = screen;
                lines[j].color = color;
            }
            window.draw(lines);

            for (const auto& step : path) {
                drawConfidenceDisc(
                    window,
                    step.position,
                    step.variance,
                    sf::Color(color.r, color.g, color.b, 40),
                    scale,
                    offset);
            }
        }

        window.display();
    }
}

}  // namespace ccrrt

#endif
