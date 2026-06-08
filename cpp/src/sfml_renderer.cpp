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

#include <SFML/Graphics.hpp>
#include <cmath>
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

}  // namespace

void SFMLRenderer::renderEnvironment(const Environment& environment) const {
    const float window_size = 800.0f;
    const float scale = window_size / static_cast<float>(environment.bounds_max - environment.bounds_min);
    const float offset = window_size * 0.95f;

    sf::RenderWindow window(
        sf::VideoMode(static_cast<unsigned>(window_size), static_cast<unsigned>(window_size)),
        "Multi-Agent CC-RRT");

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

        for (const auto& agent : environment.agents) {
            drawDisc(window, agent.start, 0.25, sf::Color(220, 20, 60), scale, offset);
            drawDisc(window, agent.goal, 0.25, sf::Color(34, 139, 34), scale, offset);
        }

        for (const auto& dynamic_obstacle : environment.dynamic_obstacles) {
            for (const auto& waypoint : dynamic_obstacle.waypoints) {
                drawDisc(window, waypoint, 0.2, sf::Color(148, 0, 211, 120), scale, offset);
            }
        }

        window.display();
    }
}

void SFMLRenderer::renderSimulationResult(
    const Environment& environment,
    const SimulationResult& result) const {
    const float window_size = 800.0f;
    const float scale = window_size / static_cast<float>(environment.bounds_max - environment.bounds_min);
    const float offset = window_size * 0.95f;

    sf::RenderWindow window(
        sf::VideoMode(static_cast<unsigned>(window_size), static_cast<unsigned>(window_size)),
        "Multi-Agent CC-RRT Result");

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
            for (const auto& waypoint : dynamic_obstacle.waypoints) {
                drawDisc(window, waypoint, 0.15, sf::Color(148, 0, 211, 100), scale, offset);
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
