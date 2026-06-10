/**
 * @file sfml_live_visualizer.cpp
 * @brief Step-by-step SFML visualization during simulation.
 */

#include "ccrrt/sfml_live_visualizer.hpp"

#if CCRRT_HAS_SFML

#include "ccrrt/geometry.hpp"

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace ccrrt {

namespace {

struct ViewTransform {
    float scale = 1.0f;
    float margin = 0.0f;
    double bounds_min = 0.0;
    float window_size = 0.0f;

    static ViewTransform fromEnvironment(const Environment& environment, float window_size) {
        const float margin = window_size * 0.05f;
        const float span = static_cast<float>(environment.bounds_max - environment.bounds_min);
        return {(window_size - 2.0f * margin) / span, margin, environment.bounds_min, window_size};
    }

    sf::Vector2f toScreen(const Vec2& point) const {
        const float x = margin + static_cast<float>((point.x - bounds_min) * scale);
        const float y = window_size - margin - static_cast<float>((point.y - bounds_min) * scale);
        return {x, y};
    }
};

void drawDisc(
    sf::RenderWindow& window,
    const Vec2& center,
    double radius,
    const sf::Color& fill,
    const ViewTransform& view) {
    sf::CircleShape circle(static_cast<float>(radius * view.scale));
    circle.setFillColor(fill);
    circle.setOrigin(static_cast<float>(radius * view.scale), static_cast<float>(radius * view.scale));
    circle.setPosition(view.toScreen(center));
    window.draw(circle);
}

void drawRing(
    sf::RenderWindow& window,
    const Vec2& center,
    double radius,
    const sf::Color& outline,
    const ViewTransform& view,
    float thickness = 2.0f) {
    sf::CircleShape circle(static_cast<float>(radius * view.scale));
    circle.setFillColor(sf::Color::Transparent);
    circle.setOutlineColor(outline);
    circle.setOutlineThickness(thickness);
    circle.setOrigin(static_cast<float>(radius * view.scale), static_cast<float>(radius * view.scale));
    circle.setPosition(view.toScreen(center));
    window.draw(circle);
}

void drawConfidenceDisc(
    sf::RenderWindow& window,
    const Vec2& center,
    double variance,
    const sf::Color& fill,
    const ViewTransform& view) {
    const double radius = std::sqrt(variance * chiSquaredThreshold2D(0.99));
    drawDisc(window, center, radius, fill, view);
}

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

void drawPolyline(
    sf::RenderWindow& window,
    const std::vector<Vec2>& points,
    const sf::Color& color,
    const ViewTransform& view) {
    if (points.size() < 2) {
        return;
    }
    sf::VertexArray line(sf::LineStrip, points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        line[i].position = view.toScreen(points[i]);
        line[i].color = color;
    }
    window.draw(line);
}

std::vector<Vec2> trajectoryPositions(const Trajectory& trajectory) {
    std::vector<Vec2> points;
    points.reserve(trajectory.nodes.size());
    for (const auto& node : trajectory.nodes) {
        points.push_back(node.position);
    }
    return points;
}

std::vector<Vec2> predictionPositions(const TrajectoryPrediction& prediction) {
    std::vector<Vec2> points;
    points.reserve(prediction.nodes.size());
    for (const auto& node : prediction.nodes) {
        points.push_back(node.position);
    }
    return points;
}

std::vector<Vec2> executedPositions(const std::vector<ExecutedStep>& executed) {
    std::vector<Vec2> points;
    points.reserve(executed.size());
    for (const auto& step : executed) {
        points.push_back(step.position);
    }
    return points;
}

std::string makeWindowTitle(const SimulationFrame& frame) {
    std::ostringstream title;
    title << "Multi-Agent CC-RRT";
    if (!frame.scenario_name.empty()) {
        title << " — " << frame.scenario_name;
    }
    if (frame.initial_plan_ready) {
        title << " — initial plan";
    } else {
        title << " — t=" << frame.timestep;
    }
    if (frame.simulation_complete) {
        title << " — complete";
    }
    return title.str();
}

}  // namespace

class SFMLLiveVisualizer::Impl {
public:
    std::unique_ptr<sf::RenderWindow> window;
    ViewTransform view{};
    bool printed_controls = false;
};

SFMLLiveVisualizer::SFMLLiveVisualizer(std::string scenario_name, int step_delay_ms)
    : scenario_name_(std::move(scenario_name)), step_delay_ms_(step_delay_ms), impl_(new Impl()) {}

SFMLLiveVisualizer::~SFMLLiveVisualizer() {
    delete impl_;
}

bool SFMLLiveVisualizer::onFrame(const SimulationFrame& frame) {
    if (!impl_->printed_controls) {
        std::cout << "\nLive visualization: Space = pause/resume, N = next step (while paused), "
                     "close window = stop simulation\n\n";
        impl_->printed_controls = true;
    }

    const float window_size = 800.0f;
    if (impl_->window == nullptr) {
        impl_->view.window_size = window_size;
        std::string title = "Multi-Agent CC-RRT";
        if (!scenario_name_.empty()) {
            title += " — " + scenario_name_;
        }
        impl_->window = std::make_unique<sf::RenderWindow>(
            sf::VideoMode(static_cast<unsigned>(window_size), static_cast<unsigned>(window_size)),
            title);
    }

    auto& window = *impl_->window;
    const Environment& environment = frame.environment;
    impl_->view = ViewTransform::fromEnvironment(environment, impl_->view.window_size);

    window.clear(sf::Color(245, 245, 245));

    const float span = static_cast<float>(environment.bounds_max - environment.bounds_min);
    sf::RectangleShape border({span * impl_->view.scale, span * impl_->view.scale});
    border.setPosition(impl_->view.toScreen({environment.bounds_min, environment.bounds_max}));
    border.setFillColor(sf::Color::Transparent);
    border.setOutlineColor(sf::Color(180, 180, 180));
    border.setOutlineThickness(1.0f);
    window.draw(border);

    for (const auto& obstacle : environment.static_obstacles) {
        drawDisc(window, obstacle.center, obstacle.radius, sf::Color(20, 20, 20), impl_->view);
    }

    for (const auto& prediction : frame.dynamic_predictions) {
        const auto points = predictionPositions(prediction);
        drawPolyline(window, points, sf::Color(148, 0, 211, 100), impl_->view);
        if (!prediction.nodes.empty()) {
            drawDisc(
                window,
                prediction.nodes.front().position,
                0.15,
                sf::Color(148, 0, 211, 200),
                impl_->view);
            drawConfidenceDisc(
                window,
                prediction.nodes.front().position,
                prediction.nodes.front().variance,
                sf::Color(148, 0, 211, 40),
                impl_->view);
        }
    }

    for (const auto& agent : frame.agents) {
        const sf::Color color = agentColor(agent.spec.name);
        drawRing(window, agent.spec.goal, 0.28, sf::Color(color.r, color.g, color.b, 120), impl_->view);

        const auto planned_points = trajectoryPositions(agent.planned);
        drawPolyline(
            window,
            planned_points,
            sf::Color(color.r, color.g, color.b, 70),
            impl_->view);

        const auto executed_points = executedPositions(agent.executed);
        drawPolyline(window, executed_points, color, impl_->view);

        for (const auto& step : agent.executed) {
            drawConfidenceDisc(
                window,
                step.position,
                step.variance,
                sf::Color(color.r, color.g, color.b, 35),
                impl_->view);
        }

        drawConfidenceDisc(
            window,
            agent.position,
            agent.variance,
            sf::Color(color.r, color.g, color.b, 55),
            impl_->view);

        const double body_radius = agent.replanned_this_step ? 0.38 : 0.32;
        drawDisc(window, agent.position, body_radius, color, impl_->view);

        if (agent.replanned_this_step) {
            drawRing(window, agent.position, 0.45, sf::Color(255, 215, 0), impl_->view, 3.0f);
        }

        if (agent.at_goal) {
            drawRing(window, agent.position, 0.5, sf::Color(50, 205, 50), impl_->view, 2.5f);
        }
    }

    window.setTitle(makeWindowTitle(frame));
    window.display();

    if (frame.simulation_complete) {
        while (window.isOpen()) {
            sf::Event event{};
            while (window.pollEvent(event)) {
                if (event.type == sf::Event::Closed) {
                    window.close();
                }
            }
            sf::sleep(sf::milliseconds(16));
        }
        return false;
    }

    const sf::Time slice = sf::milliseconds(16);
    sf::Clock clock;
    while (clock.getElapsedTime().asMilliseconds() < step_delay_ms_) {
        sf::Event event{};
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
                return false;
            }
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::Space) {
                    paused_ = !paused_;
                } else if (event.key.code == sf::Keyboard::N) {
                    step_once_ = true;
                    paused_ = false;
                }
            }
        }

        if (paused_ && !step_once_) {
            sf::sleep(sf::milliseconds(50));
            continue;
        }
        step_once_ = false;
        sf::sleep(slice);
    }

    return window.isOpen();
}

}  // namespace ccrrt

#endif
