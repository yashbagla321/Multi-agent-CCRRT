# Changelog

## Recent changes (2026)

### Runtime JSON configuration

- Added `config/ccrrt.json` — planner parameters and run options (no rebuild).
- Added `config/scenarios.json` — **all scenario geometry** (obstacles, agents, dynamic paths); replaces hardcoded C++ scenario factories.
- Added `config/python_compat.json` — preset for `Multiagent CCRRT.py` compatibility.
- Added `--config <file>` CLI flag; auto-discovers `config/ccrrt.json`.
- Dynamic obstacle `path` generators: `vertical`, `horizontal`, `line`.
- Scenario `category`: `paper` or `performance` (for benchmarks).
- Inline `"scenarios"` in `ccrrt.json` overrides entries in `scenarios_file` by name.
- Removed `ccrrt_scenarios` library; scenarios are config-driven only.
- See **[CONFIG.md](CONFIG.md)** for the full schema; per-field docs are inline in `config/ccrrt.json`.

### Python reference test case

- Added built-in scenario `python_reference` matching `Multiagent CCRRT.py` `main()` geometry.
- Added `--python-compat` flag and `pythonCompatPlannerConfig()`:
  - `expand_distance = 1.0`, `max_iterations = 500`
  - Multiplicative variance growth (`× 1.1` per tree edge)
  - Deterministic legacy collision checker
- Added `LegacyPythonCollisionChecker` (`legacy_collision_checker.cpp`).
- Added `variance_per_waypoint` on `DynamicObstacleSpec` for explicit dynamic-obstacle uncertainty schedules.

### Visualization fix

- Fixed SFML coordinate mapping in `sfml_renderer.cpp`:
  - Workspace `[bounds_min, bounds_max]` now maps correctly into the 800×800 window.
  - Previously ignored `bounds_min`, pushing most geometry off-screen.
- Added workspace border and start/goal markers on simulation result view.

### Collision checking improvements

- Monte Carlo `isEdgeSafe` now rejects edges that intersect higher-priority agent or dynamic-obstacle broadcast segments (paper §4.1; matches Python segment-intersection checks).

### Planner configuration

- Added `max_timesteps` to `PlannerConfig` (was hardcoded at 500).
- `MultiAgentPlanner` uses `std::unique_ptr<ICollisionChecker>` to switch between Monte Carlo and legacy checkers at runtime.

### Documentation

- **CONFIG.md** — runtime configuration reference.
- Per-field parameter docs inline in `config/ccrrt.json` (`"_field": "description"` before each field).
- **CHANGELOG.md** — this file.
- Updated **README.md** with config and python_reference usage.
