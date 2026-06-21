# Changelog

## Recent changes

### Runtime JSON configuration

- Added `config/ccrrt.json` for planner parameters and run options.
- Added `config/scenarios.json` for scenario geometry.
- Added `config/python_compat.json` for the Python prototype compatibility preset.
- Added `--config <file>` and JSON-driven scenario selection.
- Added dynamic obstacle path generators: `vertical`, `horizontal`, and `line`.
- Added scenario categories for `--list-scenarios` and `--benchmark-all`.

### Replay visualization

- Removed the native C++ window visualization dependency.
- Added browser replay via `tools/replay_viewer.html`.
- Added `scenario.json` export so each replay folder is self-contained.
- Added `replay_frames.json` export for active receding-horizon plans, future covariance, and collision risk over time.
- Replay folders now contain `trajectories.csv`, `summary.json`, `scenario.json`, and `replay_frames.json`.

### Planner and collision checking

- Added `python_reference` scenario matching the Python prototype geometry.
- Added `--python-compat` and `pythonCompatPlannerConfig()`.
- Added `LegacyPythonCollisionChecker`.
- Added explicit `variance_per_waypoint` support for dynamic obstacle schedules.
- Improved Monte Carlo edge checks against higher-priority agent and dynamic-obstacle broadcast segments.
- Added `max_timesteps` to `PlannerConfig`.
- `MultiAgentPlanner` can switch between Monte Carlo and legacy collision checkers at runtime.

### Documentation

- Updated `README.md`, `CONFIG.md`, and `ARCHITECTURE.md` for the browser replay workflow.
