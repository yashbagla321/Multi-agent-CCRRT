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
- Added replay panels for per-agent current/next collision probability and run summary metrics.
- Added right-side replay information rail so Agents and Summary stay visible without covering the map.
- Final executed robot states now snap to the goal once the planner reaches the goal threshold, so replay paths do not stop one node short.
- Replay colors and the Agents panel now follow scenario priority order instead of raw agent ids, and the panel labels agents as priority 1, 2, 3, etc.
- Replay folders now contain `trajectories.csv`, `summary.json`, `scenario.json`, and `replay_frames.json`.

### Planner and collision checking

- Added `python_reference` scenario matching the Python prototype geometry.
- Added `--python-compat` and `pythonCompatPlannerConfig()`.
- Added `LegacyPythonCollisionChecker`.
- Added explicit `variance_per_waypoint` support for dynamic obstacle schedules.
- Improved Monte Carlo edge checks against higher-priority agent and dynamic-obstacle broadcast segments.
- Added `max_timesteps` to `PlannerConfig`.
- `MultiAgentPlanner` can switch between Monte Carlo and legacy collision checkers at runtime.
- Wired `enable_path_smoothing` into returned planner trajectories and added regression coverage for CLI flag overrides.
- Path smoothing now keeps discrete future timesteps while shortcutting geometry, preserving chance-check timing for predictions.
- Added `--run-all` to run every scenario in normal and smoothed modes, with separate `_smooth` output folders and benchmark summaries.
- Added `--run-all-normal` and `--run-all-smooth` for separate normal-only and smoothing-only scenario sweeps.
- Added `--no-path-smoothing`; default output folders now separate normal `output/<scenario>` and smoothed `output/<scenario>_smooth` runs.
- Standardized scenario agent labels to the visualization color order: red, blue, green, orange.

### Documentation

- Updated `README.md`, `CONFIG.md`, and `ARCHITECTURE.md` for the browser replay workflow.
