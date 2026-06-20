# Runtime configuration

All **planner parameters** and **scenario geometry** (obstacles, starts, goals, dynamic paths) live in JSON files. Edit and re-run — **no rebuild required**.

Parameter descriptions are inline in the config files (`"_field_name": "..."` immediately before each field). Keys starting with `_` are ignored by the loader.

## Files

| File | Purpose |
|------|---------|
| `config/ccrrt.json` | Run options + planner parameters; points to scenarios file |
| `config/scenarios.json` | **All test scenarios** — obstacle positions/radii, agents, dynamic obstacles |
| `config/python_compat.json` | Preset for `Multiagent CCRRT.py` compatibility |

## Quick start

```powershell
cd cpp

# Requires config/ccrrt.json (loads config/scenarios.json automatically)
./build/Release/multi_agent_ccrrt.exe --no-viz

# Edit obstacle positions in config/scenarios.json, then re-run:
#   "static_obstacles": [{ "center": [7.0, 4.0], "radius": 2.0 }, ...]

# Python-compat preset
./build/Release/multi_agent_ccrrt.exe --config config/python_compat.json --no-viz

# CLI overrides config
./build/Release/multi_agent_ccrrt.exe --scenario figure6 --seed 7 --mc-samples 100
```

After build, configs are copied to `build/Release/config/` (and `build/Release/ccrrt.json`).

## Precedence

| Priority | Source |
|----------|--------|
| 1 (highest) | Command-line flags |
| 2 | Inline `"scenarios"` block in `ccrrt.json` (overrides by name) |
| 3 | `scenarios_file` (default `scenarios.json`) |
| 4 (lowest) | C++ defaults in `config.hpp` (planner only) |

A config file is **required** at startup (auto-discovered or via `--config`).

---

## `ccrrt.json` structure

```json
{
  "scenarios_file": "scenarios.json",
  "run": { ... },
  "planner": { ... },
  "scenarios": { ... }
}
```

| Field | Description |
|-------|-------------|
| `scenarios_file` | Path relative to the config file's directory (default: `scenarios.json`) |
| `run` | Which scenario to run, visualization, output paths |
| `planner` | Algorithm parameters |
| `scenarios` | Optional inline scenarios; **override** entries in `scenarios_file` by name |

See `config/ccrrt.json` for per-field descriptions.

### Path smoothing (`planner.enable_path_smoothing`)

Optional **greedy shortcut smoothing** runs after each RRT plan: collinear waypoints are removed when a direct edge is still chance-constraint safe. This reduces zigzags in the planned path (separate from `motion_step`, which controls how finely the robot moves along each edge during execution).

| Enable | How |
|--------|-----|
| Config | `"enable_path_smoothing": true` in `config/ccrrt.json` |
| CLI | `--path-smoothing` |
| Disable | `"enable_path_smoothing": false` in `config/ccrrt.json` (default) |

Default: **off** (`false`). Re-run after changing — no rebuild needed.

---

## `config/scenarios.json` — scenario geometry

All built-in scenarios are defined here:

- **Paper:** `figure5`, `figure6`, `figure7`, `python_reference`
- **Performance:** `perf_cluttered`, `perf_four_agents`, `perf_narrow_passage`, `perf_long_paths`, `perf_multi_dynamic`, `perf_stress`

### Scenario schema

```json
"figure5": {
  "category": "paper",
  "description": "Paper Fig. 5: 2 agents, 4 static, 1 dynamic",
  "bounds": { "min": -2.0, "max": 17.0 },
  "static_obstacles": [
    { "center": [7.0, 4.0], "radius": 2.0 }
  ],
  "agents": [
    {
      "id": 0,
      "priority": 0,
      "name": "red",
      "start": [4.0, 0.0],
      "goal": [0.0, 13.0]
    }
  ],
  "dynamic_obstacles": [
    {
      "id": 0,
      "initial_variance": 0.2,
      "path": {
        "type": "vertical",
        "x": 10.0,
        "y_start": 0.0,
        "y_end": 12.0,
        "step": 0.5
      }
    }
  ]
}
```

### Field reference

| Field | Description |
|-------|-------------|
| `category` | `"paper"` or `"performance"` (controls `--list-scenarios` grouping and `--benchmark-all`) |
| `description` | Human-readable label |
| `bounds.min` / `bounds.max` | Workspace limits for the scenario |
| `static_obstacles[].center` | `[x, y]` disc center |
| `static_obstacles[].radius` | Disc radius (half of Python `obstacleList` diameter) |
| `agents[].id` | Unique agent id |
| `agents[].priority` | Lower = higher priority |
| `agents[].name` | Label for visualization |
| `agents[].start` / `goal` | `[x, y]` positions |
| `dynamic_obstacles[].initial_variance` | Variance at first waypoint |
| `dynamic_obstacles[].waypoints` | Explicit `[[x,y], ...]` list |
| `dynamic_obstacles[].variance_per_waypoint` | Per-step variance (overrides growth formula) |
| `dynamic_obstacles[].path` | Generated path (alternative to `waypoints`) |

### Dynamic obstacle `path` generators

**Vertical** (motion along y at fixed x):

```json
"path": { "type": "vertical", "x": 10.0, "y_start": 0.0, "y_end": 12.0, "step": 0.5 }
```

**Horizontal** (motion along x at fixed y):

```json
"path": { "type": "horizontal", "y": 8.0, "x_start": 1.0, "x_end": 14.0, "step": 1.0 }
```

**Line** (between two points):

```json
"path": { "type": "line", "from": [1.0, 1.0], "to": [14.0, 14.0], "step": 1.0 }
```

---

## Editing examples

**Move a static obstacle** in `config/scenarios.json`:

```json
{ "center": [8.0, 4.5], "radius": 2.0 }
```

**Change agent start/goal**:

```json
"start": [5.0, 0.0],
"goal": [0.0, 14.0]
```

**Add a new scenario** — append to `scenarios.json`:

```json
"my_corridor": {
  "category": "paper",
  "description": "Custom corridor test",
  "bounds": { "min": -2, "max": 17 },
  "static_obstacles": [],
  "agents": [
    { "id": 0, "priority": 0, "name": "a", "start": [0, 0], "goal": [10, 10] }
  ]
}
```

Then set `"scenario": "my_corridor"` in `ccrrt.json` or pass `--scenario my_corridor`.

**Override one field of figure5** without duplicating the whole file — add to `ccrrt.json`:

```json
"scenarios": {
  "figure5": {
    "category": "paper",
    "description": "Figure 5 with moved red start",
    "bounds": { "min": -2, "max": 17 },
    "static_obstacles": [ ... copy or subset ... ],
    "agents": [ ... edited starts ... ],
    "dynamic_obstacles": [ ... ]
  }
}
```

Inline `scenarios` entries replace the same-named entry from `scenarios_file`.

---

## CLI flags (override config)

| Flag | Config field |
|------|--------------|
| `--scenario <name>` | `run.scenario` |
| `--output <dir>` | `run.output_directory` |
| `--no-viz` | `run.enable_visualization = false` |
| `--no-live-viz` | `run.live_visualization = false` |
| `--viz-delay-ms <n>` | `run.viz_step_delay_ms` |
| `--preview` | `run.preview_only = true` |
| `--preview-all` | `run.preview_all = true` |
| `--list-scenarios` | `run.list_scenarios = true` |
| `--benchmark-all` | `run.benchmark_all = true` |
| `--seed <n>` | `planner.rng_seed` |
| `--mc-samples <n>` | `planner.mc_samples` |
| `--path-smoothing` | `planner.enable_path_smoothing = true` |
| `--python-compat` | `run.python_compat = true` (+ planner preset) |
