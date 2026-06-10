# Runtime configuration

All **planner parameters** and **scenario geometry** (obstacles, starts, goals, dynamic paths) live in JSON files. Edit and re-run — **no rebuild required**.

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

## `ccrrt.json` top-level schema

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

---

## `run` — execution options

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `scenario` | string | `"figure5"` | Name from `scenarios.json` |
| `output_directory` | string | `""` | Empty → `output/<scenario>` |
| `enable_visualization` | bool | `true` | Master switch for SFML visualization |
| `live_visualization` | bool | `true` | Step-by-step live window during simulation |
| `viz_step_delay_ms` | int | `150` | Delay between live viz frames (milliseconds) |
| `preview_only` | bool | `false` | Layout preview only |
| `preview_all` | bool | `false` | Preview every scenario |
| `list_scenarios` | bool | `false` | Print scenario list and exit |
| `benchmark_all` | bool | `false` | Run all `category: performance` scenarios |
| `python_compat` | bool | `false` | `Multiagent CCRRT.py` compatibility mode |

---

## `planner` — algorithm parameters

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `expand_distance` | number | `0.5` | RRT step size |
| `collision_bound_M` | number | `0.2` | Max collision probability per step |
| `confidence_alpha` | number | `0.99` | α-confidence level |
| `mc_samples` | int | `1000` | Monte Carlo samples per check |
| `max_iterations` | int | `5000` | Max RRT iterations |
| `max_timesteps` | int | `500` | Max simulation timesteps |
| `goal_sample_rate` | int | `5` | Goal-bias rate (percent) |
| `initial_variance` | number | `0.2` | Initial position variance |
| `process_noise` | number | `0.2` | Additive variance per tree step |
| `max_prediction_variance` | number | `0.8` | Cap on prediction tube size |
| `measurement_noise` | number | `0.2` | Kalman measurement noise |
| `bounds_min` / `bounds_max` | number | `-2` / `17` | RRT sampling bounds |
| `rng_seed` | int | `42` | RNG seed |
| `use_legacy_collision` | bool | `false` | Python-style deterministic checks |
| `legacy_p_safe` | number | `0.8` | Legacy safety factor |
| `variance_growth_alpha` | number | `0.1` | Multiplicative variance growth (legacy) |

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
      "initial_variance": 0.1,
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

Keys starting with `_` are ignored (use for comments like `_comment`).
