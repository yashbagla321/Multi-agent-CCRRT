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

Windows, from the repository root after building with the README commands:

```powershell
cpp\build\multi_agent_ccrrt.exe --config cpp\config\ccrrt.json

# Edit obstacle positions in config/scenarios.json, then re-run:
#   "static_obstacles": [{ "center": [7.0, 4.0], "radius": 2.0 }, ...]

# Python-compat preset
cpp\build\multi_agent_ccrrt.exe --config cpp\config\python_compat.json

# CLI overrides config
cpp\build\multi_agent_ccrrt.exe --config cpp\config\ccrrt.json --scenario figure6 --seed 7 --mc-samples 100
```

Linux/macOS, from the repository root:

```bash
./cpp/build/multi_agent_ccrrt --config cpp/config/ccrrt.json

# Python-compat preset
./cpp/build/multi_agent_ccrrt --config cpp/config/python_compat.json

# CLI overrides config
./cpp/build/multi_agent_ccrrt --config cpp/config/ccrrt.json --scenario figure6 --seed 7 --mc-samples 100
```

After build, configs are copied next to the executable in `cpp/build/config/` (and `cpp/build/ccrrt.json` for auto-discovery). Those copied files are snapshots; use `--config cpp\config\ccrrt.json` on Windows or `--config cpp/config/ccrrt.json` on Linux/macOS when you want source JSON edits to reflect immediately without rebuilding.

## Replay output

Normal scenario runs write a self-contained replay folder, usually `output/<scenario>`:

| File | Purpose |
|------|---------|
| `trajectories.csv` | Executed agent positions, variance, and replan markers |
| `summary.json` | Success flag, elapsed time, step counts, and total replans |
| `scenario.json` | Bounds, static obstacles, dynamic obstacles, starts, and goals |
| `replay_frames.json` | Per-timestep active plans, future covariance, and current/next/max collision probabilities |

Open `tools/replay_viewer.html` in Chrome or Edge and select the output folder to view the run.

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
| `run` | Which scenario to run and where to write output |
| `planner` | Algorithm parameters |
| `scenarios` | Optional inline scenarios; **override** entries in `scenarios_file` by name |

See `config/ccrrt.json` for per-field descriptions.

### Path smoothing (`planner.enable_path_smoothing`)

Optional **greedy shortcut smoothing** runs after each RRT plan. It replaces zig-zag geometry with straight shortcut spans only when the full span is still chance-constraint safe against static obstacles, higher-priority broadcasts, and dynamic obstacles. The returned trajectory still keeps one node per discrete future timestep, so prediction timing and collision-probability visualization remain aligned with the theory.

| Enable | How |
|--------|-----|
| Config | `"enable_path_smoothing": true` in `config/ccrrt.json` |
| CLI | `--path-smoothing` |
| Disable | `"enable_path_smoothing": false` in `config/ccrrt.json` or `--no-path-smoothing` |

The C++ library default is off; the project config can turn it on or off. Re-run after changing the JSON; no rebuild needed.

---

## `config/scenarios.json` — scenario geometry

All built-in scenarios are defined here:

- **Paper:** `figure5`, `figure6`, `figure7`, `python_reference`
- **Performance:** `perf_cluttered`, `perf_four_agents`, `perf_narrow_passage`, `perf_long_paths`, `perf_multi_dynamic`, `perf_stress`

| Scenario | Category | Purpose |
|----------|----------|---------|
| `figure5` | paper | Paper Fig. 5: two agents, four static obstacles, one dynamic obstacle |
| `figure6` | paper | Priority and time-separated crossing |
| `figure7` | paper | Zig-zag / waiting behavior around a dynamic obstacle |
| `python_reference` | paper | Geometry aligned with `Multiagent CCRRT.py` |
| `perf_cluttered` | performance | Dense static-obstacle Monte Carlo workload |
| `perf_four_agents` | performance | Four-agent priority-chain planning |
| `perf_narrow_passage` | performance | Tight-corridor replanning |
| `perf_long_paths` | performance | Long-horizon multi-agent paths |
| `perf_multi_dynamic` | performance | Multiple dynamic-obstacle predictions |
| `perf_stress` | performance | Combined clutter, agents, and dynamic obstacles |

Agent labels should follow visualization priority/color order inside each scenario: `red`, `blue`, `green`, `orange`, then the remaining palette colors as needed. Lower `priority` values plan first and appear first in the replay side panel.

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
    { "id": 0, "priority": 0, "name": "red", "start": [0, 0], "goal": [10, 10] }
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
| `--list-scenarios` | `run.list_scenarios = true` |
| `--run-all` | `run.run_all = true` |
| `--run-all-normal` | `run.run_all_normal = true` |
| `--run-all-smooth` | `run.run_all_smooth = true` |
| `--benchmark-all` | `run.benchmark_all = true` |
| `--seed <n>` | `planner.rng_seed` |
| `--mc-samples <n>` | `planner.mc_samples` |
| `--path-smoothing` | `planner.enable_path_smoothing = true` |
| `--no-path-smoothing` | `planner.enable_path_smoothing = false` |
| `--python-compat` | `run.python_compat = true` (+ planner preset) |

## Running scenarios and benchmarks

List every scenario loaded from `config/scenarios.json`:

```powershell
cpp\build\multi_agent_ccrrt.exe --config cpp\config\ccrrt.json --list-scenarios
```

```bash
./cpp/build/multi_agent_ccrrt --config cpp/config/ccrrt.json --list-scenarios
```

Run one scenario and write a replay folder:

```powershell
cpp\build\multi_agent_ccrrt.exe --config cpp\config\ccrrt.json --scenario figure5 --output output\figure5
```

```bash
./cpp/build/multi_agent_ccrrt --config cpp/config/ccrrt.json --scenario figure5 --output output/figure5
```

Run every paper and performance scenario in both normal and smoothed modes:

```powershell
cpp\build\multi_agent_ccrrt.exe --config cpp\config\ccrrt.json --run-all
```

```bash
./cpp/build/multi_agent_ccrrt --config cpp/config/ccrrt.json --run-all
```

Default `--run-all` outputs:

| Mode | Replay folders | Benchmark summary |
|------|----------------|-------------------|
| Normal | `output/<scenario>` | `output/benchmark/benchmark.csv` |
| Smoothed | `output/<scenario>_smooth` | `output/benchmark_smooth/benchmark.csv` |

Run only the normal algorithm sweep:

```powershell
cpp\build\multi_agent_ccrrt.exe --config cpp\config\ccrrt.json --run-all-normal
```

```bash
./cpp/build/multi_agent_ccrrt --config cpp/config/ccrrt.json --run-all-normal
```

Run only the smoothed algorithm sweep:

```powershell
cpp\build\multi_agent_ccrrt.exe --config cpp\config\ccrrt.json --run-all-smooth
```

```bash
./cpp/build/multi_agent_ccrrt --config cpp/config/ccrrt.json --run-all-smooth
```

For a single scenario with no explicit `--output`, normal runs use `output/<scenario>` and smoothed runs use `output/<scenario>_smooth`.

Run the normal performance benchmark set. This runs scenarios with category `"performance"` and writes per-scenario outputs plus `output/benchmark/benchmark.csv`:

```powershell
cpp\build\multi_agent_ccrrt.exe --config cpp\config\ccrrt.json --benchmark-all --no-path-smoothing
```

```bash
./cpp/build/multi_agent_ccrrt --config cpp/config/ccrrt.json --benchmark-all --no-path-smoothing
```

Run the smoothed performance benchmark set and write `output/benchmark_smooth/benchmark.csv`:

```powershell
cpp\build\multi_agent_ccrrt.exe --config cpp\config\ccrrt.json --benchmark-all --path-smoothing
```

```bash
./cpp/build/multi_agent_ccrrt --config cpp/config/ccrrt.json --benchmark-all --path-smoothing
```
