# Multi-Agent CC-RRT (C++)

C++17 implementation of the receding-horizon multi-agent Chance-Constrained RRT algorithm from:

[On Receding Horizon Chance Constraint Motion Planning for Uncertain Multi-Agent Systems](https://doi.org/10.1115/DSCC2019-9237)

## Features

- Single-agent CC-RRT with Monte Carlo chance constraints
- Priority-based multi-agent coordination (Algorithms 1 & 2)
- Receding horizon execution with Kalman measurement updates
- Lazy collision check on the immediate horizon
- Dynamic obstacles with predictable mean trajectories and growing uncertainty
- Legacy Python-compatible collision mode (`--python-compat`)
- Runtime JSON configuration — edit `config/` and re-run without rebuilding
- Smoother executed paths via finer `motion_step`; optional RRT shortcut smoothing (`enable_path_smoothing`)
- CSV/JSON trajectory export and benchmark summaries
- Optional SFML visualization (static preview, live per-timestep simulation, post-run result view)
- Google Test unit test suite

## Build

Requirements:
- CMake 3.16+
- C++17 compiler (GCC 9+, Clang 10+, or MSVC 2022)
- Git (FetchContent downloads Eigen, nlohmann/json, and Google Test)
- SFML 2.6 for visualization (optional — see platform notes below)

Unit tests are enabled by default (`CCRRT_BUILD_TESTS=ON`). After build, runtime config files are copied next to the executable (`ccrrt.json`, `config/scenarios.json`, etc.).

### Windows

SFML is vendored locally so you do not need a system install:

```powershell
cd cpp
powershell -ExecutionPolicy Bypass -File scripts/fetch_sfml.ps1
cmake -B build -DCCRRT_ENABLE_VISUALIZATION=ON
cmake --build build --config Release
```

This downloads SFML 2.6.1 (MSVC 64-bit) into `third_party/sfml/` (`bin/`, `lib/`, `include/`). CMake uses that path automatically and copies DLLs next to the executable after build.

Executables: `build/Release/multi_agent_ccrrt.exe`, `build/Release/ccrrt_tests.exe`

Build without visualization:

```powershell
cmake -B build -DCCRRT_ENABLE_VISUALIZATION=OFF
cmake --build build --config Release
```

### Linux

Install build tools and SFML from your package manager.

**Ubuntu / Debian:**

```bash
sudo apt update
sudo apt install build-essential cmake git libsfml-dev
```

**Fedora / RHEL:**

```bash
sudo dnf install gcc-c++ cmake git SFML-devel
```

Configure and build (Release):

```bash
cd cpp
cmake -B build -DCMAKE_BUILD_TYPE=Release -DCCRRT_ENABLE_VISUALIZATION=ON
cmake --build build
```

Executables: `build/multi_agent_ccrrt`, `build/ccrrt_tests`

On Linux, CMake uses the system SFML from `libsfml-dev` / `SFML-devel`. The Windows bundled SFML under `third_party/sfml/` is not used. A display server (X11 or Wayland) is required for SFML windows.

Build without visualization (headless / CI):

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DCCRRT_ENABLE_VISUALIZATION=OFF
cmake --build build
```

Run unit tests:

```bash
ctest --test-dir build --output-on-failure
# or
./build/ccrrt_tests
```

Disable tests: `-DCCRRT_BUILD_TESTS=OFF`.

## Run

A config file is **required** at startup. The executable auto-discovers `config/ccrrt.json` (or pass `--config <file>`). Scenarios are loaded from `config/scenarios.json` unless overridden inline in the config.

```bash
# Linux
./build/multi_agent_ccrrt --scenario figure5
./build/multi_agent_ccrrt --scenario figure6 --no-viz
./build/multi_agent_ccrrt --help
```

```powershell
# Windows (MSVC)
./build/Release/multi_agent_ccrrt.exe --scenario figure5
./build/Release/multi_agent_ccrrt.exe --scenario figure6 --no-viz
./build/Release/multi_agent_ccrrt.exe --help
```

### Runtime configuration (no rebuild)

Edit `config/ccrrt.json` (planner/run settings) and `config/scenarios.json` (obstacle positions, agent starts/goals, dynamic paths). Re-run the executable — no compile step needed.

```bash
# Linux — auto-loads config/ccrrt.json when present
./build/multi_agent_ccrrt --no-viz
./build/multi_agent_ccrrt --config config/python_compat.json --no-viz
./build/multi_agent_ccrrt --mc-samples 100 --seed 42
```

```powershell
# Windows
./build/Release/multi_agent_ccrrt.exe --no-viz
./build/Release/multi_agent_ccrrt.exe --config config/python_compat.json --no-viz
./build/Release/multi_agent_ccrrt.exe --mc-samples 100 --seed 42
```

See **[CONFIG.md](CONFIG.md)** for the full schema, editing examples, and CLI reference. Per-field descriptions are inline in `config/ccrrt.json` (`"_field": "..."` before each setting). Recent changes are listed in **[CHANGELOG.md](CHANGELOG.md)**.

### Performance benchmarks

Six performance scenarios in `config/scenarios.json` stress different bottlenecks:

| Scenario | What it tests |
|----------|----------------|
| `perf_cluttered` | 10 static obstacles, dense MC collision checks |
| `perf_four_agents` | 4 agents, priority-chain planning |
| `perf_narrow_passage` | Tight corridor, frequent replans |
| `perf_long_paths` | 3 agents, long corner-to-corner horizons |
| `perf_multi_dynamic` | 3 dynamic obstacles crossing agent paths |
| `perf_stress` | 3 agents + clutter + 3 dynamic (combined load) |

```bash
# Linux
./build/multi_agent_ccrrt --scenario perf_cluttered --no-viz
./build/multi_agent_ccrrt --benchmark-all --mc-samples 200
./build/multi_agent_ccrrt --list-scenarios
```

```powershell
# Windows
./build/Release/multi_agent_ccrrt.exe --scenario perf_cluttered --no-viz
./build/Release/multi_agent_ccrrt.exe --benchmark-all --mc-samples 200
./build/Release/multi_agent_ccrrt.exe --list-scenarios
```

`--benchmark-all` writes `output/benchmark/benchmark.csv` with columns: `scenario`, `success`, `elapsed_ms`, `replan_count`, `total_steps`, `max_timestep`. Each scenario also gets its own subdirectory under `output/benchmark/`.

Use a lower `--mc-samples` value (e.g. 200) for faster iteration; use 1000 to match the paper.

## Unit tests

Google Test suite under `tests/` verifies geometry, Kalman filter, collision checkers, planners, JSON config loading, and export. See **Build** above for platform-specific `ctest` commands.

Or run directly:

```bash
./build/ccrrt_tests          # Linux
./build/Release/ccrrt_tests.exe   # Windows
```

| Test file | What it verifies |
|-----------|------------------|
| `test_geometry.cpp` | Distances, segment intersection, chi-squared thresholds |
| `test_kalman_filter.cpp` | Predict/update variance, deterministic measurements |
| `test_types.cpp` | Trajectory prediction variance growth, `Vec2` |
| `test_collision_checker.cpp` | Monte Carlo node/edge safety |
| `test_legacy_collision_checker.cpp` | Python-compatible deterministic checks |
| `test_ccrrt_planner.cpp` | Single-agent plan success/failure |
| `test_multi_agent_planner.cpp` | Multi-agent goals, metrics, legacy mode smoke |
| `test_runtime_config.cpp` | JSON config + `figure5` integration |
| `test_trajectory_exporter.cpp` | CSV/JSON/benchmark export |

### Preview scenarios (no simulation)

Visualize layouts from `config/scenarios.json` before running the planner.
Requires SFML at build time.

```bash
# Linux
./build/multi_agent_ccrrt --scenario figure5 --preview
./build/multi_agent_ccrrt --preview-all
```

```powershell
# Windows
./build/Release/multi_agent_ccrrt.exe --scenario figure5 --preview
./build/Release/multi_agent_ccrrt.exe --preview-all
```

Preview window legend:
- **Black discs** — static obstacles
- **Filled colored circle** — agent start (with faint confidence disc)
- **Colored ring** — agent goal
- **Faint line** — start-to-goal hint (not the planned path)
- **Magenta trail** — dynamic obstacle mean waypoints

### Live simulation visualization

When `live_visualization` is enabled (default), the planner opens an SFML window that updates after each timestep:

- **Solid colored lines** — executed path so far
- **Faint lines** — current receding-horizon plan
- **Filled disc** — current agent position (gold ring = replanned this step)
- **Magenta** — dynamic obstacle current position and predicted trail

Controls: **Space** = pause/resume, **N** = advance one step while paused, close window = stop simulation.

```bash
# Linux
./build/multi_agent_ccrrt --scenario figure5
./build/multi_agent_ccrrt --scenario figure5 --viz-delay-ms 50
./build/multi_agent_ccrrt --scenario figure5 --no-live-viz
./build/multi_agent_ccrrt --no-viz
```

```powershell
# Windows
./build/Release/multi_agent_ccrrt.exe --scenario figure5
./build/Release/multi_agent_ccrrt.exe --scenario figure5 --viz-delay-ms 50
./build/Release/multi_agent_ccrrt.exe --scenario figure5 --no-live-viz
./build/Release/multi_agent_ccrrt.exe --no-viz
```

### CLI options

| Flag | Description |
|------|-------------|
| `--help`, `-h` | Print usage and exit |
| `--config <file>` | Load JSON config (default: `config/ccrrt.json` if present) |
| `--scenario` | Any name from `--list-scenarios` (built-in or config-defined) |
| `--benchmark-all` | Run all `perf_*` scenarios; write `benchmark.csv` |
| `--preview` | Show scenario layout only; skip simulation |
| `--preview-all` | Preview all scenarios in sequence |
| `--list-scenarios` | Print scenario names and exit |
| `--no-viz` | Disable all SFML visualization |
| `--no-live-viz` | Skip live step-by-step viz (keep post-run static view) |
| `--viz-delay-ms <n>` | Milliseconds between live viz frames (default: 150) |
| `--output <dir>` | CSV/JSON output directory |
| `--seed <n>` | RNG seed |
| `--mc-samples <n>` | Monte Carlo samples per collision check |
| `--path-smoothing` | Enable RRT shortcut path smoothing for this run |
| `--python-compat` | Use `Multiagent CCRRT.py` planner settings |

## Project layout

```
include/ccrrt/     Public headers (Doxygen-documented)
src/               Core library implementation
config/            Runtime JSON: ccrrt.json, scenarios.json, python_compat.json
tests/             Google Test unit tests
scripts/           fetch_sfml.ps1 and other helpers
scenarios/         Legacy C++ scenario factories (geometry lives in config/scenarios.json)
main.cpp           CLI entry point
CONFIG.md          Runtime configuration reference
CHANGELOG.md       Recent feature and fix history
ARCHITECTURE.md    System design, data flow, and paper mapping
Doxyfile           API documentation generator config
```

See **[ARCHITECTURE.md](ARCHITECTURE.md)** for layer diagrams, end-to-end data flow, algorithm walkthroughs, and extension points.

## Code documentation

All public types and functions use **Doxygen** comments (`@brief`, `@param`, `@return`).
Headers declare the API; `.cpp` files include `@file` blocks and inline notes for
non-obvious logic (e.g. Algorithm 1/2 steps, collision-check stages).

| Location | What is documented |
|----------|-------------------|
| `include/ccrrt/*.hpp` | Every struct field, class, and public/private method |
| `src/*.cpp` | File purpose, helper functions, algorithm step comments |
| `config/ccrrt.json` | Per-field `_` comments for run/planner parameters |
| `CONFIG.md` | Config schema, scenario format, editing examples |
| `main.cpp` | CLI flow and exit codes |
| `include/ccrrt/ccrrt.hpp` | Umbrella header + architecture overview |

Generate HTML docs (requires [Doxygen](https://www.doxygen.nl/)):

```bash
cd cpp
doxygen Doxyfile
# Open docs/doxygen/html/index.html
```

## Algorithm parameters

Shipped defaults in `config/ccrrt.json` (override there without rebuilding):

| Parameter | Default in `ccrrt.json` | Paper (Section 5) |
|-----------|-------------------------|-------------------|
| Step size (`expand_distance`) | 0.5 | 0.5 |
| Collision bound M | 0.2 | 0.2 |
| Confidence alpha | 0.99 | 0.99 |
| MC samples | 1000 | 1000 |
| Max timesteps | 1000 | 500 |
| Motion step (`motion_step`) | 0.2 | (one RRT edge per step) |
| Path smoothing (`enable_path_smoothing`) | off | — |
| Initial variance | 0.1 | 0.2 |
| Process noise | 0.1 | 0.2 |
| Measurement noise | 0.2 | 0.2 |

C++ fallbacks when a field is omitted from JSON are defined in `include/ccrrt/config.hpp`.

## Output

Each run writes:
- `trajectories.csv` — executed agent steps (position, variance, replanned flag per step)
- `summary.json` — `success`, `replan_count`, `elapsed_ms`, `total_steps`, `max_timestep`, per-agent step counts
