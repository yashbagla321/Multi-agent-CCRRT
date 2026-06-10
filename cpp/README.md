# Multi-Agent CC-RRT (C++)

C++17 implementation of the receding-horizon multi-agent Chance-Constrained RRT algorithm from:

[On Receding Horizon Chance Constraint Motion Planning for Uncertain Multi-Agent Systems](https://doi.org/10.1115/DSCC2019-9237)

## Features

- Single-agent CC-RRT with Monte Carlo chance constraints
- Priority-based multi-agent coordination (Algorithms 1 & 2)
- Receding horizon execution with Kalman measurement updates
- Lazy collision check on the immediate horizon
- Dynamic obstacles with predictable mean trajectories and growing uncertainty
- CSV/JSON trajectory export
- Optional SFML visualization

## Build

Requirements:
- CMake 3.16+
- C++17 compiler (MSVC 2022 recommended on Windows)
- Git (for Eigen via FetchContent)
- SFML 2.6 for visualization (bundled under `third_party/sfml/` — see below)

### Bundled SFML (Windows)

SFML is vendored locally so you do not need a system install:

```powershell
cd cpp
powershell -ExecutionPolicy Bypass -File scripts/fetch_sfml.ps1
```

This downloads SFML 2.6.1 (MSVC 64-bit) into `third_party/sfml/` (`bin/`, `lib/`, `include/`). CMake uses that path automatically and copies DLLs next to the executable after build.

See [`third_party/sfml/README.md`](third_party/sfml/README.md) for manual install or other compilers.

```powershell
cd cpp
cmake -B build -DCCRRT_ENABLE_VISUALIZATION=ON
cmake --build build --config Release
```

To build without visualization:

```powershell
cmake -B build -DCCRRT_ENABLE_VISUALIZATION=OFF
cmake --build build --config Release
```

## Run

```powershell
./build/Release/multi_agent_ccrrt.exe --scenario figure5
./build/Release/multi_agent_ccrrt.exe --scenario figure6 --no-viz
./build/Release/multi_agent_ccrrt.exe --scenario figure7 --seed 42 --output output/figure7
```

### Runtime configuration (no rebuild)

Edit `config/ccrrt.json` (planner/run settings) and `config/scenarios.json` (obstacle positions, agent starts/goals, dynamic paths). Re-run the executable — no compile step needed.

```powershell
# Auto-loads config/ccrrt.json when present
./build/Release/multi_agent_ccrrt.exe --no-viz

# Explicit preset (Python prototype compatibility)
./build/Release/multi_agent_ccrrt.exe --config config/python_compat.json --no-viz

# CLI overrides config
./build/Release/multi_agent_ccrrt.exe --mc-samples 100 --seed 42
```

See **[CONFIG.md](CONFIG.md)** for the full JSON schema. Recent changes are listed in **[CHANGELOG.md](CHANGELOG.md)**.

### Performance benchmarks

Six additional scenarios in `scenarios/performance_scenarios.cpp` stress different bottlenecks:

| Scenario | What it tests |
|----------|----------------|
| `perf_cluttered` | 10 static obstacles, dense MC collision checks |
| `perf_four_agents` | 4 agents, priority-chain planning |
| `perf_narrow_passage` | Tight corridor, frequent replans |
| `perf_long_paths` | 3 agents, long corner-to-corner horizons |
| `perf_multi_dynamic` | 3 dynamic obstacles crossing agent paths |
| `perf_stress` | 3 agents + clutter + 3 dynamic (combined load) |

```powershell
# Run one performance case
./build/Release/multi_agent_ccrrt.exe --scenario perf_cluttered --no-viz

# Run all performance cases; prints timing table + benchmark.csv
./build/Release/multi_agent_ccrrt.exe --benchmark-all --mc-samples 200

# List all scenarios with descriptions
./build/Release/multi_agent_ccrrt.exe --list-scenarios
```

`--benchmark-all` writes `output/benchmark/benchmark.csv` with columns: `scenario`, `success`, `elapsed_ms`, `replan_count`, `total_steps`, `max_timestep`. Each scenario also gets its own subdirectory under `output/benchmark/`.

Use a lower `--mc-samples` value (e.g. 200) for faster iteration; use 1000 to match the paper.

## Unit tests

Google Test suite under `tests/` verifies geometry, Kalman filter, collision checkers, planners, JSON config loading, and export.

```powershell
cd cpp
cmake -B build -DCCRRT_BUILD_TESTS=ON -DCCRRT_ENABLE_VISUALIZATION=OFF
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Or run directly:

```powershell
./build/Release/ccrrt_tests.exe
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

Disable tests: `-DCCRRT_BUILD_TESTS=OFF`.

### Preview scenarios (no simulation)

Visualize layouts from `scenarios/` before running the planner.
Requires SFML at build time.

```powershell
# Preview one scenario (static obstacles, starts, goals, dynamic obstacle path)
./build/Release/multi_agent_ccrrt.exe --scenario figure5 --preview

# Preview all three paper figures in sequence (close window to advance)
./build/Release/multi_agent_ccrrt.exe --preview-all

# List scenario names
./build/Release/multi_agent_ccrrt.exe --list-scenarios
```

Preview window legend:
- **Black discs** — static obstacles
- **Filled colored circle** — agent start (with faint confidence disc)
- **Colored ring** — agent goal
- **Faint line** — start-to-goal hint (not the planned path)
- **Magenta trail** — dynamic obstacle mean waypoints

### CLI options

| Flag | Description |
|------|-------------|
| `--config <file>` | Load JSON config (default: `config/ccrrt.json` if present) |
| `--scenario` | Any name from `--list-scenarios` (built-in or config-defined) |
| `--benchmark-all` | Run all `perf_*` scenarios; write `benchmark.csv` |
| `--preview` | Show scenario layout only; skip simulation |
| `--preview-all` | Preview all scenarios in sequence |
| `--list-scenarios` | Print scenario names and exit |
| `--no-viz` | Skip SFML window after simulation |
| `--output <dir>` | CSV/JSON output directory |
| `--seed <n>` | RNG seed |
| `--mc-samples <n>` | Monte Carlo samples per collision check |
| `--python-compat` | Use `Multiagent CCRRT.py` planner settings |

## Project layout

```
include/ccrrt/     Public headers (Doxygen-documented)
src/               Core library implementation
config/            Runtime JSON: ccrrt.json, scenarios.json, python_compat.json
scenarios/         Legacy headers only (geometry is in config/scenarios.json)
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
| `scenarios/` | Scenario factories and coordinate helpers |
| `main.cpp` | CLI flow and exit codes |
| `include/ccrrt/ccrrt.hpp` | Umbrella header + architecture overview |

Generate HTML docs (requires [Doxygen](https://www.doxygen.nl/)):

```powershell
cd cpp
doxygen Doxyfile
# Open docs/doxygen/html/index.html
```

## Algorithm parameters (paper Section 5)

| Parameter | Default |
|-----------|---------|
| Map size | 17 x 17 |
| Step size | 0.5 |
| Collision bound M | 0.2 |
| Confidence alpha | 0.99 |
| MC samples | 1000 |
| Max timesteps | 500 |
| Initial variance | 0.2 |
| Process noise | 0.2 |
| Measurement noise | 0.2 |

All of the above can be changed in `config/ccrrt.json` without rebuilding.

## Output

Each run writes:
- `trajectories.csv` — executed agent steps
- `summary.json` — success flag and replan count
