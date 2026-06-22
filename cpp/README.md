# Multi-Agent CC-RRT (C++)

C++17 implementation of the receding-horizon multi-agent Chance-Constrained RRT algorithm from:

[On Receding Horizon Chance Constraint Motion Planning for Uncertain Multi-Agent Systems](https://doi.org/10.1115/DSCC2019-9237)

## Features

- Single-agent CC-RRT with Monte Carlo chance constraints
- Priority-based multi-agent coordination
- Receding-horizon execution with Kalman measurement updates
- Lazy collision checks on the immediate horizon
- Dynamic obstacles with predictable mean trajectories and uncertainty
- Runtime JSON configuration in `config/`
- CSV/JSON replay export
- Browser replay viewer for active plans, collision risk, obstacles, replans, and covariance
- Google Test unit test suite

## Build

Requirements:

- CMake 3.16+
- C++17 compiler
- Git, used by CMake `FetchContent` for Eigen, nlohmann/json, and Google Test
- Ninja, Make, or a supported CMake generator

Windows example:

```powershell
cmake -S cpp -B cpp/build -G Ninja
cmake --build cpp/build --target multi_agent_ccrrt ccrrt_tests
```

Linux example:

```bash
cd cpp
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target multi_agent_ccrrt ccrrt_tests
```

Unit tests are enabled by default with `CCRRT_BUILD_TESTS=ON`.

## Run

A config file is required at startup. For edit-and-run work, pass the source config so changes in `cpp/config/scenarios.json` are read immediately. The executable can also auto-discover the copied build config next to the executable, but that copy is refreshed by the build step.

```powershell
cpp\build\multi_agent_ccrrt.exe --config cpp\config\ccrrt.json --scenario figure5
cpp\build\multi_agent_ccrrt.exe --config cpp\config\ccrrt.json --scenario figure6
cpp\build\multi_agent_ccrrt.exe --config cpp\config\ccrrt.json --help
```

Linux/macOS, from the repository root:

```bash
./cpp/build/multi_agent_ccrrt --config cpp/config/ccrrt.json --scenario figure5
./cpp/build/multi_agent_ccrrt --config cpp/config/ccrrt.json --scenario figure6
./cpp/build/multi_agent_ccrrt --config cpp/config/ccrrt.json --help
```

List every scenario defined in `cpp/config/scenarios.json`:

```powershell
cpp\build\multi_agent_ccrrt.exe --config cpp\config\ccrrt.json --list-scenarios
```

```bash
./cpp/build/multi_agent_ccrrt --config cpp/config/ccrrt.json --list-scenarios
```

Run every paper and performance scenario in both normal and smoothed modes:

```powershell
cpp\build\multi_agent_ccrrt.exe --config cpp\config\ccrrt.json --run-all
```

```bash
./cpp/build/multi_agent_ccrrt --config cpp/config/ccrrt.json --run-all
```

This writes normal replay folders as `output/<scenario>`, smoothed replay folders as `output/<scenario>_smooth`, and benchmark summaries as `output/benchmark/benchmark.csv` and `output/benchmark_smooth/benchmark.csv`.

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

Run the performance benchmark set and write `output/benchmark/benchmark.csv`:

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

For a single scenario or benchmark run, the default output folder follows the smoothing mode. Normal outputs use `output/<scenario>` and `output/benchmark`; smoothed outputs use `output/<scenario>_smooth` and `output/benchmark_smooth`. Use `--no-path-smoothing` or `--path-smoothing` to force the mode from the command line.

Useful flags:

| Flag | Description |
|------|-------------|
| `--config <file>` | Load JSON config |
| `--scenario <name>` | Run a scenario from `scenarios.json` |
| `--list-scenarios` | Print scenario names |
| `--run-all` | Run every scenario in normal and smoothed modes |
| `--run-all-normal` | Run every scenario with smoothing disabled |
| `--run-all-smooth` | Run every scenario with smoothing enabled |
| `--benchmark-all` | Run all performance scenarios and write `benchmark.csv` |
| `--output <dir>` | Choose output directory or root for `--run-all` |
| `--seed <n>` | Override RNG seed |
| `--mc-samples <n>` | Override Monte Carlo sample count |
| `--path-smoothing` | Enable shortcut smoothing for this run |
| `--no-path-smoothing` | Disable shortcut smoothing for this run |
| `--python-compat` | Use Python-aligned planner settings |

## Visualization

Native C++ window visualization has been removed. The supported visualization path is the browser replay viewer:

1. Run a scenario or `--run-all`.
2. Open `tools/replay_viewer.html` in Chrome or Edge.
3. Select the output folder, for example `output/figure5`.

Each run writes:

- `trajectories.csv` - executed agent steps
- `summary.json` - success, replans, timing, and step counts
- `scenario.json` - bounds, obstacles, starts/goals, dynamic paths, and covariance
- `replay_frames.json` - per-timestep active plans, future-node covariance, and collision probability

The replay viewer loads those files from the folder automatically and shows executed paths, current active plans for every agent, covariance discs at current and future nodes, current/next/max collision probability at the selected timestep, static obstacles, dynamic obstacles, and replan markers.

## Runtime Configuration

Edit `cpp/config/ccrrt.json` for planner/run settings and `cpp/config/scenarios.json` for geometry. Re-run with `--config cpp\config\ccrrt.json` on Windows or `--config cpp/config/ccrrt.json` on Linux/macOS after changing JSON; no rebuild is required.

For detailed config docs, see [CONFIG.md](CONFIG.md).

## Tests

```powershell
cpp\build\ccrrt_tests.exe
```

or:

```bash
ctest --test-dir cpp/build --output-on-failure
```

## Project Layout

```text
cpp/
  main.cpp
  include/ccrrt/
  src/
  config/
  tests/
  CONFIG.md
  ARCHITECTURE.md
tools/
  replay_viewer.html
```
