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
- C++17 compiler (MSVC, GCC, or Clang)
- Git (for Eigen via FetchContent)
- Optional: SFML 2.6 for visualization

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

### CLI options

| Flag | Description |
|------|-------------|
| `--scenario` | `figure5`, `figure6`, or `figure7` |
| `--no-viz` | Skip SFML window |
| `--output <dir>` | CSV/JSON output directory |
| `--seed <n>` | RNG seed |
| `--mc-samples <n>` | Monte Carlo samples per collision check |

## Project layout

```
include/ccrrt/     Public headers
src/               Core library implementation
scenarios/         Paper figure scenario definitions
main.cpp           CLI entry point
```

## Algorithm parameters (paper Section 5)

| Parameter | Default |
|-----------|---------|
| Map size | 17 x 17 |
| Step size | 0.5 |
| Collision bound M | 0.2 |
| Confidence alpha | 0.99 |
| MC samples | 1000 |
| Initial variance | 0.2 |
| Process noise | 0.2 |
| Measurement noise | 0.2 |

## Output

Each run writes:
- `trajectories.csv` — executed agent steps
- `summary.json` — success flag and replan count
