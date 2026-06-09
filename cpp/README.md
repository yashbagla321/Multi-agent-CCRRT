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

### Preview scenarios (no simulation)

Visualize layouts from `scenarios/paper_figures.cpp` before running the planner.
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
| `--scenario` | `figure5`, `figure6`, or `figure7` |
| `--preview` | Show scenario layout only; skip simulation |
| `--preview-all` | Preview all scenarios in sequence |
| `--list-scenarios` | Print scenario names and exit |
| `--no-viz` | Skip SFML window after simulation |
| `--output <dir>` | CSV/JSON output directory |
| `--seed <n>` | RNG seed |
| `--mc-samples <n>` | Monte Carlo samples per collision check |

## Project layout

```
include/ccrrt/     Public headers (Doxygen-documented)
src/               Core library implementation
scenarios/         Paper figure scenario definitions
main.cpp           CLI entry point
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
| Initial variance | 0.2 |
| Process noise | 0.2 |
| Measurement noise | 0.2 |

## Output

Each run writes:
- `trajectories.csv` — executed agent steps
- `summary.json` — success flag and replan count
