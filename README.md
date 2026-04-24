# OAL — Obstacle Avoidance Library

OAL is a C++ library that computes a COLREGS-compliant trajectory for an autonomous surface vehicle (ASV),
avoiding both static and moving obstacles using an A*-based local planner.

## Table of Contents

- [Overview](#overview)
- [News](#news)
- [Architecture](#architecture)
- [Plots](#plots)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Building and Installing](#building-and-installing)
  - [Running the Tests](#running-the-tests)
- [Usage](#usage)
- [Configuration](#configuration)
- [Known Limitations](#known-limitations)
- [Contributing](#contributing)

---

## Overview

**Key features:**

- A*-based path planning that navigates around static and moving obstacles.
- COLREGS-compliant manoeuvring (head-on, crossing, and overtaking rules).
- Multi-velocity expansion: each reachable node is explored at a configurable set of speeds, so the planner can
  find trajectories that use a slower speed to reach the goal sooner.
- Bounding-box safety margins that account for obstacle size, position uncertainty, and velocity uncertainty.
- Path-validity checking: after the initial plan, call `IsPathValid()` to verify the path is still safe after
  the vehicle moves.
- Optional JSON logging of nodes and obstacles for offline analysis and visualisation.

---

## News

- Every reachable node is expanded with a set of (given) different velocities: the result is a trajectory with mixed
  velocities (sometimes slower gets to the goal sooner).

---

## Architecture

The library is built around the following components (all under `include/oal/` and `src/`):

| Header / Source | Description |
|---|---|
| `data_structs.hpp` | Core data types: `Pose`, `VehicleData`, `Obstacle`, `BoundingBoxData`, `EncounterData`, `AStarNode`, `AstarPath`. |
| `oal_defines.hpp` | Compile-time string constants for encounter types (`HEAD_ON`, `VH_CROSSING_*`, `VH_OVERTAKING`) and search results (`FOUND`, `PARTIAL`, `FAIL`). |
| `obstacle.hpp` / `obstacle.cpp` | `oal::Obstacle` — stores pose, velocity, bounding-box parameters, and computes vertex positions at any future time. |
| `geometric_utilities.hpp` / `geometric_utilities.cpp` | Geometry helpers used by the planner (intersection tests, visibility checks, etc.). |
| `local_planner/generator.hpp` / `generator.cpp` | `oal::Generator` — main planner class. Call `FindPath()` to compute a path and `IsPathValid()` to re-validate it. |
| `local_planner/node.hpp` / `node.cpp` | `oal::AStarNode` — a waypoint node with cost estimates and parent linkage. |
| `local_planner/path.hpp` | `oal::AstarPath` — ordered list of `AStarNode` pointers representing the planned route. |
| `local_planner/path_evaluator.hpp` / `path_evaluator.cpp` | `oal::PathEvaluator` — evaluates and prunes candidate paths; manages the high-priority obstacle class list. |

---

## Plots

The `scripts/` directory contains Python scripts to visualise bounding boxes and planned trajectories.
Here is an example output:

![Figure_1](https://github.com/SamueleD98/oal/assets/28822110/34b667d5-8ca8-4d2a-bca5-49d13a8e3098)

---

## Getting Started

### Prerequisites

| Dependency | Minimum version | Notes |
|---|---|---|
| CMake | 3.2 | Build system |
| C++ compiler | C++17 | GCC 7+, Clang 5+, or equivalent |
| [Eigen3](https://eigen.tuxfamily.org) | 3 | Linear algebra |
| [nlohmann/json](https://github.com/nlohmann/json) | 3.11.3 | JSON serialisation |
| [Boost](https://www.boost.org) | — | `filesystem` and `system` components |

On Ubuntu/Debian you can install most dependencies with:

```bash
sudo apt install cmake build-essential libeigen3-dev libboost-filesystem-dev libboost-system-dev
```

`nlohmann/json` 3.11.3 must be installed separately (e.g. via vcpkg, Conan, or from source):

```bash
# Example using the single-header release
sudo apt install nlohmann-json3-dev   # Ubuntu 22.04+ ships 3.x
```

### Building and Installing

Clone the repository and build with CMake:

```bash
git clone https://github.com/GRAAL-Lab/oal.git
cd oal
mkdir build && cd build
cmake ..
make
sudo make install
```

To disable building the test executables:

```bash
cmake -DBUILD_TESTS=OFF ..
```

### Running the Tests

After building with `BUILD_TESTS=ON` (the default):

```bash
cd build
./geometry_test
./local_planner_test
./run_from_logs
```

Or run all registered tests through CTest:

```bash
cd build
ctest --output-on-failure
```

---

## Usage

Include the library headers and link against `oal`:

```cpp
#include "oal/local_planner/generator.hpp"
#include "oal/obstacle.hpp"

// 1. Describe the vehicle
oal::VehicleData vh_data;
vh_data.velocities = {10.0, 5.0};   // available speeds (m/s)

// 2. Set pruning / search parameters
oal::PruningParams pp;
pp.colregsCompliant = true;

// 3. Create the planner
oal::Generator planner(vh_data, pp);

// 4. Define start pose and goal
oal::Pose start({0.0, 0.0}, 0.0);   // position (x,y), heading (rad)
Eigen::Vector2d goal{150.0, 0.0};

// 5. Add obstacles
oal::BoundingBoxData bb;
bb.dim_x = 10.0; bb.dim_y = 3.0;
bb.gain = 3.0; bb.minDistFromObs = 1.0;
bb.reductionWhileCheckingPath = 1.0;
bb.safeMaxGap = 0.0; bb.lookAheadSafetySpan = 0.0;
bb.Set({0,0}, "", 0, 0, 0, 0, 0, 0, 0);
// Parameters: velocity, obs_class, size_x_sigma, size_y_sigma,
//             pose_x_sigma, pose_y_sigma, pose_yaw_sigma, vel_x_sigma, vel_y_sigma
// Pass zeros when uncertainty estimates are not available.

oal::Pose obs_pose({{70.0, 5.0}, 0.0});
auto obs = std::make_shared<oal::Obstacle>("obs1", "", obs_pose, Eigen::Vector2d{0,0}, bb, start.Position());
std::vector<oal::ObsPtr> obstacles{obs};

// 6. Compute the path
oal::AstarPath path;
oal::PathReport report = planner.FindPath(start, goal, obstacles, path);

if (report.result == oal::SearchResult::FOUND) {
    path.Print();   // or iterate path.Data()
}
```

In your `CMakeLists.txt`:

```cmake
find_package(oal REQUIRED)
target_link_libraries(my_target PRIVATE oal)
```

---

## Configuration

### `VehicleData`

| Field | Type | Description |
|---|---|---|
| `velocities` | `std::vector<double>` | Candidate speeds (m/s) the planner may assign to each waypoint. |
| `max_yaw_rate` | `double` | Maximum yaw rate (rad/s). |

### `PruningParams`

| Field | Default | Description |
|---|---|---|
| `colregsCompliant` | `false` | Enforce COLREGS encounter rules during search. |
| `stopSearchIfGoalInBB` | `true` | Stop search when the goal is inside an obstacle's bounding box. |
| `samePositionThreshold` | `0.5` | Minimum distance (m) between two nodes to be considered distinct. |
| `sameTimeThreshold` | `0.1` | Minimum time difference (s) between two nodes to be considered distinct. |
| `timeOut` | `-1` | Search timeout (s). Negative value disables the timeout. |

### `BoundingBoxData`

The bounding box around each obstacle is divided into a *safety* zone (used during planning) and a *max* zone
(used during path validation). Call `BoundingBoxData::Set()` after setting the static fields to compute the
derived dimensions.

---

## Known Limitations

- Vehicle and obstacle speeds/headings are treated as constants throughout the planning horizon.
- Obstacle heading and speed direction are assumed to be equivalent (i.e. no sideways drift).

---

## Contributing

Contributions are welcome! Please follow these steps.

### Development Setup

Build with debug symbols (the default `CMAKE_BUILD_TYPE` is `debug`):

```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
```

### Tests

Add new test cases in `test/` and register them in `CMakeLists.txt` using `add_executable` and `add_test`.
Run tests with:

```bash
cd build && ctest --output-on-failure
```

### Visualisation Scripts

The `scripts/` directory contains Python helpers for plotting planned trajectories and bounding boxes.
Install the Python dependencies and run the notebooks in `scripts/`:

```bash
pip install matplotlib numpy
```

### Pull Request Guidelines

1. Fork the repository and create a feature branch from `main`.
2. Keep commits focused and descriptive.
3. Ensure all existing tests pass before opening a PR.
4. Describe your changes clearly in the PR description, including the motivation and any relevant test results.
