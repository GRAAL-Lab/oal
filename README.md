# OAL - Obstacle Avoidance Library

The library computes a colregs-compliant trajectory to a goal, avoiding static and moving obstacles.

---

## Overview

OAL is a C++ obstacle avoidance library focused on generating trajectories toward a goal while accounting for both static and dynamic obstacles. The project is organized as a reusable library, with tests and plotting/logging utilities that help inspect planner behavior during development.

## Highlights

- Computes colregs-compliant trajectories
- Supports static and moving obstacles
- Expands reachable nodes with multiple candidate velocities
- Ships as a shared C++ library with CMake install/export support
- Includes tests and debug/logging hooks for inspecting paths and obstacles

## News

- Every reachable node is expanded with a set of (given) different velocities: the result is a trajectory with mixed
  velocities (sometimes slower gets to the goal sooner)
- ..

## Repository layout

### Core library

- `include/oal/data_structs.hpp`
  - Core shared data types used across the library.
  - Defines `Pose`, `VehicleData`, `EncounterData`, `PruningParams`, `PathReport`, and `BoundingBoxData`.
  - Also provides shared aliases such as `NodePtr`, `ObsPtr`, `NodeSet`, `TimeDouble`, and obstacle vertex IDs via `VxId`.
- `include/oal/obstacle.hpp` / `src/obstacle.cpp`
  - Obstacle model and obstacle geometry utilities.
  - Stores obstacle identity, class, initial pose, velocity, bounding-box data, and obstacle vertices.
  - Exposes helpers to query obstacle position over time, retrieve obstacle vertices, and emit debug/log output.
- `include/oal/geometric_utilities.hpp` / `src/geometric_utilities.cpp`
  - Shared geometric helpers used by the planner and collision checks.
- `include/oal/oal_defines.hpp`
  - Common library-wide defines and constants.

### Local planner

- `include/oal/local_planner/generator.hpp` / `src/local_planner/generator.cpp`
  - Main planner entry point.
  - Defines `oal::Generator`, which owns vehicle capabilities, pruning parameters, debug settings, and the search flow.
  - Public API includes `FindPath(...)`, `IsPathValid(...)`, and `IsInBB(...)`.
- `include/oal/local_planner/node.hpp` / `src/local_planner/node.cpp`
  - A* search node representation.
  - Defines `oal::AStarNode`, which stores encounter data, costs, parent links, goal/reached flags, and trace/debug helpers.
- `include/oal/local_planner/path.hpp`
  - Path container returned by the planner.
  - Defines `oal::AstarPath`, a list-backed wrapper around planner nodes with path length, print, and trace logging helpers.
- `include/oal/local_planner/path_evaluator.hpp` / `src/local_planner/path_evaluator.cpp`
  - Path and motion validation utilities.
  - Provides collision checking and COLREGs-related rule evaluation for candidate motions.

### Tests and developer utilities

- `test/geometry.cc`
  - Geometry-focused test program.
- `test/local_planner/first.cc`
  - Example/local planner test scenario for building obstacles, configuring the generator, and running a search.
- `test/local_planner/runFromLog.cc`
  - Replay-oriented planner test that runs scenarios from logged data.
- `scripts/`
  - Python and notebook-based utilities for plotting traces, plotting obstacles, and development experimentation.
  - Includes `obs_functions.py`, `plotTracesWithObs.py`, `plot_functions.py`, `trace_functions.py`, and Jupyter notebooks.
- `logs/`
  - Output directory used by debug logging and scenario inspection workflows.

## Public headers

The library's public interface is exposed from `include/oal`, notably:

- `oal/data_structs.hpp`
- `oal/geometric_utilities.hpp`
- `oal/obstacle.hpp`
- `oal/local_planner/generator.hpp`
- `oal/local_planner/node.hpp`
- `oal/local_planner/path.hpp`
- `oal/local_planner/path_evaluator.hpp`


## Building and installing

The build tool used for this project is CMake. To build and install the project navigate to the root of the cloned repo
and execute the following commands:

```bash
mkdir build
cd build
cmake ..
sudo make install

## Building and installing

The build tool used for this project is CMake. To build and install the project navigate to the root of the cloned repo
and execute the following commands:

```bash
mkdir build
cd build
cmake ..
sudo make install
```

### Dependencies

From the build configuration, OAL currently requires:

- CMake 3.2+
- A C++17-compatible compiler
- `nlohmann_json` 3.11.3
- Boost components: `filesystem`, `system`

## Usage

### 1. Link OAL from CMake

After installing the library, you can integrate it into another CMake project by including the exported package config and linking against `oal`.

```cmake
find_package(oal REQUIRED)
add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE oal)
```

If you are consuming OAL from the source tree instead of an installed location, you can also add it as a subdirectory and link the same target:

```cmake
add_subdirectory(path/to/oal)
add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE oal)
```

### 2. Include the planner headers

A minimal consumer will typically include the generator and obstacle-related headers:

```cpp
#include "oal/local_planner/generator.hpp"
#include "oal/obstacle.hpp"
#include "oal/geometric_utilities.hpp"
```

### 3. Build the planner inputs

From the existing test program, the typical integration flow is:

1. Create `VehicleData` with the vehicle capabilities, including candidate velocities.
2. Create `PruningParams` to configure exploration behavior.
3. Optionally create `DebugSettings` to enable logs and planner diagnostics.
4. Define the vehicle start pose as `Pose` and the target goal as `Eigen::Vector2d`.
5. Build a `std::vector<ObsPtr>` containing the perceived obstacles.
6. Create an `oal::Generator` and call `FindPath(...)`.

### 4. Minimal integration example

```cpp
#include <memory>
#include <vector>
#include <eigen3/Eigen/Eigen>

#include "oal/local_planner/generator.hpp"
#include "oal/obstacle.hpp"

int main() {
    using namespace oal;

    VehicleData vehicle;
    vehicle.velocities = {10, 1};

    PruningParams pruning;
    auto debug = std::make_shared<DebugSettings>();

    Pose start({0, 0}, 0.6);
    Eigen::Vector2d goal{150, 0};

    std::vector<ObsPtr> obstacles;
    BoundingBoxData bb;
    bb.dim_x = 10;
    bb.dim_y = 3;
    bb.gain = 3;
    bb.minDistFromObs = 1;
    bb.reductionWhileCheckingPath = 1;
    bb.safeMaxGap = 0;
    bb.lookAheadSafetySpan = 0;
    bb.Set({0, 0}, "", 0, 0, 0, 0, 0, 0, 0);

    Pose obstacle_pose{{70, 5}, 0};
    auto obstacle = std::make_shared<Obstacle>(
        Obstacle("obs1", "", obstacle_pose, {0, 0}, bb, start.Position())
    );
    obstacles.push_back(obstacle);

    Generator generator(vehicle, pruning, debug);
    AstarPath path;
    auto report = generator.FindPath(start, goal, obstacles, path);

    return report.result == SearchResult::FOUND ? 0 : 1;
}
```

### 5. Use the tests as reference integrations

The following test programs are useful references when integrating the library:

- `test/local_planner/first.cc`: basic planner setup, random scenarios, and obstacle creation
- `test/local_planner/runFromLog.cc`: replay-oriented local planner test flow
- `test/geometry.cc`: geometry-related checks

These files show how to initialize `VehicleData`, `BoundingBoxData`, `Pose`, debug settings, and obstacle collections in a realistic workflow.

## Plots

In order to check the correctness of the CheckCollision function, I wrote a script to visualize the bounding boxes and
the vehicle path. Here an example:
![Figure_1](https://github.com/SamueleD98/oal/assets/28822110/34b667d5-8ca8-4d2a-bca5-49d13a8e3098)

## Notes

For now, I'm keeping obstacle attributes as public since I need them to be easily accessible for the plots. In the final
version, I will set them as private.

## Current limits

- Vehicle and obstacles speed/heading are considered constants
- Obstacles heading and speed are equivalent
- ..
