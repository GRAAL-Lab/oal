# OAL - Obstacle Avoidance Library

The library computes COLREGS-compliant trajectories to a goal while avoiding static and moving obstacles.

---

## Overview

OAL is a C++ obstacle avoidance library designed as a reusable shared library. It includes:

- a local planner implementation (`Generator`, path/node/path evaluator logic),
- geometric and obstacle modeling utilities,
- test executables used as integration references,
- plotting/logging helpers for development-time inspection.

## Highlights

- Computes COLREGS-compliant trajectories
- Supports static and moving obstacles
- Expands reachable nodes with multiple candidate velocities
- Exposes a shared C++ library target with CMake install/export support
- Includes test and debug/logging hooks for inspecting paths and obstacles

## News

- Every reachable node is expanded with a set of different candidate velocities, producing mixed-velocity trajectories
  (sometimes a slower segment leads to a faster overall solution)
- ..

## Repository layout

The current repository structure is:

- `src/` - library implementation sources
  - `src/local_planner/generator.cpp`: main planner orchestration (`FindPath`, path validity checks, search flow)
  - `src/local_planner/node.cpp`: node and search-node behavior/cost logic
  - `src/local_planner/path_evaluator.cpp`: path scoring/evaluation and related heuristics
  - `src/obstacle.cpp`: obstacle state and obstacle geometry behavior
  - `src/geometric_utilities.cpp`: geometric helper implementations used by planner and obstacle logic

- `include/oal/` - public C++ headers installed for consumers
  - `data_structs.hpp`: shared planner/obstacle/configuration data types
  - `obstacle.hpp`: obstacle model interface
  - `geometric_utilities.hpp`: geometry API
  - `oal_defines.hpp`: common definitions/types/macros used by public interfaces
  - `local_planner/`: public local planner interfaces
    - `generator.hpp`: main integration entry point (`oal::Generator`)
    - `node.hpp`, `path.hpp`, `path_evaluator.hpp`: local planner data structures and interfaces

- `test/` - test/integration entry points
  - `test/geometry.cc`: geometry-focused checks
  - `test/local_planner/first.cc`: primary local planner integration/random scenario exerciser
  - `test/local_planner/runFromLog.cc`: replay/debug runner that loads planner state from JSON logs

- `scripts/` - plotting and log analysis utilities
  - Python helpers for trace/obstacle plotting (for example `plotTracesWithObs.py`, `plot_functions.py`,
    `trace_functions.py`, `obs_functions.py`)
  - `scripts/old/`: legacy MATLAB utilities kept for historical/debug usage

- `logs/` - local log output workspace used during development/testing
  - includes sample/debug artifacts such as `nodes.txt`
  - additional files (for example obstacle logs) can be generated at runtime by debug settings

## Public headers

The installed public interface is under `include/oal`:

- `oal/data_structs.hpp`
- `oal/geometric_utilities.hpp`
- `oal/oal_defines.hpp`
- `oal/obstacle.hpp`
- `oal/local_planner/generator.hpp`
- `oal/local_planner/node.hpp`
- `oal/local_planner/path.hpp`
- `oal/local_planner/path_evaluator.hpp`

The main integration entry point is `oal::Generator`, which exposes path generation and validation methods.

## Building and installing

OAL uses CMake.

```bash
cmake -S . -B build
cmake --build build
cmake --install build
```

### Dependencies

From the current build configuration and public headers, OAL requires:

- CMake 3.2+
- a C++17-compatible compiler
- `nlohmann_json` 3.11.3
- Boost components: `filesystem`, `system`
- Eigen headers (`eigen3`) used by public APIs and tests

To configure test targets explicitly:

```bash
cmake -S . -B build -DBUILD_TESTS=ON
```

## Usage

### 1. Link OAL from CMake

After installing the library, consume the exported package and link against `oal`:

```cmake
find_package(oal REQUIRED)
add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE oal)
```

If consuming directly from source, use:

```cmake
add_subdirectory(path/to/oal)
add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE oal)
```

### 2. Include planner headers

```cpp
#include "oal/local_planner/generator.hpp"
#include "oal/obstacle.hpp"
#include "oal/geometric_utilities.hpp"
```

### 3. Typical integration flow

1. Create `VehicleData` (including candidate velocities).
2. Create `PruningParams` for search/pruning behavior.
3. Optionally create `DebugSettings` for diagnostics/logging.
4. Define start pose (`Pose`) and goal (`Eigen::Vector2d`).
5. Build `std::vector<ObsPtr>` with perceived obstacles.
6. Create `oal::Generator` and call `FindPath(...)`.

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

### 5. Use tests as reference integrations

- `test/local_planner/first.cc`: planner setup, randomized scenarios, and obstacle creation
- `test/local_planner/runFromLog.cc`: log replay/debug-oriented local planner flow
- `test/geometry.cc`: geometry-related checks

## Plots

To inspect collision behavior and trajectory geometry, plotting scripts are provided in `scripts/`.
Example output:
![Figure_1](https://github.com/SamueleD98/oal/assets/28822110/34b667d5-8ca8-4d2a-bca5-49d13a8e3098)

## Notes

For now, obstacle attributes are kept public to simplify plotting/debug inspection. In a more finalized API, these may
be made private.

## Current limits

- Vehicle and obstacle speed/heading are treated as constants
- Obstacle heading and speed direction are considered equivalent
- ..
