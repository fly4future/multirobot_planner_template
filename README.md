# PlannerTemplate — Multi-Robot Planner Development Kit

A standalone C++17 library template for developing a multi-robot search/coverage planner. **No ROS dependency** — build and test on any system with a C++17 compiler and CMake.

## Quick Start (with Pixi)

[Pixi](https://pixi.sh) manages all dependencies (compiler, CMake, GeographicLib, Python, matplotlib) automatically across Linux, macOS, and Windows.

```bash
# Install pixi (one-time)
curl -fsSL https://pixi.sh/install.sh | sh

# Build the project
pixi run build

# Run the example planner and visualize
pixi run demo-simple
pixi run show
```

## Quick Start (without Pixi)

If you prefer to manage dependencies yourself, you need: a C++17 compiler, CMake ≥ 3.14, and Python 3 with matplotlib for visualization. GeographicLib is fetched automatically by CMake if not installed.

```bash
mkdir build && cd build
cmake ..
make
./planner_demo ../test_data/scenario_simple.json --output result.json
python3 ../tools/visualize.py result.json
```

## Pixi Tasks

All tasks are defined in `pixi.toml`. Run them with `pixi run <task>`.

| Task | Description |
|------|-------------|
| `build` | Configure + compile the project |
| `clean` | Remove build directory |
| `demo-simple` | Run example planner on simple scenario |
| `demo-complex` | Run example planner on complex scenario |
| `plan-simple` | Run **your** planner on simple scenario |
| `plan-complex` | Run **your** planner on complex scenario |
| `show` | Open interactive plot of last result |
| `viz` | Save plot of last result to `build/plan.png` |

All `demo-*` and `plan-*` tasks write to `build/result.json`, so `show` and `viz` always visualize the most recent run.

### Typical workflow

```bash
# 1. Edit your planner in src/my_planner.cpp

# 2. Run it on a scenario
pixi run plan-simple

# 3. Visualize the result
pixi run show

# 4. Compare against the example planner
pixi run demo-simple
pixi run show
```

## Project Layout

```
include/PlannerTemplate/
  types.h               # Input/output data types (GPS coords, waypoints, no-fly zones)
  geo_utils.h           # GPS ↔ local-meters conversion (GeographicLib)
  logger.h              # Injectable logger interface
  planner_interface.h   # Abstract base class — implement this
src/
  example_planner.*     # Reference implementation (lawn-mower strip pattern)
  my_planner.*          # Skeleton for your planner — start here
  main.cpp              # Test harness with planner registry
test_data/
  scenario_simple.json  # 2 robots, rectangle area, 1 no-fly zone
  scenario_complex.json # 3 robots, irregular polygon, 2 no-fly zones
tools/
  visualize.py          # Plot search area, no-fly zones, and planned paths
```

## How to Implement Your Planner

A skeleton is provided in `src/my_planner.h` and `src/my_planner.cpp`. It's already registered in the planner registry in `main.cpp` and compiles out of the box. Just fill in your algorithm.

If you want to add a completely new planner:

1. Create your class inheriting from `planner_template::PlannerInterface`
2. Add your `.cpp` to `CMakeLists.txt`
3. Register it in `main.cpp` by adding one line to the `PLANNERS` map:
   ```cpp
   {"your_planner", [] { return std::make_unique<YourPlanner>(); }},
   ```
4. Run it: `pixi run build && build/planner_demo test_data/scenario_simple.json --planner your_planner`

### Planner interface

```cpp
class PlannerInterface {
public:
  virtual bool initialize(std::shared_ptr<Logger> logger) = 0;
  virtual PlannerOutput plan(const PlannerInput& input) = 0;
  virtual ~PlannerInterface() = default;
};
```

### Demo executable

```bash
# Default planner (example)
./build/planner_demo test_data/scenario_simple.json

# Select a specific planner
./build/planner_demo test_data/scenario_simple.json --planner my_planner

# Save JSON output for visualization
./build/planner_demo test_data/scenario_simple.json --planner my_planner --output build/result.json
```

## Input/Output Types

> **Altitude convention:** All altitude values are in meters Above Ground Level (AGL).

### Input (`PlannerInput`)

| Field           | Type                   | Description                                      |
|-----------------|------------------------|--------------------------------------------------|
| `robots`        | `vector<RobotInfo>`    | Current state of each robot                      |
| `search_area`   | `vector<GpsPoint>`     | Polygon vertices (WGS84 lat/lon)                 |
| `no_fly_zones`  | `vector<NoFlyZone>`    | Areas to avoid (obstacles, restricted airspace)  |
| `flight_height` | `double`               | Desired planning altitude (meters AGL)           |
| `config_json`   | `string`               | Optional JSON string for planner-specific config |

### Robot (`RobotInfo`)

| Field               | Type       | Description                    |
|---------------------|------------|--------------------------------|
| `name`              | `string`   | Robot identifier               |
| `position`          | `GpsPoint` | Current GPS position           |
| `altitude`          | `double`   | Current height above ground (m AGL) |
| `heading`           | `double`   | Current heading (deg, 0=N, clockwise) |
| `battery_remaining` | `double`   | Battery fraction (0.0–1.0)     |

### No-Fly Zone (`NoFlyZone`)

| Field      | Type               | Description                           |
|------------|--------------------|---------------------------------------|
| `name`     | `string`           | Optional label (e.g. "building_A")    |
| `vertices` | `vector<GpsPoint>` | Polygon vertices defining the zone    |

### Output (`PlannerOutput`)

| Field     | Type                 | Description                    |
|-----------|----------------------|--------------------------------|
| `success` | `bool`               | `true` if planning succeeded   |
| `message` | `string`             | Status or error description    |
| `paths`   | `vector<RobotPath>`  | One path per robot             |

Each `Waypoint` has: `latitude`, `longitude`, `altitude` (m AGL), `heading` (deg).

## GPS Utilities

The `geo_utils.h` header converts between GPS coordinates and local Cartesian coordinates (meters) relative to a specified origin point. Uses [GeographicLib](https://geographiclib.sourceforge.io/) for accurate WGS84 ellipsoidal calculations.

```cpp
// Convert GPS → local meters (x=East, y=North relative to origin)
LocalPoint local = gps_to_local(gps_point, origin);

// Convert local meters → GPS
GpsPoint gps = local_to_gps(local_point, origin);
```

**Recommended workflow:** receive GPS inputs → convert to local meters → run your algorithm in meters → convert back to GPS for output.

## Visualization

The `tools/visualize.py` script plots the planner output for visual validation.

```bash
# With pixi (matplotlib included automatically)
pixi run show              # interactive plot of last result
pixi run viz               # save to build/plan.png

# Without pixi (requires: pip install matplotlib)
python3 tools/visualize.py build/result.json
python3 tools/visualize.py build/result.json --save plan.png
```

The plot shows:
- **Gray polygon** — search area boundary
- **Red hatched polygons** — no-fly zones (with labels)
- **Stars** — robot start positions
- **Colored lines** — planned paths per robot (square = start, triangle = end)

## Test Scenarios

- **`scenario_simple.json`** — 2 robots, rectangular search area (~150m × 110m), 1 no-fly zone (building)
- **`scenario_complex.json`** — 3 robots, irregular 5-vertex polygon, 2 no-fly zones (tower + restricted area), custom config

### JSON scenario format

```json
{
  "robots": [
    { "name": "uav1", "latitude": 47.39, "longitude": 8.54, "battery_remaining": 0.95 }
  ],
  "search_area": [
    { "latitude": 47.39, "longitude": 8.54 },
    { "latitude": 47.39, "longitude": 8.55 },
    { "latitude": 47.40, "longitude": 8.55 }
  ],
  "no_fly_zones": [
    {
      "name": "obstacle_1",
      "vertices": [
        { "latitude": 47.393, "longitude": 8.543 },
        { "latitude": 47.393, "longitude": 8.545 },
        { "latitude": 47.395, "longitude": 8.545 },
        { "latitude": 47.395, "longitude": 8.543 }
      ]
    }
  ],
  "flight_height": 30.0,
  "config": { "your_custom_key": "value" }
}
```
