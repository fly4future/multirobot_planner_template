# PlannerTemplate — Planner Development Kit

A standalone C++17 library template for developing a multi-robot search/coverage planner. **No ROS dependency** — build and test on any system with a C++17 compiler and CMake.

## Quick Start

```bash
mkdir build && cd build
cmake ..
make

# Run planner on a test scenario
./planner_demo ../test_data/scenario_simple.json

# Export JSON output for visualization
./planner_demo ../test_data/scenario_complex.json --output result.json

# Visualize the result (requires matplotlib: pip install matplotlib)
python3 ../tools/visualize.py result.json

# Or save to an image file
python3 ../tools/visualize.py result.json --save plan.png
```

## Project Layout

```
include/PlannerTemplate/
  types.h               # Input/output data types (GPS coords, waypoints, no-fly zones)
  geo_utils.h           # GPS ↔ local-meters conversion utilities
  logger.h              # Injectable logger interface
  planner_interface.h   # Abstract base class — implement this
src/
  example_planner.*     # Reference implementation (lawn-mower strip pattern)
  main.cpp              # Standalone test harness (loads JSON, runs planner, prints output)
test_data/
  scenario_simple.json  # 2 robots, rectangle area, 1 no-fly zone
  scenario_complex.json # 3 robots, irregular polygon, 2 no-fly zones
tools/
  visualize.py          # Plot search area, no-fly zones, robot positions, and planned paths
```

## How to Implement Your Planner

1. Create a new class that inherits from `planner_template::PlannerInterface`
2. Implement `initialize()` (one-time setup) and `plan()` (compute paths)
3. Add your `.cpp` to `CMakeLists.txt` and update `main.cpp` to use it

### Minimal example

```cpp
#include "PlannerTemplate/planner_interface.h"
#include "PlannerTemplate/geo_utils.h"

class MyPlanner : public planner_template::PlannerInterface {
public:
  bool initialize(std::shared_ptr<planner_template::Logger> logger) override {
    logger_ = logger;
    return true;
  }

  planner_template::PlannerOutput plan(const planner_template::PlannerInput& input) override {
    planner_template::PlannerOutput output;

    // Use the GPS origin (first vertex) for local coordinate conversion
    auto origin = input.search_area.front();

    for (const auto& robot : input.robots) {
      planner_template::RobotPath path;
      path.robot_name = robot.name;

      // Convert GPS to local meters, run your algorithm, convert back
      for (const auto& gps : input.search_area) {
        auto local = planner_template::gps_to_local(gps, origin);
        // ... your algorithm here ...
        auto gps_out = planner_template::local_to_gps(local, origin);
        path.waypoints.push_back({gps_out.latitude, gps_out.longitude,
                                  input.flight_height, 0.0});
      }

      // Don't forget to handle input.no_fly_zones in your algorithm!

      output.paths.push_back(std::move(path));
    }

    output.success = true;
    output.message = "OK";
    return output;
  }

private:
  std::shared_ptr<planner_template::Logger> logger_;
};
```

## Input/Output Types

### Input (`PlannerInput`)

| Field           | Type                   | Description                                      |
|-----------------|------------------------|--------------------------------------------------|
| `robots`        | `vector<RobotInfo>`    | Current state of each robot                      |
| `search_area`   | `vector<GpsPoint>`     | Polygon vertices (WGS84 lat/lon)                 |
| `no_fly_zones`  | `vector<NoFlyZone>`    | Areas to avoid (obstacles, restricted airspace)  |
| `flight_height` | `double`               | Flight altitude in meters AGL                    |
| `config_json`   | `string`               | Optional JSON string for planner-specific config |

### Robot (`RobotInfo`)

| Field               | Type       | Description                  |
|---------------------|------------|------------------------------|
| `name`              | `string`   | Robot identifier             |
| `position`          | `GpsPoint` | Current GPS position         |
| `altitude`          | `double`   | Current altitude (m AGL)     |
| `heading`           | `double`   | Current heading (deg, 0=N)   |
| `battery_remaining` | `double`   | Battery fraction (0.0–1.0)   |

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

Each `Waypoint` in a path has: `latitude`, `longitude`, `altitude`, `heading`.

## GPS Utilities

The `geo_utils.h` header provides flat-earth GPS ↔ local-meters conversion (accurate within ~10 km of origin):

```cpp
// Convert GPS → local meters (relative to origin)
LocalPoint local = gps_to_local(gps_point, origin);

// Convert local meters → GPS
GpsPoint gps = local_to_gps(local_point, origin);
```

**Recommended workflow:** receive GPS inputs → convert to local meters → run your algorithm in meters → convert back to GPS for output.

## Visualization

The `tools/visualize.py` script plots the planner output for visual validation.

**Requirements:** `pip install matplotlib`

**Usage:**
```bash
# Run the planner with --output to get JSON
./build/planner_demo test_data/scenario_complex.json --output result.json

# Show interactive plot
python3 tools/visualize.py result.json

# Save to image file
python3 tools/visualize.py result.json --save plan.png
```

The plot shows:
- **Gray polygon** — search area boundary
- **Red hatched polygons** — no-fly zones (with labels)
- **Stars** — robot start positions
- **Colored lines** — planned paths per robot (square = start, triangle = end)

## Test Scenarios

- **`scenario_simple.json`** — 2 robots, rectangular search area (~150m × 110m) near Zurich, 1 no-fly zone (building)
- **`scenario_complex.json`** — 3 robots, irregular 5-vertex polygon, 2 no-fly zones (tower + restricted area), custom config

## JSON Scenario Format

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
