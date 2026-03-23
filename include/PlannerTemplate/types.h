#pragma once

#include <string>
#include <vector>

namespace planner_template {

// ---------------------------------------------------------------------------
// Conventions:
//   Altitude: ALL altitude values are in meters Above Ground Level (AGL).
//             The flight controller handles AMSL conversion at runtime.
//   Heading:  ALL heading values are in radians, 0 = East, counter-clockwise.
//             (i.e. π/2 = North, π = West, 3π/2 = South)
// ---------------------------------------------------------------------------

/// GPS coordinate (WGS84)
struct GpsPoint {
  double latitude  = 0.0;
  double longitude = 0.0;
};

/// Local coordinate in meters relative to an origin
struct LocalPoint {
  double x = 0.0;  // East  (meters)
  double y = 0.0;  // North (meters)
};

/// 4-DOF output waypoint
struct Waypoint {
  double latitude  = 0.0;
  double longitude = 0.0;
  double altitude  = 0.0;  // meters AGL — height above ground at this waypoint
  double heading   = 0.0;  // radians, 0=East, counter-clockwise
};

/// Robot state information
struct RobotInfo {
  std::string name;
  GpsPoint    position;
  double      altitude          = 0.0;  // meters AGL — current height above ground
  double      heading           = 0.0;  // radians, 0=East, counter-clockwise
  double      battery_remaining = 1.0;  // fraction 0.0–1.0
};

/// A polygonal zone to avoid (obstacle, restricted airspace, etc.)
struct NoFlyZone {
  std::string             name;           // optional label (e.g. "building_A")
  std::vector<GpsPoint>   vertices;       // polygon vertices (closed loop)
};

/// Complete planner input
struct PlannerInput {
  std::vector<RobotInfo>  robots;
  std::vector<GpsPoint>   search_area;    // polygon vertices (closed loop)
  std::vector<NoFlyZone>  no_fly_zones;   // areas to avoid
  double                  flight_height = 30.0;  // meters AGL — desired planning altitude
  std::string             config_json;    // optional planner-specific config (JSON string)
};

/// Per-robot output path
struct RobotPath {
  std::string          robot_name;
  std::vector<Waypoint> waypoints;
};

/// Complete planner output
struct PlannerOutput {
  bool                    success = false;
  std::string             message;
  std::vector<RobotPath>  paths;
};

}  // namespace planner_template
