#pragma once

#include <string>
#include <vector>

namespace planner_template {

/// GPS coordinate (WGS84)
struct GpsPoint {
  double latitude  = 0.0;
  double longitude = 0.0;
};

/// Local coordinate in meters relative to an origin
struct LocalPoint {
  double x = 0.0;
  double y = 0.0;
};

/// 4-DOF output waypoint
struct Waypoint {
  double latitude  = 0.0;
  double longitude = 0.0;
  double altitude  = 0.0;  // meters AGL
  double heading   = 0.0;  // degrees, 0=North, clockwise
};

/// Robot state information
struct RobotInfo {
  std::string name;
  GpsPoint    position;
  double      altitude          = 0.0;  // meters AGL
  double      heading           = 0.0;  // degrees
  double      battery_remaining = 1.0;  // 0.0–1.0
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
  double                  flight_height = 30.0;  // meters AGL
  std::string             config_json;    // optional planner-specific config
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
