#include "my_planner.h"

#include "PlannerTemplate/geo_utils.h"

namespace planner_template {

bool MyPlanner::initialize(std::shared_ptr<Logger> logger) {
  logger_ = logger;
  logger_->info("MyPlanner initialized");
  return true;
}

PlannerOutput MyPlanner::plan(const PlannerInput& input) {
  PlannerOutput output;

  if (input.robots.empty()) {
    output.message = "No robots provided";
    return output;
  }
  if (input.search_area.size() < 3) {
    output.message = "Search area must have at least 3 vertices";
    return output;
  }

  // Use first search area vertex as the local coordinate origin
  const GpsPoint origin = input.search_area.front();

  // Convert search area to local meters
  std::vector<LocalPoint> local_area;
  for (const auto& gps : input.search_area) {
    local_area.push_back(gps_to_local(gps, origin));
  }

  // Convert no-fly zones to local meters
  std::vector<std::vector<LocalPoint>> local_no_fly_zones;
  for (const auto& nfz : input.no_fly_zones) {
    std::vector<LocalPoint> local_nfz;
    for (const auto& gps : nfz.vertices) {
      local_nfz.push_back(gps_to_local(gps, origin));
    }
    local_no_fly_zones.push_back(std::move(local_nfz));
  }

  // TODO: Implement your planning algorithm here.
  //
  // You have:
  //   - local_area:          search area polygon in meters (East/North)
  //   - local_no_fly_zones:  obstacles in meters (East/North)
  //   - input.robots:        robot states (position, battery, etc.)
  //   - input.flight_height: desired altitude (meters AGL)
  //   - input.config_json:   optional planner-specific config
  //
  // Generate a path for each robot:
  for (const auto& robot : input.robots) {
    RobotPath path;
    path.robot_name = robot.name;

    // Example: single waypoint at the robot's current position
    path.waypoints.push_back({
        robot.position.latitude,
        robot.position.longitude,
        input.flight_height,
        0.0});

    // TODO: Replace with your computed waypoints.
    // Remember to convert local results back to GPS:
    //   GpsPoint gps = local_to_gps(local_point, origin);

    output.paths.push_back(std::move(path));
  }

  output.success = true;
  output.message = "MyPlanner: placeholder output";
  return output;
}

}  // namespace planner_template
