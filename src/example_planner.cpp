#include "example_planner.h"

#include <algorithm>
#include <limits>

#include "PlannerTemplate/geo_utils.h"

namespace planner_template {

bool ExamplePlanner::initialize(std::shared_ptr<Logger> logger) {
  logger_ = logger;
  logger_->info("ExamplePlanner initialized");
  return true;
}

PlannerOutput ExamplePlanner::plan(const PlannerInput& input) {
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
  local_area.reserve(input.search_area.size());
  for (const auto& gps : input.search_area) {
    local_area.push_back(gps_to_local(gps, origin));
  }

  // Compute bounding box
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto& p : local_area) {
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
  }

  const int n_robots = static_cast<int>(input.robots.size());
  const double strip_width = (max_x - min_x) / n_robots;
  const double sweep_step = 20.0;  // meters between sweep lines

  logger_->info("Planning for " + std::to_string(n_robots) + " robots, area " +
                std::to_string(max_x - min_x) + "x" + std::to_string(max_y - min_y) + " m");

  /// Just for demonstration, we log the no-fly zones but do not actually handle them in this simple planner.
  if (!input.no_fly_zones.empty()) {
    logger_->warn("Note: " + std::to_string(input.no_fly_zones.size()) +
                  " no-fly zone(s) provided but not handled by ExamplePlanner");
    for (const auto& nfz : input.no_fly_zones) {
      logger_->warn("  No-fly zone: " + (nfz.name.empty() ? "(unnamed)" : nfz.name) +
                    " (" + std::to_string(nfz.vertices.size()) + " vertices)");
    }
  }

   /* NOTE: HERE IS WHERE YOUR CORE PLANNING ALGORITHM WOULD GO.
    * The following is just a simple lawn-mower pattern for demonstration purposes.
   */

  // Simple lawn-mower pattern: divide area into vertical strips and sweep along Y axis
  // Generate lawn-mower pattern for each robot's strip
  for (int i = 0; i < n_robots; ++i) {
    RobotPath path;
    path.robot_name = input.robots[i].name;

    const double x_lo = min_x + i * strip_width;
    const double x_hi = x_lo + strip_width;

    // Sweep lines along Y axis within the strip
    bool forward = true;
    for (double x = x_lo; x <= x_hi; x += sweep_step) {
      LocalPoint p1{x, forward ? min_y : max_y};
      LocalPoint p2{x, forward ? max_y : min_y};

      auto gps1 = local_to_gps(p1, origin);
      auto gps2 = local_to_gps(p2, origin);

      path.waypoints.push_back({gps1.latitude, gps1.longitude, input.flight_height, 0.0});
      path.waypoints.push_back({gps2.latitude, gps2.longitude, input.flight_height, 0.0});

      forward = !forward;
    }

    logger_->info("  " + path.robot_name + ": " +
                  std::to_string(path.waypoints.size()) + " waypoints");
    output.paths.push_back(std::move(path));
  }

  output.success = true;
  output.message = "Planned " + std::to_string(n_robots) + " robot paths";
  return output;
}

}  // namespace planner_template
