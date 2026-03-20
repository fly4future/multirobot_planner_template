#include "PlannerTemplate/geo_utils.h"

#include <cmath>

namespace planner_template {

static constexpr double METERS_IN_DEGREE = 111319.5;

LocalPoint gps_to_local(GpsPoint p, GpsPoint origin) {
  const double meters_in_lon_degree =
      std::cos((origin.latitude / 180.0) * M_PI) * METERS_IN_DEGREE;

  return {
      (p.longitude - origin.longitude) * meters_in_lon_degree,
      (p.latitude - origin.latitude) * METERS_IN_DEGREE};
}

GpsPoint local_to_gps(LocalPoint p, GpsPoint origin) {
  const double meters_in_lon_degree =
      std::cos((origin.latitude / 180.0) * M_PI) * METERS_IN_DEGREE;

  return {
      origin.latitude + p.y / METERS_IN_DEGREE,
      origin.longitude + p.x / meters_in_lon_degree};
}

}  // namespace planner_template
