#include "PlannerTemplate/geo_utils.h"

#include <GeographicLib/LocalCartesian.hpp>

namespace planner_template {

LocalPoint gps_to_local(GpsPoint p, GpsPoint origin) {
  GeographicLib::LocalCartesian proj(origin.latitude, origin.longitude, 0.0);

  double x, y, z;
  proj.Forward(p.latitude, p.longitude, 0.0, x, y, z);

  // LocalCartesian returns (x=East, y=North, z=Up)
  return {x, y};
}

GpsPoint local_to_gps(LocalPoint p, GpsPoint origin) {
  GeographicLib::LocalCartesian proj(origin.latitude, origin.longitude, 0.0);

  double lat, lon, h;
  proj.Reverse(p.x, p.y, 0.0, lat, lon, h);

  return {lat, lon};
}

}  // namespace planner_template
