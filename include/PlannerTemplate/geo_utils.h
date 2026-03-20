#pragma once

#include "PlannerTemplate/types.h"

namespace planner_template {

/// Convert a GPS point to local ENU (East-North-Up) meters relative to an origin.
/// Uses WGS84 ellipsoidal model via GeographicLib (accurate at any distance).
/// Returns: x = East (meters), y = North (meters).
LocalPoint gps_to_local(GpsPoint p, GpsPoint origin);

/// Convert local ENU meter coordinates back to GPS relative to an origin.
/// Returns: latitude, longitude (WGS84).
GpsPoint local_to_gps(LocalPoint p, GpsPoint origin);

}  // namespace planner_template
