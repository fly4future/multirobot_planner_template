#pragma once

#include "PlannerTemplate/types.h"

namespace planner_template {

/// Convert a GPS point to local meters relative to an origin.
/// Uses flat-earth approximation (accurate within ~10 km of origin).
LocalPoint gps_to_local(GpsPoint p, GpsPoint origin);

/// Convert local meter coordinates back to GPS relative to an origin.
GpsPoint local_to_gps(LocalPoint p, GpsPoint origin);

}  // namespace planner_template
