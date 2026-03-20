#pragma once

#include <memory>

#include "PlannerTemplate/logger.h"
#include "PlannerTemplate/types.h"

namespace planner_template {

/// Abstract base class that the partner implements.
///
/// Lifecycle:
///   1. Construct your planner
///   2. Call initialize() once with a logger
///   3. Call plan() as many times as needed
class PlannerInterface {
public:
  /// One-time setup. Return false if initialization fails.
  virtual bool initialize(std::shared_ptr<Logger> logger) = 0;

  /// Compute paths for all robots. Must be safe to call multiple times.
  virtual PlannerOutput plan(const PlannerInput& input) = 0;

  virtual ~PlannerInterface() = default;
};

}  // namespace planner_template
