#pragma once

#include "PlannerTemplate/planner_interface.h"

namespace planner_template {

/// Reference implementation: splits the search area bounding box into
/// N vertical strips (one per robot) and generates a lawn-mower sweep
/// pattern within each strip at the requested flight height.
class ExamplePlanner : public PlannerInterface {
public:
  bool initialize(std::shared_ptr<Logger> logger) override;
  PlannerOutput plan(const PlannerInput& input) override;

private:
  std::shared_ptr<Logger> logger_;
};

}  // namespace planner_template
