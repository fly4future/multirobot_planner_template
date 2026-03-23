#pragma once

#include "PlannerTemplate/planner_interface.h"

namespace planner_template {

/// Skeleton planner — replace this with your implementation.
/// See example_planner.h/.cpp for a working reference.
class MyPlanner : public PlannerInterface {
public:
  bool initialize(std::shared_ptr<Logger> logger) override;
  PlannerOutput plan(const PlannerInput& input) override;

private:
  std::shared_ptr<Logger> logger_;
};

}  // namespace planner_template
