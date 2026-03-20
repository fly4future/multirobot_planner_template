#include "PlannerTemplate/logger.h"

#include <iostream>

namespace planner_template {

void StdoutLogger::info(const std::string& msg) {
  std::cout << "[INFO]  " << msg << "\n";
}

void StdoutLogger::warn(const std::string& msg) {
  std::cout << "[WARN]  " << msg << "\n";
}

void StdoutLogger::error(const std::string& msg) {
  std::cerr << "[ERROR] " << msg << "\n";
}

}  // namespace planner_template
