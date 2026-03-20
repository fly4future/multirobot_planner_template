#pragma once

#include <memory>
#include <string>

namespace planner_template {

/// Injectable logger interface.
/// The IROC ROS wrapper will provide an implementation that forwards to RCLCPP logging.
/// The standalone harness provides a simple stdout logger.
class Logger {
public:
  virtual void info(const std::string& msg)  = 0;
  virtual void warn(const std::string& msg)  = 0;
  virtual void error(const std::string& msg) = 0;
  virtual ~Logger() = default;
};

/// Simple stdout/stderr logger for standalone use
class StdoutLogger : public Logger {
public:
  void info(const std::string& msg) override;
  void warn(const std::string& msg) override;
  void error(const std::string& msg) override;
};

}  // namespace planner_template
