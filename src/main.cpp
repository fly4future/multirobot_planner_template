#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>

#include "PlannerTemplate/logger.h"
#include "PlannerTemplate/planner_interface.h"
#include "PlannerTemplate/types.h"

// ── Planner includes ────────────────────────────────────────────────────────
#include "example_planner.h"
#include "my_planner.h"
// Add your planner header here

using json = nlohmann::json;
using namespace planner_template;

// ── Planner registry ────────────────────────────────────────────────────────
// To register a new planner, add an entry mapping its name to a factory lambda.
using PlannerFactory = std::function<std::unique_ptr<PlannerInterface>()>;

static const std::map<std::string, PlannerFactory> PLANNERS = {
    {"example",    [] { return std::make_unique<ExamplePlanner>(); }},
    {"my_planner", [] { return std::make_unique<MyPlanner>(); }},
    // {"your_planner", [] { return std::make_unique<YourPlanner>(); }},
};

static constexpr const char* DEFAULT_PLANNER = "example";

// ── JSON parsing helpers ────────────────────────────────────────────────────

static PlannerInput parse_scenario(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    throw std::runtime_error("Cannot open file: " + path);
  }
  json j = json::parse(ifs);

  PlannerInput input;

  // Parse robots
  for (const auto& r : j.at("robots")) {
    RobotInfo ri;
    ri.name              = r.at("name").get<std::string>();
    ri.position.latitude = r.at("latitude").get<double>();
    ri.position.longitude = r.at("longitude").get<double>();
    ri.altitude          = r.value("altitude", 0.0);
    ri.heading           = r.value("heading", 0.0);
    ri.battery_remaining = r.value("battery_remaining", 1.0);
    input.robots.push_back(ri);
  }

  // Parse search area polygon
  for (const auto& pt : j.at("search_area")) {
    GpsPoint gp;
    gp.latitude  = pt.at("latitude").get<double>();
    gp.longitude = pt.at("longitude").get<double>();
    input.search_area.push_back(gp);
  }

  // Parse no-fly zones (optional)
  if (j.contains("no_fly_zones")) {
    for (const auto& nfz : j["no_fly_zones"]) {
      NoFlyZone zone;
      zone.name = nfz.value("name", "");
      for (const auto& pt : nfz.at("vertices")) {
        GpsPoint gp;
        gp.latitude  = pt.at("latitude").get<double>();
        gp.longitude = pt.at("longitude").get<double>();
        zone.vertices.push_back(gp);
      }
      input.no_fly_zones.push_back(zone);
    }
  }

  input.flight_height = j.value("flight_height", 30.0);

  if (j.contains("config")) {
    input.config_json = j["config"].dump();
  }

  return input;
}

static json output_to_json(const PlannerInput& input, const PlannerOutput& output) {
  json j;

  // Include input for visualization context
  json j_area = json::array();
  for (const auto& gp : input.search_area) {
    j_area.push_back({{"latitude", gp.latitude}, {"longitude", gp.longitude}});
  }
  j["search_area"] = j_area;

  // No-fly zones
  json j_nfz = json::array();
  for (const auto& nfz : input.no_fly_zones) {
    json j_verts = json::array();
    for (const auto& gp : nfz.vertices) {
      j_verts.push_back({{"latitude", gp.latitude}, {"longitude", gp.longitude}});
    }
    j_nfz.push_back({{"name", nfz.name}, {"vertices", j_verts}});
  }
  j["no_fly_zones"] = j_nfz;

  json j_robots = json::array();
  for (const auto& ri : input.robots) {
    j_robots.push_back({
        {"name", ri.name},
        {"latitude", ri.position.latitude},
        {"longitude", ri.position.longitude},
        {"battery_remaining", ri.battery_remaining}});
  }
  j["robots"] = j_robots;

  // Output
  j["success"] = output.success;
  j["message"] = output.message;
  j["flight_height"] = input.flight_height;

  json j_paths = json::array();
  for (const auto& rp : output.paths) {
    json j_wps = json::array();
    for (const auto& wp : rp.waypoints) {
      j_wps.push_back({
          {"latitude", wp.latitude},
          {"longitude", wp.longitude},
          {"altitude", wp.altitude},
          {"heading", wp.heading}});
    }
    j_paths.push_back({{"robot_name", rp.robot_name}, {"waypoints", j_wps}});
  }
  j["paths"] = j_paths;

  return j;
}

// ── Main ────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " <scenario.json> [--planner name] [--output results.json]\n";
    std::cerr << "\nAvailable planners: ";
    for (const auto& [name, _] : PLANNERS) {
      std::cerr << name << " ";
    }
    std::cerr << "(default: " << DEFAULT_PLANNER << ")\n";
    return 1;
  }

  // Parse arguments
  std::string scenario_path = argv[1];
  std::string planner_name = DEFAULT_PLANNER;
  std::string output_path;

  for (int i = 2; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--planner" && i + 1 < argc) {
      planner_name = argv[++i];
    } else if (arg == "--output" && i + 1 < argc) {
      output_path = argv[++i];
    }
  }

  try {
    // Look up planner
    auto it = PLANNERS.find(planner_name);
    if (it == PLANNERS.end()) {
      std::cerr << "Error: unknown planner '" << planner_name << "'\n";
      std::cerr << "Available planners: ";
      for (const auto& [name, _] : PLANNERS) {
        std::cerr << name << " ";
      }
      std::cerr << "\n";
      return 1;
    }

    auto input = parse_scenario(scenario_path);

    auto logger = std::make_shared<StdoutLogger>();
    auto planner = it->second();
    planner->initialize(logger);

    auto output = planner->plan(input);

    // Print output
    std::cout << "\n=== Result (" << planner_name << ") ===\n";
    std::cout << "Success: " << (output.success ? "true" : "false") << "\n";
    std::cout << "Message: " << output.message << "\n\n";

    for (const auto& rp : output.paths) {
      std::cout << "Robot: " << rp.robot_name
                << "  (" << rp.waypoints.size() << " waypoints)\n";
      for (size_t i = 0; i < rp.waypoints.size(); ++i) {
        const auto& wp = rp.waypoints[i];
        std::cout << "  [" << std::setw(3) << i << "] "
                  << std::fixed << std::setprecision(7)
                  << wp.latitude << ", " << wp.longitude
                  << "  alt=" << std::setprecision(1) << wp.altitude
                  << "  hdg=" << wp.heading << "\n";
      }
      std::cout << "\n";
    }

    // Write JSON output if requested
    if (!output_path.empty()) {
      json j = output_to_json(input, output);
      std::ofstream ofs(output_path);
      if (!ofs.is_open()) {
        std::cerr << "Error: cannot write to " << output_path << "\n";
        return 1;
      }
      ofs << j.dump(2) << "\n";
      std::cout << "JSON output written to: " << output_path << "\n";
    }

    return output.success ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
}
