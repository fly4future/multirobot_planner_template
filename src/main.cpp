#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

#include "PlannerTemplate/logger.h"
#include "PlannerTemplate/types.h"
#include "example_planner.h"

using json = nlohmann::json;
using namespace planner_template;

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

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <scenario.json> [--output results.json]\n";
    return 1;
  }

  // Parse optional --output flag
  std::string output_path;
  std::string scenario_path = argv[1];
  for (int i = 2; i < argc; ++i) {
    if (std::string(argv[i]) == "--output" && i + 1 < argc) {
      output_path = argv[++i];
    }
  }

  try {
    auto input = parse_scenario(scenario_path);

    auto logger = std::make_shared<StdoutLogger>();
    ExamplePlanner planner;
    planner.initialize(logger);

    auto output = planner.plan(input);

    // Print output
    std::cout << "\n=== Result ===\n";
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
