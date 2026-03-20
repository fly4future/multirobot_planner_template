cmake_minimum_required(VERSION 3.14)
project(PlannerTemplate LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Library ──────────────────────────────────────────────────────────────────
add_library(PlannerTemplateLib SHARED
  src/geo_utils.cpp
  src/logger.cpp
  src/example_planner.cpp
)

target_include_directories(PlannerTemplateLib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src          # for example_planner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party  # for nlohmann/json.hpp
)

# ── Demo executable ──────────────────────────────────────────────────────────
add_executable(planner_demo
  src/main.cpp
)

target_link_libraries(planner_demo PRIVATE PlannerTemplateLib)

target_include_directories(planner_demo
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)
