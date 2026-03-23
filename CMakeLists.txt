cmake_minimum_required(VERSION 3.14)
project(PlannerTemplate LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── GeographicLib ────────────────────────────────────────────────────────────
# Try system/pixi-installed package first, fall back to FetchContent
find_package(GeographicLib QUIET)
if(NOT GeographicLib_FOUND)
  message(STATUS "GeographicLib not found locally, fetching from GitHub...")
  include(FetchContent)
  FetchContent_Declare(
    GeographicLib
    GIT_REPOSITORY https://github.com/geographiclib/geographiclib.git
    GIT_TAG        v2.7
  )
  set(BUILD_BOTH_LIBS OFF CACHE BOOL "" FORCE)
  set(BUILD_DOCUMENTATION OFF CACHE BOOL "" FORCE)
  set(GEOGRAPHICLIB_PRECISION 2 CACHE STRING "" FORCE)
  set(BUILD_TOOLS OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(GeographicLib)
else()
  message(STATUS "Found GeographicLib: ${GeographicLib_VERSION}")
endif()

# ── Library ──────────────────────────────────────────────────────────────────
add_library(PlannerTemplateLib SHARED
  src/geo_utils.cpp
  src/logger.cpp
  src/example_planner.cpp
  src/my_planner.cpp
)

target_include_directories(PlannerTemplateLib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src          # for example_planner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party  # for nlohmann/json.hpp
)

target_link_libraries(PlannerTemplateLib
  PRIVATE
    GeographicLib::GeographicLib
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
