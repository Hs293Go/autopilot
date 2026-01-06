cmake_minimum_required(VERSION 3.31)

# Prevent in-source builds
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
  message(
    FATAL_ERROR
      "In-source builds are not allowed. Please create a separate build directory."
      "You may need to delete CMakeCache.txt and CMakeFiles/ from the source directory."
  )
endif()

project(autopilot)

find_package(spdlog REQUIRED)
find_package(Boost REQUIRED)
include(FetchContent)

FetchContent_Declare(
  Eigen3
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.bz2
      SYSTEM FIND_PACKAGE_ARGS 3.4 CONFIG)

FetchContent_Declare(
  nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
      SYSTEM FIND_PACKAGE_ARGS 3.12 CONFIG)
FetchContent_MakeAvailable(nlohmann_json Eigen3)

add_library(autopilot_autopilot)
target_sources(
  autopilot_autopilot
  PRIVATE src/async_estimator.cpp
          src/cascade_controller.cpp
          src/definitions.cpp
          src/ekf.cpp
          src/eskf.cpp
          src/geometric_controller.cpp
          src/quadrotor_model.cpp
          src/quadrotor_simulator.cpp
          src/sensors.cpp
  PUBLIC FILE_SET
         HEADERS
         BASE_DIRS
         include
         FILES
         include/autopilot/base/config_base.hpp
         include/autopilot/base/controller_base.hpp
         include/autopilot/base/estimator_base.hpp
         include/autopilot/base/factory.hpp
         include/autopilot/base/module.hpp
         include/autopilot/controllers/cascade_controller.hpp
         include/autopilot/controllers/geometric_controller.hpp
         include/autopilot/core/butterworth_filter.hpp
         include/autopilot/core/common.hpp
         include/autopilot/core/definitions.hpp
         include/autopilot/core/geometry.hpp
         include/autopilot/core/math.hpp
         include/autopilot/core/quadrotor_model.hpp
         include/autopilot/estimators/async_estimator.hpp
         include/autopilot/estimators/ekf.hpp
         include/autopilot/estimators/eskf.hpp
         include/autopilot/estimators/sensor_data.hpp
         include/autopilot/planning/polynomial.hpp
         include/autopilot/simulator/quadrotor_simulator.hpp
         include/autopilot/simulator/sensors.hpp)
target_compile_features(autopilot_autopilot PUBLIC cxx_std_23)
target_link_libraries(autopilot_autopilot PUBLIC Eigen3::Eigen spdlog::spdlog
                                                 Boost::boost)

add_library(autopilot::autopilot ALIAS autopilot_autopilot)
set_target_properties(autopilot_autopilot PROPERTIES EXPORT_NAME autopilot)

add_library(autopilot_extensions)
target_sources(
  autopilot_extensions
  PRIVATE src/json_loader.cpp src/pretty_printer.cpp
  PUBLIC FILE_SET HEADERS BASE_DIRS include FILES
         include/autopilot/extensions/json_loader.hpp)
target_link_libraries(autopilot_extensions PUBLIC autopilot::autopilot
                                                  nlohmann_json::nlohmann_json)
add_library(autopilot::extensions ALIAS autopilot_extensions)

option(AUTOPILOT_BUILD_TESTING "Build the autopilot tests" ON)
option(AUTOPILOT_BUILD_EXAMPLES "Build the autopilot examples" ON)
include(CMakeDependentOption)
cmake_dependent_option(
  AUTOPILOT_SKIP_BUILD_VALIDATION
  "Skip building the autopilot validation suite" OFF
  "NOT AUTOPILOT_BUILD_TESTING;NOT AUTOPILOT_BUILD_EXAMPLES" OFF)
set(BUILD_TESTING ${AUTOPILOT_BUILD_TESTING})

include(CTest)

if(NOT AUTOPILOT_SKIP_BUILD_VALIDATION)
  add_subdirectory(validation)
endif()

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

if(AUTOPILOT_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
