cmake_minimum_required(VERSION 3.28)

cmake_policy(SET CMP0077 NEW)

# Prevent in-source builds
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
  message(
    FATAL_ERROR
      "In-source builds are not allowed. Please create a separate build directory."
      "You may need to delete CMakeCache.txt and CMakeFiles/ from the source directory."
  )
endif()

project(autopilot)

set(AUTOPILOT_ESTIMATOR_COVARIANCE_MATRIX_MAX_SIZE
    20
    CACHE STRING "Maximum size of the estimator covariance matrix")
message(
  STATUS
    "Autopilot: Estimator covariance matrix max size set to ${AUTOPILOT_ESTIMATOR_COVARIANCE_MATRIX_MAX_SIZE}"
)

include(FetchContent)

FetchContent_Declare(
  fmt URL https://github.com/fmtlib/fmt/releases/download/12.1.0/fmt-12.1.0.zip
          SYSTEM FIND_PACKAGE_ARGS 8.1.1 CONFIG)
set(SPDLOG_FMT_EXTERNAL ON)
FetchContent_Declare(
  spdlog URL https://github.com/gabime/spdlog/archive/refs/tags/v1.17.0.tar.gz
             SYSTEM FIND_PACKAGE_ARGS 1.9.2 CONFIG)

include(cmake/eigen_source_build_tweaks.cmake)
FetchContent_Declare(
  Eigen3
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.1/eigen-3.4.1.tar.bz2
      SYSTEM FIND_PACKAGE_ARGS 3.4.0 CONFIG)

FetchContent_Declare(
  nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
      SYSTEM FIND_PACKAGE_ARGS 3.10 CONFIG)

set(BOOST_MATH_STANDALONE ON)
FetchContent_Declare(
  Boost
  URL https://github.com/boostorg/math/archive/refs/tags/boost-1.90.0.tar.gz
      SYSTEM FIND_PACKAGE_ARGS 1.90.0 CONFIG)
FetchContent_MakeAvailable(fmt spdlog nlohmann_json Eigen3 Boost)

add_library(autopilot_autopilot)
target_sources(
  autopilot_autopilot
  PRIVATE src/definitions.cpp
          src/ekf.cpp
          src/eskf.cpp
          src/quadrotor_model.cpp
          src/quadrotor_simulator.cpp
          src/sensors.cpp
          src/json_loader.cpp
          src/pretty_printer.cpp
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
         include/autopilot/core/butterworth_filter.hpp
         include/autopilot/core/common.hpp
         include/autopilot/core/definitions.hpp
         include/autopilot/core/geometry.hpp
         include/autopilot/core/math.hpp
         include/autopilot/core/quadrotor_model.hpp
         include/autopilot/extensions/json_loader.hpp
         include/autopilot/extensions/pretty_printer.hpp
         include/autopilot/planning/polynomial.hpp
         include/autopilot/simulator/quadrotor_simulator.hpp
         include/autopilot/simulator/sensors.hpp)
target_compile_definitions(
  autopilot_autopilot
  PUBLIC
    AUTOPILOT_ESTIMATOR_COVARIANCE_MATRIX_MAX_SIZE=${AUTOPILOT_ESTIMATOR_COVARIANCE_MATRIX_MAX_SIZE}
)
target_compile_features(autopilot_autopilot PUBLIC cxx_std_23)
target_link_libraries(
  autopilot_autopilot PUBLIC Eigen3::Eigen spdlog::spdlog fmt::fmt
                             nlohmann_json::nlohmann_json Boost::math)

add_library(autopilot::autopilot ALIAS autopilot_autopilot)
set_target_properties(autopilot_autopilot PROPERTIES EXPORT_NAME autopilot)

add_library(autopilot_controllers STATIC)
target_sources(
  autopilot_controllers
  PRIVATE src/attitude_error.cpp src/cascade_controller.cpp
          src/geometric_controller.cpp
  PUBLIC FILE_SET
         HEADERS
         BASE_DIRS
         include
         FILES
         include/autopilot/controllers/attitude_error.hpp
         include/autopilot/controllers/cascade_controller.hpp
         include/autopilot/controllers/geometric_controller.hpp)
target_link_libraries(autopilot_controllers PUBLIC autopilot::autopilot)
add_library(autopilot::controllers ALIAS autopilot_controllers)
set_target_properties(autopilot_controllers PROPERTIES EXPORT_NAME controllers)

add_library(autopilot_estimators STATIC)
target_sources(
  autopilot_estimators
  PRIVATE src/async_estimator.cpp src/quadrotor_simulator.cpp src/ekf.cpp
          src/eskf.cpp
  PUBLIC FILE_SET
         HEADERS
         BASE_DIRS
         include
         FILES
         include/autopilot/estimators/async_estimator.hpp
         include/autopilot/estimators/ekf.hpp
         include/autopilot/estimators/eskf.hpp
         include/autopilot/estimators/sensor_data.hpp)
target_link_libraries(
  autopilot_estimators
  PUBLIC autopilot::autopilot
  PRIVATE "$<COMPILE_ONLY:Boost::math>")
add_library(autopilot::estimators ALIAS autopilot_estimators)
set_target_properties(autopilot_estimators PROPERTIES EXPORT_NAME estimators)

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
  message(STATUS "Autopilot: Building tests")
  add_subdirectory(tests)
endif()

if(AUTOPILOT_BUILD_EXAMPLES)
  message(STATUS "Autopilot: Building examples")
  add_subdirectory(examples)
endif()
