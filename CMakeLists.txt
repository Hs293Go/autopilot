cmake_minimum_required(VERSION 3.31)
project(autopilot)

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)

add_library(autopilot_autopilot)
target_sources(
  autopilot_autopilot
  PRIVATE src/geometric_controller.cpp src/quadrotor_model.cpp
          src/cascade_controller.cpp src/definitions.cpp
          src/quadrotor_simulator.cpp src/sensors.cpp
  PUBLIC FILE_SET
         HEADERS
         BASE_DIRS
         include
         FILES
         include/autopilot/base.hpp
         include/autopilot/cascade_controller.hpp
         include/autopilot/common.hpp
         include/autopilot/definitions.hpp
         include/autopilot/expected.hpp
         include/autopilot/geometry.hpp
         include/autopilot/math.hpp
         include/autopilot/module.hpp
         include/autopilot/geometric_controller.hpp
         include/autopilot/quadrotor_simulator.hpp
         include/autopilot/quadrotor_model.hpp
         include/autopilot/sensors.hpp
         include/autopilot/third_party/tl/expected.hpp)
target_compile_features(autopilot_autopilot PUBLIC cxx_std_23)
target_link_libraries(autopilot_autopilot PUBLIC Eigen3::Eigen spdlog::spdlog)

add_library(autopilot::autopilot ALIAS autopilot_autopilot)
set_target_properties(autopilot_autopilot PROPERTIES EXPORT_NAME autopilot)

option(AUTOPILOT_BUILD_TESTING "Build the autopilot tests" ON)
set(BUILD_TESTING ${AUTOPILOT_BUILD_TESTING})

include(CTest)

if(BUILD_TESTING)
  add_subdirectory(validation)
endif()

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
