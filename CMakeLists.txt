cmake_minimum_required(VERSION 3.31)
project(autopilot)

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)

add_library(autopilot)
target_sources(
  autopilot
  PRIVATE src/geometric_controller.cpp src/quadrotor_model.cpp
          src/cascade_controller.cpp src/definitions.cpp
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
         include/autopilot/quadrotor_model.hpp
         include/autopilot/third_party/tl/expected.hpp)
target_compile_features(autopilot PUBLIC cxx_std_23)
target_link_libraries(autopilot PUBLIC Eigen3::Eigen)
