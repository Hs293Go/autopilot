project(autopilot_tests)

include(GoogleTest)

include(FetchContent)
FetchContent_Declare(
  GTest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG main
  SYSTEM FIND_PACKAGE_ARGS 1.14.0 CONFIG)
FetchContent_MakeAvailable(GTest)

add_library(testing)
target_sources(
  testing
  PUBLIC FILE_SET
         HEADERS
         BASE_DIRS
         include
         FILES
         include/testing/mock_factory.hpp
         include/testing/matchers.hpp
  PRIVATE src/testing/mock_factory.cpp)
target_link_libraries(testing PUBLIC GTest::gmock autopilot::autopilot)

add_executable(test_integration src/integration/test_trajectory_execution.cpp)
target_link_libraries(
  test_integration
  PRIVATE autopilot::autopilot
          autopilot::validation
          autopilot::controllers
          GTest::gtest
          GTest::gmock
          GTest::gtest_main
          testing)

add_executable(
  test_unit
  src/unit/test_butterworth_filter.cpp
  src/unit/test_factory.cpp
  src/unit/test_heading_policy.cpp
  src/unit/test_spatial_rotations.cpp
  src/unit/test_angles.cpp
  src/unit/test_config_loader.cpp
  src/unit/test_quadrotor_model.cpp
  src/unit/test_polynomial.cpp
  src/unit/test_attitude_error.cpp
  src/unit/test_mission_system.cpp
  src/unit/test_minimum_snap_solver.cpp)
target_link_libraries(
  test_unit
  PRIVATE autopilot::autopilot
          autopilot::controllers
          autopilot::planning
          GTest::gtest
          GTest::gtest_main
          GTest::gmock
          $<LINK_LIBRARY:WHOLE_ARCHIVE,testing>)

add_custom_target(tests DEPENDS test_unit test_integration)

gtest_discover_tests(
  test_unit
  PROPERTIES LABELS "unit"
  TIMEOUT 10)

gtest_discover_tests(
  test_integration
  PROPERTIES LABELS "integration"
  TIMEOUT 60)
