project(autopilot_tests)

include(FetchContent)
FetchContent_Declare(
  GTest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG main
  FIND_PACKAGE_ARGS 1.14.0 CONFIG)
FetchContent_MakeAvailable(GTest)

add_library(testing INTERFACE)
target_sources(testing INTERFACE FILE_SET HEADERS BASE_DIRS include FILES
                                 include/testing/matchers.hpp)
target_link_libraries(testing INTERFACE GTest::gmock)

add_executable(test_integration src/integration/test_cascade_controller_sim.cpp)
target_link_libraries(
  test_integration PRIVATE autopilot::autopilot autopilot::validation
                           GTest::gtest GTest::gmock GTest::gtest_main testing)

add_executable(
  test_unit src/unit/test_spatial_rotations.cpp src/unit/test_config_loader.cpp
            src/unit/test_quadrotor_model.cpp src/unit/test_polynomial.cpp)
target_link_libraries(
  test_unit PRIVATE autopilot::autopilot autopilot::extensions GTest::gtest
                    GTest::gtest_main GTest::gmock testing)

add_custom_target(tests DEPENDS test_unit test_integration)
