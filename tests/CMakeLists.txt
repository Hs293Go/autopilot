project(autopilot_tests)

include(FetchContent)
FetchContent_Declare(
  GTest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG main
  FIND_PACKAGE_ARGS 1.14.0 CONFIG)
FetchContent_MakeAvailable(GTest)

add_executable(test_integration src/integration/test_cascade_controller_sim.cpp)
target_link_libraries(
  test_integration PRIVATE autopilot::autopilot autopilot::validation
                           GTest::gtest GTest::gtest_main)
