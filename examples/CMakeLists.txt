project(autopilot_examples)

include(FetchContent)
FetchContent_Declare(
  rerun_sdk
  URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
)
FetchContent_MakeAvailable(rerun_sdk)

add_executable(cascade_controller_sim src/cascade_controller_sim.cpp)
target_link_libraries(
  cascade_controller_sim PRIVATE autopilot::autopilot autopilot::validation
                                 rerun_sdk)
