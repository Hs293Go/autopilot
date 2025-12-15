project(autopilot_examples)

include(FetchContent)
FetchContent_Declare(
  rerun_sdk
  URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
)
FetchContent_MakeAvailable(rerun_sdk)

add_executable(cascade_controller_sim src/cascade_controller_sim.cpp)
target_link_libraries(
  cascade_controller_sim
  PRIVATE $<LINK_LIBRARY:WHOLE_ARCHIVE,autopilot::autopilot>
          autopilot::validation autopilot::extensions rerun_sdk)
target_compile_definitions(
  cascade_controller_sim
  PRIVATE
    CONFIG_FILE=\"${CMAKE_CURRENT_SOURCE_DIR}/config/cascade_controller_sim.json\"
)
add_executable(format_types src/format_types.cpp)
target_link_libraries(format_types PRIVATE autopilot::autopilot)
