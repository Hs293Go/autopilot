project(autopilot_examples)

include(FetchContent)
FetchContent_Declare(
  rerun_sdk
  URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
      SYSTEM)
FetchContent_MakeAvailable(rerun_sdk)

add_library(visualization)
target_sources(
  visualization
  PRIVATE src/visualization.cpp
  PUBLIC FILE_SET HEADERS BASE_DIRS include FILES
         include/examples/visualization.hpp)
target_link_libraries(visualization PUBLIC rerun_sdk autopilot::autopilot)

add_executable(cascade_controller_sim src/cascade_controller_sim.cpp)
target_link_libraries(
  cascade_controller_sim
  PRIVATE
    "$<LINK_LIBRARY:WHOLE_ARCHIVE,autopilot::validation,autopilot::controllers,autopilot::estimators>"
    visualization)

target_compile_definitions(
  cascade_controller_sim
  PRIVATE
    CONFIG_FILE=\"${CMAKE_CURRENT_SOURCE_DIR}/config/cascade_controller_sim.json\"
)
add_executable(format_types src/format_types.cpp)
target_link_libraries(format_types PRIVATE autopilot::autopilot)
