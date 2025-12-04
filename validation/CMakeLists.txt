project(autopilot_validation)
add_library(autopilot_validation)
target_sources(
  autopilot_validation
  PUBLIC FILE_SET HEADERS BASE_DIRS include FILES
         include/validation/mission_runner.hpp
  PRIVATE src/mission_runner.cpp)
target_link_libraries(autopilot_validation PRIVATE autopilot::autopilot)

add_library(autopilot::validation ALIAS autopilot_validation)
set_target_properties(autopilot_validation PROPERTIES EXPORT_NAME validation)
