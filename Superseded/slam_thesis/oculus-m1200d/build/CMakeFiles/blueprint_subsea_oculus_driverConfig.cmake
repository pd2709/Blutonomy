# Compute paths
get_filename_component(
  BLUEPRINT_SUBSEA_OCULUS_DRIVER_CMAKE_DIR
  "${CMAKE_CURRENT_LIST_FILE}"
  PATH)

set(
  BLUEPRINT_SUBSEA_OCULUS_DRIVER_INCLUDE_DIRS "${BLUEPRINT_SUBSEA_OCULUS_DRIVER_CMAKE_DIR_CMAKE_DIR}/../../../include")

# Our library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET BLUEPRINT_SUBSEA_OCULUS_DRIVER_BINARY_DIR)
  include("${BLUEPRINT_SUBSEA_OCULUS_DRIVER_CMAKE_DIR}/blueprint_subsea_oculus_driverTargets.cmake")
endif()
