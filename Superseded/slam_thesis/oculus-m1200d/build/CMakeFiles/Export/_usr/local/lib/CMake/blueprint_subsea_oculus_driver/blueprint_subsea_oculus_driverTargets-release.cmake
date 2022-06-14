#----------------------------------------------------------------
# Generated CMake target import file for configuration "RELEASE".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "blueprint_subsea_oculus_driver" for configuration "RELEASE"
set_property(TARGET blueprint_subsea_oculus_driver APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(blueprint_subsea_oculus_driver PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "/usr/local/lib/libblueprint_subsea_oculus_driver.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS blueprint_subsea_oculus_driver )
list(APPEND _IMPORT_CHECK_FILES_FOR_blueprint_subsea_oculus_driver "/usr/local/lib/libblueprint_subsea_oculus_driver.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
