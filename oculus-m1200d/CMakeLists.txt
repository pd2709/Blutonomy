cmake_minimum_required(VERSION 3.9)

option(BUILD_DOC "Build documentation" ON)

set(BLUEPRINT_SUBSEA_OCULUS_DRIVER_MAJOR_VERSION 0)
set(BLUEPRINT_SUBSEA_OCULUS_DRIVER_MINOR_VERSION 2)
set(BLUEPRINT_SUBSEA_OCULUS_DRIVER_PATCH_VERSION 0)
set(BLUEPRINT_SUBSEA_OCULUS_DRIVER_VERSION
    ${BLUEPRINT_SUBSEA_OCULUS_DRIVER_MAJOR_VERSION}.${BLUEPRINT_SUBSEA_OCULUS_DRIVER_MINOR_VERSION}.${BLUEPRINT_SUBSEA_OCULUS_DRIVER_PATCH_VERSION})

project(
    "blueprint_subsea_oculus_driver"
    DESCRIPTION "C++ driver for the Oculus Imaging Sonar."
    LANGUAGES CXX
    VERSION ${BLUEPRINT_SUBSEA_OCULUS_DRIVER_VERSION})

# Force C++14 and disallow extensions such as G++.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_CXX_FLAGS "-O2 -pipe -march=native -mtune=native -fno-omit-frame-pointer -fstack-protector-all -ftree-loop-distribution -ftree-vectorize -ftree-vectorizer-verbose=2 -funroll-loops -pedantic -Wall -Wcast-align -Wcast-qual -Wconversion -Wctor-dtor-privacy -Wdisabled-optimization -Wdouble-promotion -Werror -Wextra -Wfloat-equal -Wformat=2 -Winit-self -Wlogical-op -Wmissing-braces -Wmissing-declarations -Wmissing-include-dirs -Wnoexcept -Wno-implicit-fallthrough -Wold-style-cast -Woverloaded-virtual -Wparentheses -Wredundant-decls -Wreorder -Wshadow -Wsign-conversion -Wsign-promo -Wstrict-aliasing=3 -Wstrict-null-sentinel -Wstrict-overflow=2 -Wswitch-default -Wundef -Wunreachable-code -Wunused -Wuseless-cast")

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")
set(INSTALL_CMAKE_DIR lib/CMake/${PROJECT_NAME} CACHE PATH "Installation directory for CMake files")

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
    set(var INSTALL_${p}_DIR)
    if(NOT IS_ABSOLUTE "${${var}}")
        set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
    endif()
endforeach()


# --------------------------------------------------------------------------------
# API documentation generation.
# --------------------------------------------------------------------------------

find_package(Doxygen REQUIRED)

if (DOXYGEN_FOUND)

    # Set input and output files.
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # Request to configure the file.
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    # Note the option ALL which allows to build the docs together with the application.
    add_custom_target(doc_doxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM)

else (DOXYGEN_FOUND)

    message("Doxygen need to be installed to generate the doxygen documentation")

endif (DOXYGEN_FOUND)

# --------------------------------------------------------------------------------

# set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
# set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")


find_package(Threads REQUIRED)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost 1.6 REQUIRED COMPONENTS log system)

add_library(
    ${PROJECT_NAME}
    STATIC
    src/OculusDriver.cpp)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
    ${Boost_INCLUDE_DIR})

target_include_directories(
    ${PROJECT_NAME}
    PRIVATE
    include)

target_link_libraries(
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
)

set_target_properties(
    ${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    PUBLIC_HEADER "include/${PROJECT_NAME}/OculusDriver.hpp")

set(APP_NAME raw_oculus_test)


# --------------------------------------------------------------------------------
# Basic testing functionality application.
# --------------------------------------------------------------------------------

add_executable(
    ${APP_NAME}
    app/raw_oculus_test.cpp
)

target_link_libraries(
    ${APP_NAME}
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
)

target_include_directories(
    ${APP_NAME}
    PUBLIC
    ${Boost_INCLUDE_DIRS}
)


# --------------------------------------------------------------------------------
# System-wide installation.
# --------------------------------------------------------------------------------

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
    LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
    ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
    PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/${PROJECT_NAME}" COMPONENT dev)



# The interesting stuff goes here
# ===============================
# Add all targets to the build-tree export set
export(
    TARGETS ${PROJECT_NAME}
    FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(
    PACKAGE ${PROJECT_NAME})

# Create the Config and ConfigVersion files
file(
    RELATIVE_PATH REL_INCLUDE_DIR
    "${INSTALL_CMAKE_DIR}"
    "${INSTALL_INCLUDE_DIR}")

# ... for the build tree
set(CONF_INCLUDE_DIRS
    "${PROJECT_SOURCE_DIR}"
    "${PROJECT_BINARY_DIR}")
configure_file(
    ${PROJECT_NAME}Config.cmake.in
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    @ONLY)

# ... for the install tree
set(
    CONF_INCLUDE_DIRS
    "\${BLUEPRINT_SUBSEA_OCULUS_DRIVER_CMAKE_DIR_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(
    ${PROJECT_NAME}Config.cmake.in
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
    @ONLY)

# ... for both
configure_file(
    ${PROJECT_NAME}ConfigVersion.cmake.in
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    @ONLY)

# Install the phils_librariesConfig.cmake and phils_librariesConfigVersion.cmake
install(
    FILES
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION "${INSTALL_CMAKE_DIR}"
    COMPONENT dev)

# Install the export set for use with the install-tree
install(
    EXPORT ${PROJECT_NAME}Targets
    DESTINATION "${INSTALL_CMAKE_DIR}"
    COMPONENT dev)
