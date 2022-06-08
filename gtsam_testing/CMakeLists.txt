# This file shows how to build and link a user project against GTSAM using CMake
###################################################################################
# To create your own project, replace "example" with the actual name of your project
cmake_minimum_required(VERSION 3.0)
project(gtsam_testing)

# Find GTSAM, either from a local build, or from a Debian/Ubuntu package.
find_package(GTSAM REQUIRED)
find_package(GTest REQUIRED)
include_directories(
  ${GTEST_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  )

add_executable(gtsam_testing
  # OdometryExample.cpp
  3DLandmarkTrial.cpp
)

add_executable(runTests test/tests.cpp GeometricTransforms.cpp)

# By using CMake exported targets, a simple "link" dependency introduces the
# include directories (-I) flags, links against Boost, and add any other
# required build flags (e.g. C++11, etc.)
target_link_libraries(gtsam_testing PRIVATE gtsam)
target_link_libraries(runTests ${GTEST_LIBRARIES} pthread gtsam)