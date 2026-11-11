# CMake policies
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
  cmake_policy(SET CMP0148 NEW)
endif()

# Deps
find_package(Eigen3 REQUIRED COMPONENTS Core Dense Geometry)

file(
  DOWNLOAD
  https://github.com/cpm-cmake/CPM.cmake/releases/download/v0.40.5/CPM.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake
  EXPECTED_HASH SHA256=c46b876ae3b9f994b4f05a4c15553e0485636862064f1fcc9d8b4f832086bc5d
)
include(${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake)

message("\n-----------------------------------------------\n")

# graph_core provides the logger (graph_core/logger.h) and the parameters (graph_core/params.h)
CPMFindPackage(
  NAME graph_core
  GITHUB_REPOSITORY JRL-CARI-CNR-UNIBS/graph_core
  GIT_TAG master
)

message("\n-----------------------------------------------\n")
