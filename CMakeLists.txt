cmake_minimum_required(VERSION 3.16)
project(trajectories_processors_lib)

if(NOT CMAKE_BUILD_TYPE)
  #set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_BUILD_TYPE "Release")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  if(CMAKE_BUILD_TYPE MATCHES "Release")
    add_compile_options(-funroll-loops -Wall -Ofast -flto -O3)
  endif()
  if(CMAKE_BUILD_TYPE MATCHES "Debug")
    add_compile_options(-Wall -g -Og)
  endif()
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#Options
option(ADD_CUSTOM_UNINSTALL "Add uninstall target" ON)

# Dependencies
include(cmake_config/Dependencies.cmake)

add_library(${PROJECT_NAME} SHARED
             src/trajectory_processor_base.cpp
             src/spline_trajectory_processor.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC 
  Eigen3::Eigen
  cnr_logger::cnr_logger
  cnr_param::cnr_param
  )

add_library("${PROJECT_NAME}::${PROJECT_NAME}" ALIAS ${PROJECT_NAME})

# Install
install(DIRECTORY include/openmore
  DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT ${PROJECT_NAME}Targets
  DESTINATION "share/${PROJECT_NAME}/cmake"
  NAMESPACE ${PROJECT_NAME}::
  FILE ${PROJECT_NAME}Targets.cmake
)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION 0.0.1
  COMPATIBILITY AnyNewerVersion)
configure_package_config_file(
  ${CMAKE_SOURCE_DIR}/cmake_config/${PROJECT_NAME}Config.cmake.in ${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION share/${PROJECT_NAME}/cmake
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION "share/${PROJECT_NAME}/cmake")

# make uninstall
if(ADD_CUSTOM_UNINSTALL)
    message(STATUS "Adding custom uninstall")
    add_custom_target("uninstall" COMMENT "Uninstall installed files")
    add_custom_command(
        TARGET "uninstall"
        POST_BUILD
        COMMENT "Uninstall files with install_manifest.txt"
        COMMAND xargs rm -vf < install_manifest.txt || echo Nothing in
        install_manifest.txt to be uninstalled!
        )
endif()

# Clang-format target
file(GLOB_RECURSE ALL_SOURCE_FILES *.cpp *.h)

add_custom_target(
    format
    COMMAND chmod +x cmake_config/clang_format_target.sh
    COMMAND cmake_config/clang_format_target.sh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Running clang-format on all source files"
)
