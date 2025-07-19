cmake_minimum_required(VERSION 3.8)
project(reference_costmap_generator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Optimization flags for performance
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # Add optimization flags for Release build
  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3 -march=native -DNDEBUG)
  endif()
endif()

# parameters
option(USE_OPENMP "USE_OPENMP" ON) # ON / OFF

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_filters REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_visualization REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# use eigen3 (matrix computation library)
find_package(Eigen3 REQUIRED)

# load openmp
if(USE_OPENMP)
  find_package(OpenMP REQUIRED)
  if(OpenMP_FOUND)
      message(WARNING "OpenMP found. Activate CPU acceleration.")
      set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  endif()
  if(NOT OPENMP_FOUND)
      message(FATAL ERROR "Unable to find OpenMP library.")
  endif()
endif()
# Set include directories for all targets in this project
include_directories(include)

add_library(reference_costmap_generator SHARED src/reference_costmap_generator.cpp)

ament_target_dependencies(reference_costmap_generator PUBLIC
  grid_map_core
  grid_map_filters
  grid_map_msgs
  grid_map_ros
  grid_map_visualization
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

target_link_libraries(reference_costmap_generator PUBLIC Eigen3::Eigen)
# Link OpenMP if enabled
if(USE_OPENMP AND OpenMP_FOUND)
  message(WARNING "linking the OpenMP library.")
  target_link_libraries(reference_costmap_generator PUBLIC OpenMP::OpenMP_CXX)
endif()

add_executable(${PROJECT_NAME}_node src/reference_costmap_generator_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_node PUBLIC
  rclcpp
  grid_map_core
  grid_map_filters
  grid_map_msgs
  grid_map_ros
  grid_map_visualization
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

target_link_libraries(${PROJECT_NAME}_node PUBLIC reference_costmap_generator)
# -- Installation Rules --

# Install the compiled library and executable targets.
# The EXPORT keyword creates a CMake export set, making the library target
# available for other packages to link against.
install(TARGETS
  reference_costmap_generator
  ${PROJECT_NAME}_node
  EXPORT export_reference_costmap_generator # Name of the export set
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} # A common pattern for node executables
  INCLUDES DESTINATION include
)

# Install non-code artifacts like launch and config files.
# The destination is relative to the install root, typically 'install/<pkg_name>/'.
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# -- Exporting for Downstream Packages --
# These commands replace the functionality of catkin_package().

# Export the library targets defined in the install() EXPORT clause.
# This allows other packages to use `ament_target_dependencies` on this package.
ament_export_targets(export_reference_costmap_generator HAS_LIBRARY_TARGET)

# Export the list of dependencies that are part of this package's public API.
# Downstream packages will automatically have these dependencies available.
ament_export_dependencies(
  rclcpp
  grid_map_core
  grid_map_ros
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

# This must be the VERY LAST command in the file.
# It collects all the exported information and generates the necessary
# CMake config files and resource index markers for the ament build system.
ament_package()