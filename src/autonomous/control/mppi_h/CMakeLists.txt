cmake_minimum_required(VERSION 3.8)
project(mppi_h)

# Set the C++ standard to ensure compatibility and consistent builds
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Parameters
option(USE_OPENMP "Enable OpenMP for CPU acceleration" ON)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rviz_2d_overlay_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_filters REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_visualization REQUIRED)
find_package(mppi_eval_msgs REQUIRED)
# For Eigen3, use the eigen3_cmake_module for robust discovery across systems.
find_package(eigen3_cmake_module REQUIRED)
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

set(dependencies
  rclcpp
  geometry_msgs
  visualization_msgs
  rviz_2d_overlay_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  grid_map_core
  grid_map_ros
  grid_map_filters
  grid_map_msgs
  grid_map_visualization
  mppi_eval_msgs
)

# Declare C++ libraries
add_library(mppi_h_core SHARED src/mppi_h_core.cpp)
add_library(mppi_mode1_core SHARED src/mode1_mppi_3d/mppi_3d_core.cpp)
add_library(mppi_mode2_core SHARED src/mode2_mppi_4d/mppi_4d_core.cpp)
# Target-specific include directories
target_include_directories(mppi_h_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(mppi_mode1_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(mppi_mode2_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Link Eigen3 to libraries that require it using modern CMake targets
target_link_libraries(mppi_h_core PUBLIC Eigen3::Eigen)
target_link_libraries(mppi_mode1_core PUBLIC Eigen3::Eigen)
target_link_libraries(mppi_mode2_core PUBLIC Eigen3::Eigen)

# Link OpenMP to the specific libraries that use it
if(USE_OPENMP AND OpenMP_FOUND)
  target_link_libraries(mppi_mode1_core PUBLIC OpenMP::OpenMP_CXX)
  target_link_libraries(mppi_mode2_core PUBLIC OpenMP::OpenMP_CXX)
endif()

# Declare the C++ executable
add_executable(${PROJECT_NAME}_node src/mppi_h_node.cpp src/mppi_h.cpp)

ament_target_dependencies(mppi_mode1_core PUBLIC
  ${dependencies}
)

ament_target_dependencies(mppi_mode2_core PUBLIC
  ${dependencies}
)

ament_target_dependencies(mppi_h_core PUBLIC
  ${dependencies}
)

# Use target_link_libraries for libraries built in this package
target_link_libraries(${PROJECT_NAME}_node PUBLIC
  mppi_h_core
  mppi_mode1_core
  mppi_mode2_core
)

install(TARGETS
  mppi_h_core
  mppi_mode1_core
  mppi_mode2_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ${dependencies}
)
#############
## Install ##
#############

# Install the main node executable
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install public headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# This MUST be the last call in the file
ament_package()