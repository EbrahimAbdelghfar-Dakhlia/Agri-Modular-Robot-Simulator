cmake_minimum_required(VERSION 3.8)
project(mppi_h)

option(USE_OPENMP "USE_OPENMP" ON) # ON / OFF
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

add_library(mppi_h_core SHARED src/mppi_h_core.cpp)
add_library(mppi_mode1_core SHARED src/mode1_mppi_3d/mppi_3d_core.cpp)
add_library(mppi_mode2_core SHARED src/mode2_mppi_4d/mppi_4d_core.cpp)

## link openmp
if(USE_OPENMP)
  if (OPENMP_FOUND)
      if (TARGET OpenMP::OpenMP_CXX)
          target_link_libraries(mppi_mode1_core PUBLIC OpenMP::OpenMP_CXX)
          target_link_libraries(mppi_mode2_core PUBLIC OpenMP::OpenMP_CXX)
      endif()
  endif()
endif()

ament_target_dependencies(mppi_mode1_core PUBLIC
  ${dependencies}
)

ament_target_dependencies(mppi_mode2_core PUBLIC
  ${dependencies}
)

ament_target_dependencies(mppi_h_core PUBLIC
  ${dependencies}
)

target_include_directories(mppi_h_core PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(mppi_h_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(mppi_mode1_core PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(mppi_mode1_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(mppi_mode2_core PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(mppi_mode2_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/mppi_h_node.cpp src/mppi_h.cpp)
ament_target_dependencies(${PROJECT_NAME}_node PUBLIC
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_node PUBLIC 
mppi_h_core
mppi_mode1_core
mppi_mode2_core
)

# Install
install(TARGETS mppi_h_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS mppi_mode1_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS mppi_mode2_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)
## Export dependencies
ament_export_dependencies(${dependencies})
ament_package()
