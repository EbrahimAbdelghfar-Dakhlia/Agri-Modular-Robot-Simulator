cmake_minimum_required(VERSION 3.8)
project(reference_costmap_generator)

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
find_package(Eigen3 REQUIRED)

option(USE_OPENMP "USE_OPENMP" ON)
if(USE_OPENMP)
  find_package(OpenMP REQUIRED)
  if(OpenMP_FOUND)
    message(STATUS "OpenMP found. Activating CPU acceleration.")
  else()
    message(FATAL_ERROR "Unable to find OpenMP library.")
  endif()
endif()

add_library(reference_costmap_generator SHARED src/reference_costmap_generator.cpp)
target_include_directories(reference_costmap_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
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
target_link_libraries(reference_costmap_generator PUBLIC ${EIGEN3_INCLUDE_DIR})
if(USE_OPENMP)
  target_link_libraries(reference_costmap_generator PUBLIC OpenMP::OpenMP_CXX)
endif()

add_executable(${PROJECT_NAME}_node src/reference_costmap_generator_node.cpp)
target_link_libraries(${PROJECT_NAME}_node PRIVATE reference_costmap_generator)
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

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS reference_costmap_generator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()