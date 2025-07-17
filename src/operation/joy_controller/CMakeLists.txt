cmake_minimum_required(VERSION 3.8)
project(joy_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
)

## Declare a C++ executable
add_executable(joy_controller_node 
  src/joy_controller_node.cpp 
  src/joy_controller.cpp
)

ament_target_dependencies(joy_controller_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

#############
## Install ##
#############

# Install the executable
install(TARGETS joy_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
