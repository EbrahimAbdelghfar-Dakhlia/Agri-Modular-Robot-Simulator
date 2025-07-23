# Reference Costmap Generator - ROS2 Conversion

This document outlines the conversion of the Reference Costmap Generator from ROS1 to ROS2.

## Key Changes Made

### 1. Node Architecture
- **ROS1**: Used `ros::NodeHandle` and private node handle (`~`)
- **ROS2**: Inherits from `rclcpp::Node`

### 2. Parameter Handling
- **ROS1**: `private_nh_.param<type>("param_name", variable, default_value)`
- **ROS2**: `this->declare_parameter("param_name", default_value)` followed by `this->get_parameter("param_name").as_type()`

### 3. Publishers and Subscribers
- **ROS1**: `nh_.advertise<MessageType>()` and `nh_.subscribe()`
- **ROS2**: `this->create_publisher<MessageType>()` and `this->create_subscription<MessageType>()`

### 4. Message Types
- **ROS1**: `nav_msgs::Path` → **ROS2**: `nav_msgs::msg::Path`
- **ROS1**: `nav_msgs::OccupancyGrid` → **ROS2**: `nav_msgs::msg::OccupancyGrid`
- **ROS1**: `visualization_msgs::Marker` → **ROS2**: `visualization_msgs::msg::Marker`
- **ROS1**: `grid_map_msgs::GridMap` → **ROS2**: `grid_map_msgs::msg::GridMap`
- All message types now include `::msg::` namespace

### 5. Callback Signatures
- **ROS1**: `const MessageType::ConstPtr& msg`
- **ROS2**: `const MessageType::SharedPtr msg`

### 6. Publishing
- **ROS1**: `publisher.publish(message)`
- **ROS2**: `publisher->publish(message)`

### 7. Time
- **ROS1**: `ros::Time::now()` and `time.toNSec()`
- **ROS2**: `this->get_clock()->now()` and `time.nanoseconds()`

### 8. Logging
- **ROS1**: `ROS_WARN("message")`
- **ROS2**: `RCLCPP_WARN(this->get_logger(), "message")`

## File Changes

### Source Files
- `src/reference_costmap_generator.cpp`: Complete conversion to ROS2 APIs
- `src/reference_costmap_generator_node.cpp`: Updated main function for ROS2
- `include/reference_costmap_generator/reference_costmap_generator.hpp`: Updated class declarations and message types

### Configuration Files
- `package.xml`: Already ROS2 format, fixed package name
- `CMakeLists.txt`: Added ament_target_dependencies for executable
- `config/reference_costmap_generator.yaml`: Updated to ROS2 parameter format with `ros__parameters`
- `launch/reference_costmap_generator.launch.py`: Created new Python launch file for ROS2

## Functionality

The Reference Costmap Generator creates two types of costmaps from a reference path:

1. **Distance Error Map**: For each cell in the map, calculates the distance to the nearest point in the reference path
2. **Reference Yaw Map**: For each cell in the map, calculates the yaw angle of the nearest point in the reference path

### Publishers
- `goal_pose_marker`: Visualization marker for the goal pose (sphere or arrow)
- `distance_error_map`: Grid map containing distance errors
- `ref_yaw_map`: Grid map containing reference yaw angles

### Subscribers
- `ref_path`: Reference path from path planner
- `map`: Occupancy grid map

## Build Instructions

```bash
# Navigate to your ROS2 workspace
cd /path/to/your/ros2_ws

# Build the package
colcon build --packages-select reference_costmap_generator

# Source the workspace
source install/setup.bash

# Launch the node
ros2 launch reference_costmap_generator reference_costmap_generator.launch.py
```

## Testing

To verify the conversion:

1. Check that the node starts without errors
2. Verify all topic subscriptions and publications are working
3. Confirm parameter loading from the YAML file
4. Test the costmap generation functionality
5. Verify visualization markers are published correctly

## Dependencies

Ensure these ROS2 packages are installed:
- `rclcpp`
- `nav_msgs`
- `visualization_msgs`
- `tf2_ros`
- `tf2_geometry_msgs`
- `grid_map_ros`
- `grid_map_msgs`
- `grid_map_core`
- `grid_map_visualization`

## Performance Features

- **OpenMP Support**: The package supports parallel processing using OpenMP for improved performance when calculating distance and yaw maps
- **Configurable Resolution**: Map resolution can be scaled using the `map_resolution_scale` parameter for performance tuning
