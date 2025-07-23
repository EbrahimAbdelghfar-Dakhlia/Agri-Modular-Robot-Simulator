# MPPI 4D Controller - ROS2 Conversion

This document outlines the conversion of the MPPI 4D controller from ROS1 to ROS2.

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
- **ROS1**: `geometry_msgs::Twist` → **ROS2**: `geometry_msgs::msg::Twist`
- **ROS1**: `std_msgs::Float32` → **ROS2**: `std_msgs::msg::Float32`
- All message types now include `::msg::` namespace

### 5. Callback Signatures
- **ROS1**: `const MessageType::ConstPtr& msg`
- **ROS2**: `const MessageType::SharedPtr msg`

### 6. Publishing
- **ROS1**: `publisher.publish(message)`
- **ROS2**: `publisher->publish(message)`

### 7. Timers
- **ROS1**: `private_nh_.createTimer(ros::Duration(interval), callback)`
- **ROS2**: `this->create_wall_timer(std::chrono::duration<double>(interval), callback)`

### 8. Logging
- **ROS1**: `ROS_WARN("message")`
- **ROS2**: `RCLCPP_WARN(this->get_logger(), "message")`

### 9. Time
- **ROS1**: `ros::Time::now()`
- **ROS2**: `this->get_clock()->now()`

### 10. Duration
- **ROS1**: `ros::Duration(seconds)`
- **ROS2**: `rclcpp::Duration::from_seconds(seconds)`

## File Changes

### Source Files
- `src/mppi_4d.cpp`: Complete conversion to ROS2 APIs
- `src/mppi_4d_node.cpp`: Updated main function for ROS2
- `include/mppi_4d/mppi_4d.hpp`: Updated class declarations and message types

### Configuration Files
- `package.xml`: Already ROS2 format, fixed package name
- `CMakeLists.txt`: Cleaned up ROS1 references
- `config/mppi_4d.yaml`: Updated to ROS2 parameter format with `ros__parameters`
- `launch/mppi_4d.launch.py`: Created new Python launch file for ROS2

## Removed Features
- **Overlay Text Publisher**: The `jsk_rviz_plugins::OverlayText` functionality was removed as it's ROS1-specific and not directly available in ROS2.

## Build Instructions

```bash
# Navigate to your ROS2 workspace
cd /path/to/your/ros2_ws

# Build the package
colcon build --packages-select mppi_4d

# Source the workspace
source install/setup.bash

# Launch the node
ros2 launch mppi_4d mppi_4d.launch.py
```

## Testing

To verify the conversion:

1. Check that the node starts without errors
2. Verify all topic subscriptions and publications are working
3. Confirm parameter loading from the YAML file
4. Test the control loop functionality

## Dependencies

Ensure these ROS2 packages are installed:
- `rclcpp`
- `geometry_msgs`
- `nav_msgs`
- `visualization_msgs`
- `tf2_ros`
- `tf2_geometry_msgs`
- `grid_map_ros`
- `grid_map_msgs`
- `mppi_eval_msgs`
