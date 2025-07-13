# vel_driver - ROS2 Package

This package has been converted from ROS1 to ROS2. It provides a velocity driver for swerve drive vehicles in Gazebo simulation.

## Features

- Converts Twist commands to individual wheel steering and rotation commands
- Supports 4-wheel swerve drive vehicles
- Configurable vehicle parameters via YAML file

## Building

```bash
colcon build --packages-select vel_driver
```

## Running

```bash
ros2 launch vel_driver vel_driver.launch.py
```

## Configuration

Edit `config/vel_driver.yaml` to modify:
- Topic names for input/output
- Vehicle parameters (wheelbase, track width, tire radius)

## Changes from ROS1

- Updated to use `rclcpp` instead of `roscpp`
- Changed message types to ROS2 format (`geometry_msgs::msg::Twist`)
- Updated launch file to Python format
- Modified parameter handling to use ROS2 parameter system
- Updated logging to use `RCLCPP_*` macros
This package convert 3DoF twist message (vx, vy, yaw_rate) to 8DoF vehicle command message and publish it.  
Gazebo will subscribe the 8DoF vehicle command message and apply it to the 4WIDS vehicle model in the simulation.


## Subscribing Topics

| Topic name      | Type                 | Description                                                  |
| --------------- | -------------------- | ------------------------------------------------------------ |
| /cmd_vel        | geometry_msgs/Twist  | Target vx, vy, yaw_rate (on the Global Frame) to follow      |


## Publishing Topics

| Topic name      | Type              | Description                                                  |
| --------------- | ----------------- | ------------------------------------------------------------ |
| /fwids/front_left_steer_rad/command | std_msgs/Float64 | The target steering angle [rad] of the front left wheel. |
| /fwids/front_left_rotor_radpersec/command | std_msgs/Float64 | The target rotor speed [rad/s] of the front left wheel. |
| /fwids/front_right_steer_rad/command | std_msgs/Float64 | The target steering angle [rad] of the front right wheel. |
| /fwids/front_right_rotor_radpersec/command | std_msgs/Float64 | The target rotor speed [rad/s] of the front right wheel. |
| /fwids/rear_left_steer_rad/command | std_msgs/Float64 | The target steering angle [rad] of the rear left wheel. |
| /fwids/rear_left_rotor_radpersec/command | std_msgs/Float64 | The target rotor speed [rad/s] of the rear left wheel. |
| /fwids/rear_right_steer_rad/command | std_msgs/Float64 | The target steering angle [rad] of the rear right wheel. |
| /fwids/rear_right_rotor_radpersec/command | std_msgs/Float64 | The target rotor speed [rad/s] of the rear right wheel. |


## Node Parameters
| Parameter name               | Type   | Description                                                  |
| ---------------------------- | ------ | ------------------------------------------------------------ |
| l_f                         | double | length from the front axle to the center of mass [m] |
| l_r                         | double | length from the rear axle to the center of mass [m] |
| d_l                         | double | length from the left wheel to the center of mass [m] |
| d_r                         | double | length from the right wheel to the center of mass [m] |
| tire_radius                 | double | tire radius of the target 4WIDS vehicle [m] |


## Usage
To launch the node individually, run the following commands.
```
source devel/setup.bash
roslaunch vel_driver vel_driver.launch
```


## Note
The conversion between 3DoF twist message and 8DoF vehicle command message is based on 
the assumption that there are no tire slip. See the following reference for more details.  
https://arxiv.org/abs/2409.08648
