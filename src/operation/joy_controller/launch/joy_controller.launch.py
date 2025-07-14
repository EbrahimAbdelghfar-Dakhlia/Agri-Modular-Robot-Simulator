from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Declare launch arguments
    joy_param_path_arg = DeclareLaunchArgument(
        'joy_param_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('joy_controller'),
            'config',
            'joy.yaml'
        ]),
        description='Path to joy configuration file'
    )
    
    joy_controller_param_path_arg = DeclareLaunchArgument(
        'joy_controller_param_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('joy_controller'),
            'config',
            'joy_controller.yaml'
        ]),
        description='Path to joy_controller configuration file'
    )

    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
    )

    # Joy controller node
    joy_controller_node = Node(
        package='joy_controller',
        executable='joy_controller_node',
        name='joy_controller',
        output='screen',
        parameters=[LaunchConfiguration('joy_controller_param_path')]
    )

    return LaunchDescription([
        joy_param_path_arg,
        joy_controller_param_path_arg,
        joy_node,
        joy_controller_node
    ])
