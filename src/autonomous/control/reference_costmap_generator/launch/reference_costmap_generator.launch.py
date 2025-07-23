#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package share directory
    package_dir = get_package_share_directory('reference_costmap_generator')
    
    # Declare launch arguments
    reference_costmap_generator_param_path_arg = DeclareLaunchArgument(
        'reference_costmap_generator_param_path',
        default_value=os.path.join(package_dir, 'config', 'reference_costmap_generator.yaml'),
        description='Path to Reference Costmap Generator parameter file'
    )
    
    # Create the Reference Costmap Generator node
    reference_costmap_generator_node = Node(
        package='reference_costmap_generator',
        executable='reference_costmap_generator_node',
        name='reference_costmap_generator',
        output='screen',
        parameters=[LaunchConfiguration('reference_costmap_generator_param_path')]
    )
    
    return LaunchDescription([
        reference_costmap_generator_param_path_arg,
        reference_costmap_generator_node
    ])
