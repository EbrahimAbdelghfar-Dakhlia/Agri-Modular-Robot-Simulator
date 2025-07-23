#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package share directory
    package_dir = get_package_share_directory('mppi_4d')
    
    # Declare launch arguments
    mppi_4d_param_path_arg = DeclareLaunchArgument(
        'mppi_4d_param_path',
        default_value=os.path.join(package_dir, 'config', 'mppi_4d.yaml'),
        description='Path to MPPI 4D parameter file'
    )
    
    # Create the MPPI 4D node
    mppi_4d_node = Node(
        package='mppi_4d',
        executable='mppi_4d_node',
        name='mppi_4d',
        output='screen',
        parameters=[LaunchConfiguration('mppi_4d_param_path')]
    )
    
    return LaunchDescription([
        mppi_4d_param_path_arg,
        mppi_4d_node
    ])
