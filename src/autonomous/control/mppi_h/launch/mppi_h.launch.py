import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    mppi_h_param_path = os.path.join(
        get_package_share_directory('mppi_h'),
        'config',
        'mppi_h.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mppi_h_param_path',
            default_value=mppi_h_param_path,
            description='Path to the MPPI H parameters file'
        ),

        Node(
            package='mppi_h',
            executable='mppi_h_node',
            name='mppi_h',
            output='screen',
            parameters=[LaunchConfiguration('mppi_h_param_path')]
        ),
    ])
