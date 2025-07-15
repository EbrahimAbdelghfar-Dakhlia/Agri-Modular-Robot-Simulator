import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    mppi_4d_param_path = LaunchConfiguration('mppi_4d_param_path', default=os.path.join(
        get_package_share_directory('mppi_4d'), 'config', 'mppi_4d.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'mppi_4d_param_path',
            default_value=mppi_4d_param_path,
            description='Path to the MPPI 4D parameters file'
        ),

        Node(
            package='mppi_4d',
            executable='mppi_4d_node',
            name='mppi_4d',
            output='screen',
            parameters=[mppi_4d_param_path]
        ),
    ])
