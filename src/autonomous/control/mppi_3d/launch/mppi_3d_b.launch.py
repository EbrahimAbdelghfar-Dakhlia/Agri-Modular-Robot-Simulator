import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    mppi_3d_param_path = LaunchConfiguration('mppi_3d_param_path', default=os.path.join(
        get_package_share_directory('mppi_3d'), 'config', 'mppi_3d_b.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'mppi_3d_param_path',
            default_value=mppi_3d_param_path,
            description='Path to the MPPI 3D parameters file'
        ),

        Node(
            package='mppi_3d',
            executable='mppi_3d_node',
            name='mppi_3d',
            output='screen',
            parameters=[mppi_3d_param_path]
        ),
    ])
