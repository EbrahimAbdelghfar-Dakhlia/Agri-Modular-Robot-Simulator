import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('mppi_3d')
    
    # Declare the launch argument
    mppi_3d_param_path_arg = DeclareLaunchArgument(
        'mppi_3d_param_path',
        default_value=os.path.join(pkg_dir, 'config', 'mppi_3d_a.yaml'),
        description='Path to the MPPI 3D parameters file'
    )

    # Define the node
    mppi_3d_node = Node(
        package='mppi_3d',
        executable='mppi_3d_node',
        name='mppi_3d',
        output='screen',
        parameters=[LaunchConfiguration('mppi_3d_param_path')]
    )

    return LaunchDescription([
        mppi_3d_param_path_arg,
        mppi_3d_node,
    ])
