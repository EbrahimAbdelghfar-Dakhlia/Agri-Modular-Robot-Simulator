from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    reference_costmap_generator_param_path = os.path.join(
        get_package_share_directory('reference_costmap_generator'),
        'config',
        'reference_costmap_generator.yaml'
    )

    return LaunchDescription([
        Node(
            package='reference_costmap_generator',
            executable='reference_costmap_generator_node',
            name='reference_costmap_generator',
            output='screen',
            parameters=[reference_costmap_generator_param_path]
        ),
    ])
