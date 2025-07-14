#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo 
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.conditions import IfCondition
from time import sleep
def generate_launch_description():

    hardware_connected = LaunchConfiguration('hardware_connected')
    odometry_source = LaunchConfiguration('odom_source')
    baud_rate = LaunchConfiguration('baud_rate')

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )

    Odometry_source_arg = DeclareLaunchArgument(
        'odom_source',
        default_value='scan',
        choices=['scan', 'camera'],
        description='Choose which odometry source to use: scan or camera'
    )

    bring_up_hardwares_arg = DeclareLaunchArgument(
        'hardware_connected',
        default_value='true',
        choices=['true', 'false'],
        description='Choose which hardware to bring up: true or false'
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('agri_robot_description'), 'launch/description_launch.py'),
        )
    )

    sllidar_and_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch/sllidar_s2_launch.py'),
        )
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch/rs_launch.py')),
            launch_arguments={
                'pointcloud.enable': 'true',
                'depth_module.depth_profile': '640x480x15',
                'rgb_camera.color_profile': '640x480x15',
                'temporal_filter.enable': 'true',
                'spatial_filter.enable': 'true',
            }.items()
    )

    ros_agent = Node(
        package='robot_bringup',  # Replace with your package name
        executable='micro_ros_agent_launcher',
        name='micro_ros_agent_launcher',
        output='screen',
        condition=IfCondition(hardware_connected),
        parameters=[
            {'baud_rate': baud_rate},      # Default baud rate, can be overridden
        ])
    
    scan_odometry = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        condition=IfCondition(PythonExpression(['"', odometry_source, '" == "scan"'])),
        parameters=[{
            'laser_scan_topic' : '/scan',
            'odom_topic' : '/odom',
            'publish_tf' : True,
            'base_frame_id' : 'base_link',
            'odom_frame_id' : 'odom',
            'init_pose_from_topic' : '',
            'freq' : 20.0}],)
    
    camera_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        condition=IfCondition(PythonExpression(['"', odometry_source, '" == "camera"'])),)
    
    ld = LaunchDescription()

    ld.add_action(baud_rate_arg)
    ld.add_action(Odometry_source_arg)
    ld.add_action(bring_up_hardwares_arg)
    ld.add_action(ros_agent)
    ld.add_action(robot_description_launch)
    ld.add_action(realsense_launch)
    ld.add_action(sllidar_and_odometry_launch)
    ld.add_action(scan_odometry)
    ld.add_action(camera_odometry)
    
    return ld