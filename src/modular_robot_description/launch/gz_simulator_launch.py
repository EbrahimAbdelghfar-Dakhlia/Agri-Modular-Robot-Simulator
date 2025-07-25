import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument,SetEnvironmentVariable
from launch_ros.actions import Node
import launch_ros.descriptions
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
def has_nvidia_gpu():
    """
    Check if the system has an NVIDIA GPU on Linux.
    Returns True if an NVIDIA GPU is detected, False otherwise.
    Uses only standard libraries.
    """
    import subprocess
    
    try:
        # Try using lspci command (common on most Linux distributions)
        process = subprocess.run(['lspci'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if 'nvidia' in process.stdout.lower():
            return True
            
        # As a backup, check if nvidia-smi command exists and runs successfully
        nvidia_smi = subprocess.run(['nvidia-smi'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return nvidia_smi.returncode == 0
            
    except FileNotFoundError:
        # lspci or nvidia-smi command not found
        return False
    except Exception:
        # Any other error, assume no NVIDIA GPU
        return False

def generate_launch_description():
    ld=LaunchDescription()
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    urdf_file= LaunchConfiguration('urdf_file')
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    Pkg_directory = get_package_share_directory('virtual_maize_field') #this is the package name that contains the models
    bringup_dir = get_package_share_directory('modular_robot_description')
    world = os.path.join(bringup_dir , "world", "generated.world")
    sdf_file  =  os.path.join(bringup_dir, 'urdf', 'robot.xacro.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Declare launch arguments
    vel_driver_param_path_arg = DeclareLaunchArgument(
        'vel_driver_param_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('vel_driver'),
            'config',
            'vel_driver.yaml'
        ]),
        description='Path to the vel_driver parameter file'
    )

    # Create the node
    vel_driver_node = Node(
        package='vel_driver',
        executable='vel_driver_node',
        name='vel_driver',
        output='screen',
        parameters=[LaunchConfiguration('vel_driver_param_path')]
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc}
        ])

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(Pkg_directory, 'models'),
            str(Path(Pkg_directory).parent.resolve()),
            os.path.join(bringup_dir, 'world'),
            str(Path(bringup_dir).parent.resolve())
        ])
    )

    gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_MODEL_PATH',
        value=':'.join([
            os.path.join(Pkg_directory, 'models'),
            str(Path(Pkg_directory).parent.resolve())
        ])
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(bringup_dir, 'config', 'gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    robot_control = Node(
        package='modular_robot_description',
        executable='robot_control',
        name='robot_control',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    if has_nvidia_gpu(): #check if the gpu is nvidia to export the following variables to make simulation work on gpu
        nvidia_prime = SetEnvironmentVariable(
        name='__NV_PRIME_RENDER_OFFLOAD',
        value='1')

        nvidia_ = SetEnvironmentVariable(
            name='__GLX_VENDOR_LIBRARY_NAME',
            value='nvidia')
        
        ld.add_action(nvidia_prime)
        ld.add_action(nvidia_)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={"gz_args": ["-r -v 4 ", world]}.items(),
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "robot",
            "-topic",
            "/robot_description",
            "-x",
            "3.7",
            "-y",
            "-1.6",
            "-z",
            "0.70",
            "-Y",
            "0.0",
        ],
        output="screen",
    )


    tf_map= Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments= ["0", "0", "0", "0", "0", "0", "map", "odom"])
    
    ld.add_action(DeclareLaunchArgument('use_sim_time',default_value='True',description='Use sim time if true'))
    ld.add_action(DeclareLaunchArgument('urdf_file',default_value=os.path.join(bringup_dir, 'urdf', 'assembly_robot.urdf'),description='Whether to start RVIZ'))
    ld.add_action(DeclareLaunchArgument('use_robot_state_pub',default_value='True',description='Whether to start the robot state publisher'))
    ld.add_action(gz_resource_path)
    ld.add_action(gz_model_path)
    ld.add_action(gz_sim)
    ld.add_action(bridge)
    ld.add_action(spawn_entity)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(tf_map)
    ld.add_action(joint_state_publisher)
    ld.add_action(robot_control)
    ld.add_action(vel_driver_param_path_arg)
    ld.add_action(vel_driver_node)
    # Launch!
    return ld