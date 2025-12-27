from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    
    pkg_home_cleaner_sim = FindPackageShare("home_cleaner_sim")
    pkg_home_cleaning_robot = FindPackageShare("home_cleaning_robot")
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")

    
    world_path = PathJoinSubstitution([pkg_home_cleaner_sim, "worlds", "home_layout.sdf"])
    
    
    xacro_path = PathJoinSubstitution([pkg_home_cleaning_robot, "urdf", "cleaning_bot.xacro"])

    #Gazebo'yu Başlat
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={'gz_args': ['-r ', world_path]}.items(),
    )

    # Robot State Publisher 
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_path]), value_type=str),
            'use_sim_time': True 
        }]
    )

    # Robotu Gazebo'ya Doğur (Spawn)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'home_cleaner_bot',
            '-topic', 'robot_description', #
            '-x', '0.0',
            '-y', '-2.0',
            '-z', '0.2'
        ],
        output='screen'
    )

    #ROS-Gazebo Köprüsü (BRIDGE) - EKSİK OLAN KISIM BUYDU
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@gz.msgs/msg/Twist@gz.msgs/msg/Twist',
            '/scan@sensor_msgs/msg/LaserScan@sensor_msgs/msg/LaserScan',
            '/odom@nav_msgs/msg/Odometry@nav_msgs/msg/Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V', 
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' 
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge
    ])
