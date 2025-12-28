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

    #ROS-Gazebo Köprüsü
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': True}],
        arguments=[
            # Hareket: ROS -> Gazebo (Model ismine göre yönlendiriyoruz)
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/home_cleaner_bot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            
            # Lidar ve Odom: Gazebo -> ROS
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            
            # TF ve Saat
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' 
        ],
        output='screen'
    )
    
    # İsim karmaşasını çözen yapıştırıcı (Static Transform)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_lidar',
        parameters=[{'use_sim_time': True}],
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'home_cleaner_bot/base_link/lidar_sensor']
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        static_tf
    ])
