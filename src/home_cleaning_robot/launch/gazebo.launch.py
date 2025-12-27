from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # The path to the world SDF file.
    world = PathJoinSubstitution([
        FindPackageShare("home_cleaner_sim"),
        "worlds",
        "home_layout.sdf"
    ])

    # The path to the URDF robot model.
    urdf = PathJoinSubstitution([
        FindPackageShare("home_cleaning_robot"),
        "urdf",
        "cleaning_bot.xacro"
    ])

    # Gazebo Harmonic launch arguments. (You must install ros_gz bridge, recommended is *ros-humble-ros-gzharmonic*)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': ['-r ', world, ' '],
        }.items()
    )

    # Publish the robot's state.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(
                Command(['xacro ', urdf]),
                value_type=str
            )
        }]
    )

    # Spawn robot into Gazebo.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'cleaning_bot',
            '-topic', '/robot_description',
            '-z', '0.5'
        ],
        output='screen'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            'odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/world/default/model/cleaning_bot/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model'
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom'),
            ('/scan', '/scan')
        ],
        output='screen'
    )

    # Launch everything in order.
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge
    ])
