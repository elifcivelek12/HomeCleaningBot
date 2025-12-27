# ESKI LAUNCH DOSYASI. (Referans icin duruyor.)

#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('home_cleaning_robot')

    # URDF file path
    urdf_file = os.path.join(package_dir, 'urdf', 'cleaning_bot.urdf')

    # Read URDF content
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # Joint State Publisher
        Node(
            package='home_cleaning_robot',
            executable='state_publisher',
            name='state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # Joint State Publisher GUI (This allows us to rotate the wheels manually.)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            arguments=[urdf_file],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # RViz2 (For visualizing the robot. *This will be replaced with Gazebo view later.*)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(package_dir, 'urdf', 'r2d2.rviz')],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }] 
        )
    ])