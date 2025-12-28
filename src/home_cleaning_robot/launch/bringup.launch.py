import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Complete launch file for the home cleaning robot with Gazebo, RViz, and Nav2
    """
    # Package directories
    pkg_home_cleaning_robot = get_package_share_directory('home_cleaning_robot')
    
    # Paths
    rviz_config_file = os.path.join(pkg_home_cleaning_robot, 'config', 'nav2.rviz')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Include Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_home_cleaning_robot, 'launch', 'gazebo.launch.py')
        )
    )
    
    # Include Nav2 launch with delay to allow Gazebo and robot to initialize
    nav2_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_home_cleaning_robot, 'launch', 'nav2.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time
                }.items()
            )
        ]
    )
    
    # Launch RViz2 with Nav2 configuration
    rviz_launch = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )
    
    # Set initial pose after Nav2 is up
    initial_pose_setter = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='home_cleaning_robot',
                executable='set_initial_pose',
                name='initial_pose_setter',
                output='screen'
            )
        ]
    )

    control_panel = Node(
        package='home_cleaning_robot',
        executable='control_panel',
        name='cleaning_control_panel',
        output='screen'
    )
    
    # Behavior Manager Node
    behavior_manager = Node(
        package='home_cleaning_robot',
        executable='behavior_manager',
        name='behavior_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add launch files and nodes
    ld.add_action(gazebo_launch)
    ld.add_action(nav2_launch)
    ld.add_action(rviz_launch)
    ld.add_action(initial_pose_setter)

    ld.add_action(behavior_manager)

    ld.add_action(control_panel)
    
    return ld
