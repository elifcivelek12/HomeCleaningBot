import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'home_cleaner_sim'
    pkg_share = get_package_share_directory(pkg_name)

    # Dünya dosyamızın yolu
    world_file = os.path.join(pkg_share, 'worlds', 'home_layout.sdf')

    # Gazebo Simulatörünü Başlat
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # İleride Robot Spawn (Doğurma) ve Bridge buraya eklenebilir. Bu launch dosyası şuan için sadece evi simulasyon ortamında görüntülemek için

    return LaunchDescription([
        gazebo
    ])
