from setuptools import find_packages, setup

package_name = 'home_cleaning_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch klasöründeki dosyalar
        ('share/' + package_name + '/launch', [
            'launch/gazebo.launch.py',
            'launch/slam.launch.py',
            'launch/nav2.launch.py',
            'launch/bringup.launch.py'
        ]),
        # URDF ve RViz dosyaları
        ('share/' + package_name + '/urdf', [
            'urdf/cleaning_bot.xacro', 
            'urdf/r2d2.rviz'
        ]),
        # Config klasörü ve içindeki YAML dosyası 
        ('share/' + package_name + '/config', [
            'config/mapper_params_online_async.yaml',
            'config/my_home_map.yaml',
            'config/my_home_map.pgm',
            'config/nav2_params.yaml',
            'config/nav2.rviz',
        ]),
    ],    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mehmet Sefa Toksoy, Elif Ceyda Civelek',
    maintainer_email='elif.civelek@istun.edu.tr, mehmet.toksoy@istun.edu.tr',
    description='Home Cleaner Bot Robot URDF Package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'state_publisher = home_cleaning_robot.state_publisher:main',
            'set_initial_pose = home_cleaning_robot.initial_pose_setter:main',
            'behavior_manager = home_cleaning_robot.behavior_manager:main',
            'control_panel = home_cleaning_robot.cleaning_control_panel:main',
        ],
    },
)
