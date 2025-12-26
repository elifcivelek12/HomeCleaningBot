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
        ('share/' + package_name + '/launch', ['launch/robot_state.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/cleaning_bot.urdf', 'urdf/r2d2.rviz'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mehmet Sefa Toksoy, Elif Ceyda Civelek',
    maintainer_email='elifceydacivelek@istun.edu.tr, mehmetsefatoksoy@istun.edu.tr',
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
        ],
    },
)
