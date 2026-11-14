"""Rubik's cube cell (phase 7, D-024): Gazebo simulation with the nest and the RGB-D camera
above it, MoveIt 2 move_group (+ RViz). The cube is spawned by the harness
(scripts/rubik_benchmark.py) or by hand (scripts/rubik_scene.py).

    ros2 launch openfactory_bringup rubik.launch.py rviz:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

ROBOT_XACRO = 'rubik_cell.urdf.xacro'
FIXTURES_XACRO = 'rubik_fixtures.urdf.xacro'
SRDF_XACRO = 'rubik_cell.srdf.xacro'


def _include(package, file_name, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch', file_name])),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='true', description='Run the Gazebo server only.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.'),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='Simulate the RGB-D camera sensor (needs a display, D-005).'),
        DeclareLaunchArgument(
            'depth_noise', default_value='true',
            description='Apply the depth error model of the camera (D-011).'),
        DeclareLaunchArgument('seed', default_value='1', description='Seed of the depth noise.'),
        _include('openfactory_simulation', 'simulation.launch.py', {
            'headless': LaunchConfiguration('headless'),
            'camera': LaunchConfiguration('camera'),
            'depth_noise': LaunchConfiguration('depth_noise'),
            'seed': LaunchConfiguration('seed'),
            'robot_xacro': ROBOT_XACRO,
            'fixtures_xacro': FIXTURES_XACRO,
        }),
        _include('openfactory_moveit_config', 'move_group.launch.py', {
            'use_sim_time': 'true',
            'rviz': LaunchConfiguration('rviz'),
            'urdf': ROBOT_XACRO,
            'srdf': SRDF_XACRO,
        }),
    ])
