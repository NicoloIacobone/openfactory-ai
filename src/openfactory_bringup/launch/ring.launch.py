"""Ring of N arms (phase 6, D-020): Gazebo simulation with one RGB-D camera per bin, one
MoveIt 2 move_group for every arm, one scene perception per bin (+ RViz).

    ros2 launch openfactory_bringup ring.launch.py n:=3 rviz:=false

Arm i picks from bin i and places into bin (i + 1) mod n; the loop itself is
`ros2 run openfactory_manipulation relay --arms 3`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(package, file_name, arguments, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch', file_name])),
        launch_arguments=arguments.items(),
        condition=condition,
    )


def generate_launch_description():
    n = LaunchConfiguration('n')
    return LaunchDescription([
        DeclareLaunchArgument('n', default_value='2', description='Number of arms (>= 2).'),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution(
                [FindPackageShare('openfactory_simulation'), 'worlds', 'cell.sdf']),
            description='Gazebo world file (absolute path).'),
        DeclareLaunchArgument(
            'headless', default_value='true', description='Run the Gazebo server only.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.'),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='Simulate the RGB-D camera sensors (need a display, D-005).'),
        DeclareLaunchArgument(
            'depth_noise', default_value='true',
            description='Apply the depth error model of the cameras (D-011).'),
        DeclareLaunchArgument('seed', default_value='1', description='Seed of the depth noise.'),
        DeclareLaunchArgument(
            'perception', default_value='true', description='Start scene_perception.'),
        _include('openfactory_simulation', 'ring_simulation.launch.py', {
            'n': n,
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
            'camera': LaunchConfiguration('camera'),
            'depth_noise': LaunchConfiguration('depth_noise'),
            'seed': LaunchConfiguration('seed'),
        }),
        _include('openfactory_moveit_config', 'ring_move_group.launch.py', {
            'n': n,
            'use_sim_time': 'true',
            'rviz': LaunchConfiguration('rviz'),
        }),
        _include('openfactory_perception', 'ring_perception.launch.py',
                 {'n': n, 'use_sim_time': 'true'},
                 condition=IfCondition(LaunchConfiguration('perception'))),
    ])
