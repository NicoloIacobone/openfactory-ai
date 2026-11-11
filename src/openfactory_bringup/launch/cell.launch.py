"""Whole cell: Gazebo simulation with the RGB-D camera, MoveIt 2 move_group, scene
perception (+ RViz)."""

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
    return LaunchDescription([
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
            description='Simulate the RGB-D camera sensor (needs a display, D-005).'),
        DeclareLaunchArgument(
            'depth_noise', default_value='true',
            description='Apply the depth error model of the camera (D-011).'),
        DeclareLaunchArgument('seed', default_value='1', description='Seed of the depth noise.'),
        DeclareLaunchArgument(
            'perception', default_value='true', description='Start scene_perception.'),
        _include('openfactory_simulation', 'simulation.launch.py', {
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
            'camera': LaunchConfiguration('camera'),
            'depth_noise': LaunchConfiguration('depth_noise'),
            'seed': LaunchConfiguration('seed'),
        }),
        _include('openfactory_moveit_config', 'move_group.launch.py', {
            'use_sim_time': 'true',
            'rviz': LaunchConfiguration('rviz'),
        }),
        _include('openfactory_perception', 'perception.launch.py', {'use_sim_time': 'true'},
                 condition=IfCondition(LaunchConfiguration('perception'))),
    ])
