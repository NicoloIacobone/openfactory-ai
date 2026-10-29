"""Whole cell: Gazebo simulation + MoveIt 2 move_group (+ RViz)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(package, file_name, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch', file_name])),
        launch_arguments=arguments.items(),
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
        _include('openfactory_simulation', 'simulation.launch.py', {
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
        }),
        _include('openfactory_moveit_config', 'move_group.launch.py', {
            'use_sim_time': 'true',
            'rviz': LaunchConfiguration('rviz'),
        }),
    ])
