"""scene_perception: geometric object detection in the input bin from the RGB-D camera."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('openfactory_perception'), 'config', 'perception.yaml']),
            description='Perception parameters (absolute path).'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use the simulator clock.'),
        Node(
            package='openfactory_perception',
            executable='scene_perception',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen',
        ),
    ])
