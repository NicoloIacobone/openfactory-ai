"""scene_perception for the ring of N arms (phase 6): one instance per bin, in the namespace of
the bin (`/bin_<k>/perception/objects` from `/bin_<k>/camera/...`).

    ros2 launch openfactory_perception ring_perception.launch.py n:=3
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from openfactory_description import ring
from openfactory_perception import constants as c


def _nodes(context):
    n = ring.check_n(int(LaunchConfiguration('n').perform(context)))
    return [
        Node(
            package='openfactory_perception',
            executable='scene_perception',
            namespace=ring.bin_namespace(index),
            remappings=[(topic, ring.bin_topic(index, topic)) for topic in c.TOPICS],
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'bin_frame': ring.bin_name(index),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen',
        )
        for index in range(n)
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('n', default_value='2', description='Number of arms (>= 2).'),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('openfactory_perception'), 'config', 'perception.yaml']),
            description='Perception parameters (absolute path).'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use the simulator clock.'),
        OpaqueFunction(function=_nodes),
    ])
