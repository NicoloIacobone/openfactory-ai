"""Test bench of the Rubik's cube mechanism (phase 7): the bench world alone, Gazebo server
only. The cube is spawned and driven by scripts/rubik_mechanism_test.py.

    ros2 launch openfactory_simulation rubik_bench.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution(
        [FindPackageShare('openfactory_simulation'), 'worlds', 'rubik_bench.sdf'])
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
            launch_arguments={'gz_args': ['-s -r -v 3 ', world],
                              'on_exit_shutdown': 'true'}.items(),
        ),
    ])
