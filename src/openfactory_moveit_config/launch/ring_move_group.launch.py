"""MoveIt 2 move_group (and optionally RViz) for the ring of N arms (phase 6, D-020).

    ros2 launch openfactory_moveit_config ring_move_group.launch.py n:=3

One move_group plans every arm: the URDF holds all of them, with one planning group per arm
(`arm_<i>_manipulator`). SRDF, kinematics, joint limits and controllers of one N are generated
at launch from the configuration of the single-arm cell (openfactory_moveit_config/ring_config.py).
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from openfactory_description import ring
from openfactory_moveit_config import ring_config


def _share(package):
    return Path(get_package_share_directory(package))


def _ring(context):
    n = int(LaunchConfiguration('n').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time')
    files = ring_config.write(n)

    moveit_config = (
        MoveItConfigsBuilder(robot_name='ur', package_name='ur_moveit_config')
        # Same xacro as robot_state_publisher; the Gazebo controllers path is irrelevant here.
        .robot_description(
            file_path=str(_share('openfactory_description') / 'urdf' / 'ring.urdf.xacro'),
            mappings={'layout_file': str(ring.write_layout(n)), 'include_fixtures': 'true'})
        .robot_description_semantic(file_path=str(files['srdf']))
        .robot_description_kinematics(file_path=str(files['kinematics']))
        .joint_limits(file_path=str(files['joint_limits']))
        .trajectory_execution(file_path=str(files['controllers']))
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time, 'publish_robot_description_semantic': True},
        ],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        arguments=['-d', str(_share('ur_moveit_config') / 'config' / 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='log',
    )
    return [move_group, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('n', default_value='2', description='Number of arms (>= 2).'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use the simulator clock.'),
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz.'),
        OpaqueFunction(function=_ring),
    ])
