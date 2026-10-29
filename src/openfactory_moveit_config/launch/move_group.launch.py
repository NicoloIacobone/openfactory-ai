"""MoveIt 2 move_group (and optionally RViz) for the cell.

Reuses ur_moveit_config for kinematics, joint limits and the OMPL pipeline; the URDF,
SRDF and controllers are the project's own. The builder logs "Cannot infer URDF/SRDF" for
ur_moveit_config before they are overridden below; the warnings are harmless.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _share(package):
    return Path(get_package_share_directory(package))


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    moveit_config = (
        MoveItConfigsBuilder(robot_name='ur', package_name='ur_moveit_config')
        # Same xacro as robot_state_publisher; the Gazebo controllers path is irrelevant here.
        .robot_description(
            file_path=str(_share('openfactory_description') / 'urdf' / 'openfactory_cell.urdf.xacro'),
            mappings={'include_table': 'true'})
        .robot_description_semantic(
            file_path=str(_share('openfactory_moveit_config') / 'srdf' / 'openfactory_cell.srdf.xacro'))
        .trajectory_execution(
            file_path=str(_share('openfactory_moveit_config') / 'config' / 'moveit_controllers.yaml'))
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
        condition=IfCondition(rviz),
        output='log',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use the simulator clock.'),
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz.'),
        move_group,
        rviz_node,
    ])
