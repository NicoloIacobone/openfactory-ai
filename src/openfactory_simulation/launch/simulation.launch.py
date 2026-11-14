"""Gazebo cell: world, fixtures (table, bins, RGB-D camera), UR5e with gz_ros2_control, bridge,
controllers. The Rubik's cube cell (phase 7) is the same launch with its own robot and fixtures
xacro (robot_xacro:=rubik_cell.urdf.xacro fixtures_xacro:=rubik_fixtures.urdf.xacro).

With camera:=true (default) Gazebo renders the camera with Ogre 1, which needs the VM display
even when headless:=true (D-005): launch from a desktop terminal, or set DISPLAY and XAUTHORITY.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ROBOT_NAME = 'ur'
FIXTURES_NAME = 'fixtures'
CONTROLLERS = ['joint_state_broadcaster', 'joint_trajectory_controller', 'gripper_controller']


def _xacro(file_name, **args):
    command = [
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('openfactory_description'), 'urdf', file_name]),
    ]
    for key, value in args.items():
        command += [' ', f'{key}:=', value]
    return Command(command)


def _spawn(name, description):
    return Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', name, '-string', description],
        output='screen',
    )


def generate_launch_description():
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    controllers_file = PathJoinSubstitution(
        [FindPackageShare('openfactory_simulation'), 'config', 'ros2_controllers.yaml'])

    # Same xacro and arguments for robot_state_publisher (which gz_ros2_control reads)
    # and the spawned robot; only the fixtures are split off into its own Gazebo model.
    robot_xacro = LaunchConfiguration('robot_xacro')
    robot_with_fixtures = _xacro(
        robot_xacro,
        include_fixtures='true', simulation_controllers=controllers_file)
    robot_only = _xacro(
        robot_xacro,
        include_fixtures='false', simulation_controllers=controllers_file)
    fixtures = _xacro(LaunchConfiguration('fixtures_xacro'),
                      camera_sensor=LaunchConfiguration('camera'))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            # D-005: Ogre 1 for the GUI; the server engine is set in the world's Sensors system
            'gz_args': IfElseSubstitution(
                headless,
                if_value=['-s -r -v 3 ', world],
                else_value=['-r -v 3 --render-engine ogre ', world]),
            'on_exit_shutdown': 'true',
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution(
                [FindPackageShare('openfactory_simulation'), 'config', 'bridge.yaml']),
        }],
        output='screen',
    )

    # D-011: ideal Gazebo depth -> depth with the camera's error model
    depth_sensor_model = Node(
        package='openfactory_simulation',
        executable='depth_sensor_model',
        parameters=[{
            'noise': ParameterValue(LaunchConfiguration('depth_noise'), value_type=bool),
            'seed': ParameterValue(LaunchConfiguration('seed'), value_type=int),
            'use_sim_time': True,
        }],
        condition=IfCondition(LaunchConfiguration('camera')),
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_with_fixtures, value_type=str),
            'use_sim_time': True,
        }],
        output='screen',
    )

    spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager'],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )
        for name in CONTROLLERS
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution(
                [FindPackageShare('openfactory_simulation'), 'worlds', 'cell.sdf']),
            description='Gazebo world file (absolute path).'),
        DeclareLaunchArgument(
            'headless', default_value='true',
            description='Run the Gazebo server only (no GUI).'),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='Simulate the RGB-D camera sensor (needs a display); false: body only.'),
        DeclareLaunchArgument(
            'depth_noise', default_value='true',
            description='Apply the depth error model; false: ideal depth, range limits only.'),
        DeclareLaunchArgument(
            'seed', default_value='1', description='Seed of the depth noise.'),
        DeclareLaunchArgument(
            'robot_xacro', default_value='openfactory_cell.urdf.xacro',
            description='Robot xacro in openfactory_description/urdf (takes include_fixtures).'),
        DeclareLaunchArgument(
            'fixtures_xacro', default_value='fixtures.urdf.xacro',
            description='Fixtures xacro in openfactory_description/urdf (takes camera_sensor).'),
        gazebo,
        bridge,
        depth_sensor_model,
        robot_state_publisher,
        _spawn(FIXTURES_NAME, fixtures),
        _spawn(ROBOT_NAME, robot_only),
        *spawners,
    ])
