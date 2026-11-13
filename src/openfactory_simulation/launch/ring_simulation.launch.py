"""Gazebo ring of N arms (phase 6, D-020): world, fixtures (round table, one bin and one RGB-D
camera per arm), N UR5e in one model with gz_ros2_control, bridge, controllers.

    ros2 launch openfactory_simulation ring_simulation.launch.py n:=3

The layout, the controllers and the bridge mappings of one N are generated at launch
(openfactory_description/ring.py, openfactory_simulation/ring_config.py). With camera:=true
(default) Gazebo renders the cameras with Ogre 1, which needs the VM display even when
headless:=true (D-005).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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

from openfactory_description import ring
from openfactory_simulation import ring_config

ROBOT_NAME = 'ring'
FIXTURES_NAME = 'ring_fixtures'
# topics of scripts/depth_sensor_model.py, moved into the namespace of each bin
DEPTH_MODEL_TOPICS = ('/camera/depth/image_ideal', '/camera/depth/image_raw',
                      '/camera/camera_info')


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


def _ring(context):
    n = int(LaunchConfiguration('n').perform(context))
    seed = int(LaunchConfiguration('seed').perform(context))
    layout_file = str(ring.write_layout(n))
    controllers_file, bridge_file = (str(path) for path in ring_config.write(n))

    # Same xacro and arguments for robot_state_publisher (which gz_ros2_control reads) and the
    # spawned arms; only the fixtures are split off into their own Gazebo model.
    arms_with_fixtures = _xacro(
        'ring.urdf.xacro', layout_file=layout_file, include_fixtures='true',
        simulation_controllers=controllers_file)
    arms_only = _xacro(
        'ring.urdf.xacro', layout_file=layout_file, include_fixtures='false',
        simulation_controllers=controllers_file)
    fixtures = _xacro('ring_fixtures.urdf.xacro', layout_file=layout_file,
                      camera_sensor=LaunchConfiguration('camera'))

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_file}],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(arms_with_fixtures, value_type=str),
            'use_sim_time': True,
        }],
        output='screen',
    )

    # one spawner for every controller: 2 N + 1 processes would each load rclpy
    spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[*ring_config.controller_names(n), '--controller-manager',
                   '/controller_manager'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # D-011: ideal Gazebo depth -> depth with the camera's error model, one node per camera.
    # Each camera has its own noise stream.
    depth_models = [
        Node(
            package='openfactory_simulation',
            executable='depth_sensor_model',
            namespace=ring.bin_namespace(index),
            remappings=[(topic, ring.bin_topic(index, topic)) for topic in DEPTH_MODEL_TOPICS],
            parameters=[{
                'noise': ParameterValue(LaunchConfiguration('depth_noise'), value_type=bool),
                'seed': seed * 1000 + index,
                'use_sim_time': True,
            }],
            condition=IfCondition(LaunchConfiguration('camera')),
            output='screen',
        )
        for index in range(n)
    ]

    return [bridge, robot_state_publisher, _spawn(FIXTURES_NAME, fixtures),
            _spawn(ROBOT_NAME, arms_only), spawner, *depth_models]


def generate_launch_description():
    world = LaunchConfiguration('world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            # D-005: Ogre 1 for the GUI; the server engine is set in the world's Sensors system
            'gz_args': IfElseSubstitution(
                LaunchConfiguration('headless'),
                if_value=['-s -r -v 3 ', world],
                else_value=['-r -v 3 --render-engine ogre ', world]),
            'on_exit_shutdown': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('n', default_value='2', description='Number of arms (>= 2).'),
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
            description='Simulate the RGB-D camera sensors (need a display); false: bodies only.'),
        DeclareLaunchArgument(
            'depth_noise', default_value='true',
            description='Apply the depth error model; false: ideal depth, range limits only.'),
        DeclareLaunchArgument(
            'seed', default_value='1', description='Seed of the depth noise.'),
        gazebo,
        OpaqueFunction(function=_ring),
    ])
