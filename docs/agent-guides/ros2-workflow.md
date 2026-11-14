# ROS 2 workflow

Applies when a task creates or changes packages, nodes, launch files, parameters, TF, or ROS tests.

## Packages

- One package per responsibility, under `src/`, named `openfactory_<role>`:

  | Package | Holds |
  |---|---|
  | `openfactory_description` | URDF/Xacro, meshes references, static robot config; Python module `ring` (layout and names of the ring of N arms) |
  | `openfactory_simulation` | worlds, object models, sensors, randomization, simulation launch; the Gazebo system plugin of the Rubik's cube (C++, built against `gz_sim_vendor` and `gz_plugin_vendor`) |
  | `openfactory_moveit_config` | planning groups, kinematics, controllers, collision config |
  | `openfactory_perception` | RGB-D processing, point clouds, segmentation, pose estimation |
  | `openfactory_manipulation` | grasp generation/scoring, task orchestration, execution |
  | `openfactory_bringup` | whole-system launch files |
  | `openfactory_interfaces` | only if a stable custom interface is approved |

- Create a package only when its first real file exists. No empty skeletons.
- Python nodes: `ament_python`. Use `ament_cmake` only where needed (interfaces, C++ for performance, MoveIt config generated as CMake).
- Dependencies go in `package.xml` (`depend`, `exec_depend`, `test_depend`) and must be real, installed packages. Check with `apt-cache policy ros-jazzy-<name>` in the VM before adding.
- Third-party robot models: reuse the vendor's description package from apt when available instead of copying URDF/meshes into the repo.

## Nodes

- Thin nodes: subscribe, call pure functions, publish. Logic lives in ROS-free modules (`<pkg>/<pkg>/core/` or similar) with pytest tests.
- rclpy pitfall: never name node attributes like `Node` internals (`_publishers`, `_subscriptions`, `_timers`, `_clients`, `_services`); `destroy_node` breaks on shutdown.
- Sensor topics use `qos_profile_sensor_data`; latched state topics use `transient_local` durability.
- Names (topics, frames, joints) as module constants or parameters; one constants module per package that others import.

## Parameters and launch

- Parameters in `config/*.yaml`, loaded by launch. No hard-coded tunables in node code.
- `use_sim_time: true` on every node when the simulator runs; the `/clock` bridge must exist before nodes that use time.
- Each package exposes its own launch files; `openfactory_bringup` includes them. Launch arguments for anything that varies between runs (seed, world, headless, rviz).
- Long-running launches run detached in the VM with their output in `~/<name>.log` (`docs/environment.md`, "Operating the VM over SSH").
- A launch whose nodes depend on a number (the N arms of the ring) resolves it in an `OpaqueFunction` and generates the per-N files there (`ring_simulation.launch.py`, `ring_move_group.launch.py`).

## TF and frames

- REP 103: SI units, radians, right-handed, x forward / y left / z up for bodies; optical frames z forward / x right / y down.
- REP 105 style tree: `world` → robot base → links; `world` → camera mount → `*_optical_frame`.
- Every pose and point cloud carries `frame_id` and stamp; transform with `tf2` at the stamp of the data, never assume frames coincide.
- New or renamed frames are recorded in `docs/architecture.md`, "Frames".
- Check the tree with `ros2 run tf2_tools view_frames` and `ros2 run tf2_ros tf2_echo <a> <b>`.

## Build and test

Standard colcon workflow in the VM; exact commands in `CLAUDE.md`, "Build and test". After a change:

1. Build only what changed: `colcon build --symlink-install --base-paths /mnt/openfactory-ai/src --packages-up-to <pkg>`.
2. Unit tests (pytest) for pure logic; `launch_testing` for node/launch integration when the behavior depends on ROS wiring. Check exit codes: a node crashing on shutdown is a failure.
3. Run the tests in the VM and report the result.
