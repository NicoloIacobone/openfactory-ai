# Architecture

Components, interfaces and frames. Everything here is **proposed** until the phase that implements it closes; changing an implemented interface needs explicit approval.

## System loop

```
simulated RGB-D camera + robot state
  → scene_perception     RGB/depth → objects, segmentation, 3D poses, confidence
  → grasp_selector       geometry + robot state → ranked feasible grasps
  → task_planner         MoveIt 2: IK, collisions, trajectory
  → robot_executor       controllers + gripper: pick and place
  → scene_manager        verify, update scene/task state, recover or continue
  → back to perception
```

Perception never commands joints. Simulator ground truth (`/ground_truth/...`) is consumed only by evaluation and debugging tools.

## Components

| Component | Package | Input | Output |
|---|---|---|---|
| `scene_perception` | `openfactory_perception` | RGB, depth, `camera_info`, TF | objects, segmentation, 3D poses, confidence |
| `grasp_selector` | `openfactory_manipulation` | object poses/geometry, point cloud, robot state | ranked feasible grasp candidates |
| `task_planner` | `openfactory_manipulation` | best grasp, destination bin, task state | MoveIt requests, next action |
| `robot_executor` | `openfactory_manipulation` | trajectories, gripper commands | execution result, state |
| `scene_manager` | `openfactory_manipulation` | prior state, perception update, action result | updated scene/task state, recovery request |
| `factory_manager` | later | production goals, station states | task assignments (not in the MVP) |

## Interfaces

To be defined in the phase that introduces each component. Standard types first (`sensor_msgs`, `geometry_msgs`, `vision_msgs`, `moveit_msgs`, `control_msgs`).

| Name | Type | Producer → consumer | Notes |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | Gazebo (`ros_gz_bridge`) → all nodes | `use_sim_time: true` everywhere (phase 1) |
| `/robot_description` | `std_msgs/String` (latched) | `robot_state_publisher` → `gz_ros2_control`, RViz | cell URDF with the table (phase 1) |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` → `robot_state_publisher`, `move_group` | phase 1 |
| `/joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | `move_group` → `joint_trajectory_controller` | only path for arm motion (phase 1) |
| `/move_action` | `moveit_msgs/action/MoveGroup` | clients → `move_group` | plan and execute (phase 1) |
| `/gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | manipulation → `gripper_controller` | `position` = left finger joint = half the jaw opening (m); stalling on an object succeeds (phase 2) |

## Frames

Arm and cell frames fixed in phase 1, gripper and floor in phase 2; camera frames in phase 3. Conventions: REP 103 / REP 105, SI units, radians; camera data in `*_optical_frame`.

| Frame | Parent | Defined by | Notes |
|---|---|---|---|
| `world` | — | `openfactory_description` | cell floor, z up; origin below the robot base |
| `table` | `world` | `openfactory_description/config/cell.yaml` | centre of the table top surface |
| `base_link` | `world` | `cell.yaml` (`robot_mount`) | UR5e base on the table top, REP 103 |
| `base_link_inertia` … `wrist_3_link`, `flange`, `tool0` | arm chain | `ur_description` | `tool0` is the MoveIt group tip |
| `floor` | `world` | `openfactory_description` | collision box for MoveIt only, top face at z = 0 (phase 2) |
| `input_bin`, `output_bin` | `table` | `cell.yaml` (`bins`, `bin_wall`) | centre of the bin's inner floor surface, axes parallel to the table (phase 2) |
| `gripper_palm`, `left_finger`, `right_finger` | `tool0` | `openfactory_description/config/gripper.yaml` | fingers slide along `tool0` y (phase 2) |
| `gripper_tcp` | `tool0` | `gripper.yaml` (`tcp_offset`) | grasp point between the pads, +z along the tool axis (phase 2) |

Cell geometry (table size and pose, robot mount, bins) is defined once in `cell.yaml`. In Gazebo the table and the bins form one separate model fixed to the world (`fixtures.urdf.xacro`) so that they collide with the arm; for `robot_state_publisher` and MoveIt they are links of the cell URDF (`include_fixtures:=true`, together with the `floor` collision box), so they are in TF and in collision checking without extra nodes. The gripper is part of the robot in every URDF variant.
