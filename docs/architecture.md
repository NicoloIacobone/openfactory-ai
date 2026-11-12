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

Perception never commands joints. Simulator ground truth is consumed only by evaluation and debugging tools (see "Ground truth and scene control").

## Components

| Component | Package | Input | Output |
|---|---|---|---|
| `scene_perception` | `openfactory_perception` | RGB, depth, `camera_info`, TF | objects, segmentation, 3D poses, confidence |
| `grasp_selector` | `openfactory_manipulation` | object poses/geometry, point cloud, robot state | ranked feasible grasp candidates |
| `task_planner` | `openfactory_manipulation` | best grasp, destination bin, task state | MoveIt requests, next action |
| `robot_executor` | `openfactory_manipulation` | trajectories, gripper commands | execution result, state |
| `scene_manager` | `openfactory_manipulation` | prior state, perception update, action result | updated scene/task state, recovery request |
| `factory_manager` | later | production goals, station states | task assignments (not in the MVP) |

Phase 2 implements `task_planner` and `robot_executor` as a library (`openfactory_manipulation.pick_place.PickPlace`, pure logic in `core/`) driven by the trial harness or `ros2 run openfactory_manipulation pick_place_once`; the object pose is the known-pose input of D-009, not a perception output.

Phase 3 implements `scene_perception` as the node `openfactory_perception/scene_perception` (pure logic in `core/`: depth to points, bin crop and floor removal, top-view segmentation, model-based recognition of the catalog primitives, D-012). `grasp_selector` is not a component yet: `openfactory_manipulation.camera_pick` takes the most confident hypothesis and passes its type and pose to the same `PickPlace` cycle (`ros2 run openfactory_manipulation pick_perceived_once`, D-013); grasp candidates and feasibility ranking are phase 4.

Phase 4 implements the other two as libraries of `openfactory_manipulation`, run by `ros2 run openfactory_manipulation bin_picking` or by the benchmark harness:

- `grasp_selector` is `grasp_selector.GraspSelector` (pure logic in `core/grasps.py`, D-016): top-down candidates per hypothesis, hard filters (gripper width, fingers against the bin walls and the other objects, then MoveIt: collision-free IK above the grasp, approach line, lift line), and a baseline score on the survivors only. Every rejection is counted by reason.
- `scene_manager` is `bin_picking.BinPicking` (pure logic in `core/task.py`, D-017). One step: view of the input bin with the arm at ready → every object of the view into the planning scene → grasp selection → `PickPlace.run_grasp` → next view, which verifies the pick (the object must be gone from the input bin) and feeds the next step. It keeps the task state of a scene: failed grasps that are not repeated, attempt budgets, the objects it placed and where. The loop ends with a reason (`bin_empty`, `no_feasible_grasp`, `only_unknown`, `output_full`, `attempt_budget`, `failure_budget`, `no_view`).

Phase 5 extends both without new interfaces:

- `grasp_selector` may also try `unknown` clusters (`selection.unknown_grasps`, D-018): top-down candidates across the long and the short side of the measured footprint, through the same hard filters; in the planning scene the cluster is the box of its footprint and height. A jaw-opening check against such a measured width uses `selection.unknown_width_tolerance`.
- The survivors are ordered by a ranking strategy (`openfactory_manipulation/core/ranking.py`, `ranking.strategy`, D-019): `baseline` (D-016 score), `uniform_random` (seeded per scene, for data collection) or `learned` (logistic P(success) from pre-motion features, minus a motion term). The `learned` model is a JSON file in `openfactory_manipulation/config/` produced offline by `scripts/train_grasp_ranker.py` from training logs; the cell only reads it. Every candidate's features, prior, score and `p_success` go into the attempt log.

The output bin is not observed by the camera: the cell places on a fixed grid of points in `output_bin` and remembers what it put where (planning-scene obstacles), so the object's final pose there is known only to the evaluation harness. If no place point is feasible the held object is set down where it was picked. The planning-scene world holds objects only and belongs to this loop, which empties it at every new scene.

## Interfaces

To be defined in the phase that introduces each component. Standard types first (`sensor_msgs`, `geometry_msgs`, `vision_msgs`, `moveit_msgs`, `control_msgs`).

| Name | Type | Producer → consumer | Notes |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | Gazebo (`ros_gz_bridge`) → all nodes | `use_sim_time: true` everywhere (phase 1) |
| `/robot_description` | `std_msgs/String` (latched) | `robot_state_publisher` → `gz_ros2_control`, RViz | cell URDF with the table (phase 1) |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` → `robot_state_publisher`, `move_group` | phase 1 |
| `/joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | `move_group` → `joint_trajectory_controller` | only path for arm motion (phase 1) |
| `/move_action` | `moveit_msgs/action/MoveGroup` | clients → `move_group` | plan and execute (phase 1) |
| `/move_action` (`plan_only`) | `moveit_msgs/action/MoveGroup` | `openfactory_manipulation` → `move_group` | free-space plans to joint or `tool0` pose goals; executed separately (phase 2) |
| `/compute_cartesian_path` | `moveit_msgs/srv/GetCartesianPath` | `openfactory_manipulation` → `move_group` | straight `tool0` lines: approach, lift, transfer, descend, retreat (phase 2); from a given start state when `grasp_selector` checks a candidate's approach and lift, and when a place point is checked before the transfer (phase 4) |
| `/execute_trajectory` | `moveit_msgs/action/ExecuteTrajectory` | `openfactory_manipulation` → `move_group` | executes planned trajectories through `joint_trajectory_controller` (phase 2) |
| `/apply_planning_scene` | `moveit_msgs/srv/ApplyPlanningScene` | `openfactory_manipulation` → `move_group` | object added at its known pose, attached to `gripper_tcp` on grasp, removed on release (phase 2). Bin picking: every object of the view as `in_<k>` (catalog shape, or a box for `unknown` clusters) and every object the cell placed as `out_<slot>` (phase 4) |
| `/compute_ik` | `moveit_msgs/srv/GetPositionIK` | `openfactory_manipulation` (`grasp_selector`) → `move_group` | collision-free arm configuration above a grasp candidate, with the fingers at the approach opening; a hard filter and the goal of the move to the pre-grasp (phase 4) |
| `/gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | manipulation → `gripper_controller` | `position` = left finger joint = half the jaw opening (m); stalling on an object succeeds (phase 2) |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo (`ros_gz_bridge`) → `depth_sensor_model`, `scene_perception` | intrinsics of colour and depth (one optical centre), frame `camera_optical_frame` (phase 3) |
| `/camera/color/image_raw` | `sensor_msgs/Image` `rgb8` | Gazebo (`ros_gz_bridge`, lazy) → `scene_perception` debug image | not used for recognition (phase 3) |
| `/camera/depth/image_ideal` | `sensor_msgs/Image` `32FC1` | Gazebo (`ros_gz_bridge`, lazy) → `depth_sensor_model`, harness | simulator's ideal z-depth; perception must not subscribe to it (phase 3) |
| `/camera/depth/image_raw` | `sensor_msgs/Image` `32FC1` | `depth_sensor_model` → `scene_perception` | measured z-depth in metres with the camera's error model, NaN = invalid (REP 118), same stamp and frame as the ideal image (phase 3) |
| `/perception/objects` | `vision_msgs/Detection3DArray` | `scene_perception` → `openfactory_manipulation` | one message per depth frame, stamped with the image stamp, frame `world`; empty when the bin is empty. Per detection: `results[0].hypothesis.class_id` (catalog name or `unknown`), `.score` in (0, 1], `results[0].pose` = pose of the catalog object frame, `bbox` = catalog bounding box at that pose (phase 3) |
| `/perception/debug/points` | `sensor_msgs/PointCloud2` (x, y, z, label) | `scene_perception` → RViz | object points in `input_bin`, published while subscribed (phase 3) |
| `/perception/debug/image` | `sensor_msgs/Image` `rgb8` | `scene_perception` → RViz | colour image with the detections drawn, published while subscribed (phase 3) |

## Frames

Arm and cell frames fixed in phase 1, gripper and floor in phase 2, camera frames in phase 3. Conventions: REP 103 / REP 105, SI units, radians; camera data in `*_optical_frame`.

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
| `camera_link` | `table` | `cell.yaml` (`camera_mount`) | optical centre on the front face of the camera body, REP 103: x along the view, y left and z up in the image; 0.70 m above the centre of the `input_bin` floor, looking straight down (phase 3) |
| `camera_optical_frame` | `camera_link` | `openfactory_description/urdf/camera.xacro` | same point, optical convention: z along the view, x right, y down in the image; frame of every image and of `camera_info`. In the cell: x = world +x, y = world −y, z = world −z (phase 3) |

Cell geometry (table size and pose, robot mount, bins, camera mount) is defined once in `cell.yaml`. In Gazebo the table, the bins and the camera form one separate model fixed to the world (`fixtures.urdf.xacro`) so that they collide with the arm; for `robot_state_publisher` and MoveIt they are links of the cell URDF (`include_fixtures:=true`, together with the `floor` collision box), so they are in TF and in collision checking without extra nodes. The gripper is part of the robot in every URDF variant.

## Camera and calibration

One fixed RGB-D camera above the input bin (D-011), modelled on the depth stream of the Intel RealSense D415; sensor parameters in `openfactory_description/config/camera.yaml`, mount pose in `cell.yaml`. The Gazebo `rgbd_camera` sensor exists only in the fixtures model (`camera_sensor:=true`); launch argument `camera:=false` leaves the camera body without the sensor, for physics-only runs without a display (D-005).

| Item | Value | Source |
|---|---|---|
| Image | 640 × 360, 5 Hz nominal | `camera.yaml` |
| Intrinsics | fx = fy = 502.30 px (HFOV 65°), cx = 320, cy = 180, no distortion | `/camera/camera_info`; never hard-coded |
| Principal-point correction | (−0.5, −0.5) px added to (cx, cy) | `openfactory_perception/config/perception.yaml`, measured in E-003 part A: Gazebo's pixel (u, v) looks along the ray of (u + 0.5, v + 0.5) |
| Extrinsics | TF `world → table → camera_link → camera_optical_frame` | `cell.yaml`, published by `robot_state_publisher` on `/tf_static` |
| Depth | z-depth (distance along the optical axis, not along the ray), metres, `32FC1`; invalid = NaN; valid range 0.24–2.0 m | `camera.yaml` `depth`, applied by `depth_sensor_model` |
| Depth error model | Gaussian disparity noise 0.08 px (σ_z = z² σ_d / (f B), 1.4 mm at 0.7 m), 1 mm quantization, seeded per frame from (seed, image stamp) | `openfactory_simulation/depth_noise.py`; `depth_noise:=false` or `ros2 param set /depth_sensor_model noise false` gives ideal depth |

Calibration is verified, not assumed: `scripts/phase3_camera_check.py` compares `camera_info` with `camera.yaml`, fits the floor of the empty bin seen by the camera against the plane z = 0 of `input_bin`, and projects a known point (top-face centre of a cube at a ground-truth pose) into the image through TF and K (E-003 part A).

Perception sees whatever is inside the input-bin volume below 0.12 m, the arm included; consumers take a view while the arm is at the ready configuration, clear of the bin.

## Objects

Rigid primitives (cube, cylinder, triangular prism) are defined once in `openfactory_description/config/objects.yaml` (shape, size, mass, colour; object frame at the bounding-box centre, z up when resting). The Gazebo models are SDF generated from it by `scripts/scene_objects.py`; manipulation reads the same catalog for object geometry.

## Ground truth and scene control

The trial harness talks to Gazebo directly through Gazebo Transport (`scripts/gz_scene.py`), outside the ROS graph:

| Gazebo interface | Use | Who |
|---|---|---|
| `/world/cell/create/blocking`, `/world/cell/remove/blocking` (`gz.msgs.EntityFactory` / `Entity` → `Boolean`) | spawn and remove objects between trials | harness only |
| `/world/cell/dynamic_pose/info` (`gz.msgs.Pose_V`, model poses in `world`) | ground truth for success evaluation and debug labels | harness evaluation only |

Ground truth is not bridged to ROS (D-009): no ROS node can subscribe to it, so perception and manipulation cannot leak it. If a ROS ground-truth topic is ever needed for debugging, it lives under `/ground_truth/`. The ideal depth image (`/camera/depth/image_ideal`) is simulator data too: only the depth sensor model and the harness's calibration check read it. A pytest in `openfactory_perception` and one in `openfactory_manipulation` fail if a file of the package names ground truth, Gazebo transport, the bridge or (perception) the ideal depth topic.

Scene randomization (`scripts/randomize_scene.py`) is seeded per scene from (seed, scene index); scenes hold separate objects resting in their catalog orientation, one to three in phase 3, five close to each other and to the walls in phase 4 (`scripts/config/phase4_bin_picking.yaml`); in phase 5 a cylinder may lie on its side and a prism stand on an end face (`tumbled_share`, `scripts/config/phase5_tumbled.yaml`, rest states in `scripts/scene_objects.py`). The benchmark harness (`scripts/benchmark.py`) reads ground truth after every attempt of the cell and judges it in `scripts/bin_picking_eval.py`; the cell's own verdict comes only from its fingers and its camera.
