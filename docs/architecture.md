# Architecture

Components, interfaces and frames of the single-arm cell as implemented through phase 5, and of the ring of N arms built from it in phase 6 ("Ring of N arms" below). The Rubik's cube cell of phase 7 is being built ("Rubik's cube cell" at the end): so far its cube model and the cube's logic. A component marked *later* is a proposal. Changing an implemented interface needs explicit approval. History is in `docs/decisions.md` and `docs/experiments.md`.

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

| Component | Implemented as | Input | Output |
|---|---|---|---|
| `scene_perception` | node `openfactory_perception/scene_perception` | RGB, depth, `camera_info`, TF | object hypotheses: class or `unknown`, 3D pose, confidence |
| `grasp_selector` | `openfactory_manipulation` library `grasp_selector.GraspSelector` | object hypotheses, robot state | ranked feasible grasp candidates |
| `task_planner` + `robot_executor` | `openfactory_manipulation` library `pick_place.PickPlace` | chosen grasp, place point | MoveIt requests, trajectories, gripper commands, cycle result |
| `scene_manager` | `openfactory_manipulation` library `bin_picking.BinPicking` | prior state, new view, cycle result | task state, next action, recovery |
| `factory_manager` | later | production goals, station states | task assignments |

The libraries keep their logic in ROS-free modules under `core/` and run inside `ros2 run openfactory_manipulation bin_picking` or the benchmark harness.

- `scene_perception` (pure logic in `openfactory_perception/core`, D-012): depth to points, bin crop and floor removal, top-view segmentation, model-based recognition of the catalog primitives. A cluster that matches no catalog shape (for example a tumbled object) is published as `unknown` with its footprint and measured height.
- `grasp_selector` (`core/grasps.py`, `core/ranking.py`; D-016, D-018, D-019): top-down candidates for catalog hypotheses and, with `selection.unknown_grasps`, for `unknown` clusters across the long and the short side of the footprint. Hard filters: gripper width, fingers against the bin walls and the other objects, then MoveIt (collision-free IK above the grasp, approach line, lift line); every rejection is counted by reason. The survivors are ordered by `ranking.strategy`: `learned` (default; logistic P(success) from pre-motion features minus a motion term, a JSON model in `openfactory_manipulation/config/` produced offline by `scripts/train_grasp_ranker.py`, which the cell only reads), `baseline` (D-016 score) or `uniform_random` (seeded per scene, for data collection). Every candidate's features, prior, score and `p_success` go into the attempt log.
- `PickPlace` (`core/pick_place.py`, D-010): pre-grasp by a free-space plan, every move near or with an object as a straight line, jaw-opening checks after closing and after lift and transfer, recovery to ready on failure. `pick_place_once` runs one cycle at a known pose (D-009), `pick_perceived_once` one cycle on the most confident hypothesis (D-013).
- `BinPicking` (`core/task.py`, D-017): one step is view of the input bin with the arm at ready → every object of the view into the planning scene (an `unknown` cluster as the box of its footprint and height) → grasp selection → `PickPlace.run_grasp` → next view, which verifies the pick (the object must be gone from the input bin) and feeds the next step. It keeps the task state of a scene: failed grasps that are not repeated, attempt budgets, the objects it placed and where. A jaw-opening check against a measured `unknown` width uses `selection.unknown_width_tolerance`. The loop ends with a reason (`bin_empty`, `no_feasible_grasp`, `only_unknown`, `output_full`, `attempt_budget`, `failure_budget`, `no_view`).

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

## Ring of N arms

Phase 6 (D-020, D-021). N UR5e arms and N bins on a round table: arm *i* picks from bin *i* and places into bin *(i + 1) mod N*, one arm at a time, for ever. It is a second system next to the single-arm cell, built from the same packages; the cell's launch, interfaces and frames above are unchanged.

```
ros2 launch openfactory_bringup ring.launch.py n:=3 rviz:=false
ros2 run openfactory_manipulation relay --arms 3
```

### Layout

Every arm sees its two bins exactly as the cell's arm sees `input_bin` and `output_bin`: 0.40 m in front of its base and 0.30 m to each side, the source on its right and the destination on its left. The two numbers are read from `cell.yaml` (`chord` = distance between the two bins = 0.60 m, `standoff` = base to the line that joins them = 0.40 m), so the ready configuration, the place points and the reach measured on the cell hold for every arm. Seen from above, with the ring centre at the world origin (`openfactory_description/ring.py`, pytest for N = 2 … 40):

- the bins are the vertices of a regular N-gon of side `chord`: `r_bin = chord / (2 sin(π/N))`; bin *k* is at angle −2πk/N (the bins advance clockwise) with its frame x along the ring;
- the arms stand outside it on the axis of each side, facing the centre: `r_arm = chord / (2 tan(π/N)) + standoff`;
- the table is a disc of radius `r_arm + table_margin` (`ring.yaml`), 0.75 m high as in the cell; one camera hangs 0.70 m above each bin floor, image axes along the bin axes.

N = 2 is the cell with a second arm facing the first across the two bins (bins at (±0.30, 0), arms at (0, ∓0.40)). Neighbouring bases are `chord cos(π/N) + 2 standoff sin(π/N)` apart: 0.80 m at N = 2, 0.99 m at N = 3 and 4, never less than `chord`. The geometry sets no upper limit on N; memory and CPU do (`docs/environment.md`).

### One model, files generated per N

One URDF (`ring.urdf.xacro`, robot `openfactory_ring`) holds every arm with the prefix `arm_<i>_`; one `move_group` plans each arm against all the others (planning group `arm_<i>_manipulator`), one `controller_manager` runs a trajectory and a gripper controller per arm. In Gazebo the arms are one model with `self_collide`, next to the fixtures model (`ring_fixtures.urdf.xacro`), so that arms stop each other physically too. xacro has no loops and the configuration files are per N, so a launch writes them to `/tmp/openfactory_ring/n<N>/` before starting anything; nothing there is a source:

| File | Written by | From |
|---|---|---|
| `layout.yaml` (poses and names of arms, bins, cameras, table) | `openfactory_description/ring.py` | `cell.yaml`, `ring.yaml` |
| `ros2_controllers.yaml`, `bridge.yaml` | `openfactory_simulation/ring_config.py` | the cell's `ros2_controllers.yaml` and `bridge.yaml` |
| `ring.srdf`, `kinematics.yaml`, `joint_limits.yaml`, `moveit_controllers.yaml` | `openfactory_moveit_config/ring_config.py` | the cell's SRDF and `moveit_controllers.yaml`, `ur_moveit_config` |

The xacro macros of the gripper, the bin and the camera take a prefix or a name and a pose; the cell URDF calls them with the old names. In the SRDF every arm keeps the collision matrix of the cell's arm; pairs of links of different arms stay enabled, except the fixed bases.

### Components

| Component | Implemented as | Notes |
|---|---|---|
| `scene_perception` | one node per bin, `/bin_<k>/scene_perception` (`ring_perception.launch.py`) | the cell's node with parameter `bin_frame: bin_<k>` and its topics remapped into `/bin_<k>` |
| depth sensor model | one node per camera, `/bin_<k>/depth_sensor_model` | noise seed `1000 × seed + k` |
| `grasp_selector`, `task_planner` + `robot_executor` | one `BinPicking` worker per arm inside `relay.RelayLoop`, all on one node | `MoveItInterface.for_arm` gives each worker its arm's names (`names.ring_arm`); TF, joint states and the planning-scene world are shared |
| `scene_manager` | `openfactory_manipulation` library `relay.RelayLoop`, rules in `core/relay.py` | turns, place points, verification, parking |

One turn (`RelayLoop.step`): with every arm parked, a view of every bin → the next arm round the ring whose source bin shows an object it may try and whose destination bin has a free place point → `BinPicking.step` for that arm (to ready, view of its source bin, planning scene, grasp selection, pick and place, view) → park → view of the destination bin.

- **Parking.** Arms that are not working stand upright above their base (`relay.park_joints`, the UR `up` state and the start state in Gazebo). Only the working arm goes to `ready_joints`: with N = 2 that configuration puts both tools at the same point, the ring centre. So MoveIt checks every plan of the working arm against arms that do not move, and no arm is in a camera view while views are taken.
- **Place points.** Every bin is observed, so nothing is remembered about the destination bin: a place point of `place.slots` is free when every object seen there is at least `relay.place_margin` from it (footprint circle to point), the roomiest first; the objects seen there enter the planning scene as obstacles (`dst_<k>`, grown by `place.obstacle_padding`) and MoveIt has the last word, as in the cell.
- **Order of the objects.** An arm takes first the object that has been in its source bin longest (`core/relay.waiting_since`: the bin remembers the turn in which each handed-over object arrived, objects never handed over are the oldest); the grasp ranking orders the grasps within that. Every object goes round instead of one going back and forth.
- **Verification.** A handover counts when the pick is verified as in the cell (finger checks, execution, object gone from the source bin) **and** the next view of the destination bin shows an object within `relay.arrival_radius` of the place point (`object_not_arrived` otherwise).
- **Failures.** A failed grasp is not repeated while its object is still there; the memory of a bin is pruned when the object is gone. The loop ends with a reason: `handovers_done`, `no_work` (no arm has an object and a free place point), `failure_budget` (`relay.max_consecutive_failures` in a row), `park_failed`, `no_view`.

### Interfaces and frames of the ring

Those of the cell with a prefix or a namespace; types and semantics are unchanged.

| Name | Cell equivalent | Notes |
|---|---|---|
| `/arm_<i>_joint_trajectory_controller/follow_joint_trajectory` | `/joint_trajectory_controller/follow_joint_trajectory` | one per arm; `move_group` picks the controller from the joints of the trajectory |
| `/arm_<i>_gripper_controller/gripper_cmd` | `/gripper_controller/gripper_cmd` | one per arm |
| `/joint_states`, `/robot_description`, `/move_action`, `/execute_trajectory`, `/compute_cartesian_path`, `/compute_ik`, `/apply_planning_scene`, `/clock` | the same | one of each for the whole ring; requests name the group `arm_<i>_manipulator` and the link `arm_<i>_tool0` |
| `/bin_<k>/camera/camera_info`, `/bin_<k>/camera/color/image_raw`, `/bin_<k>/camera/depth/image_ideal`, `/bin_<k>/camera/depth/image_raw` | `/camera/...` | one camera per bin; the ideal depth is still simulator data that perception must not read |
| `/bin_<k>/perception/objects`, `/bin_<k>/perception/debug/points`, `/bin_<k>/perception/debug/image` | `/perception/...` | hypotheses of bin *k*, frame `world` |

| Frame | Parent | Notes |
|---|---|---|
| `world` | — | floor, z up; origin below the ring centre |
| `table` | `world` | centre of the top surface of the round table |
| `arm_<i>_base_link` … `arm_<i>_tool0`, `arm_<i>_gripper_palm`, `arm_<i>_left_finger`, `arm_<i>_right_finger`, `arm_<i>_gripper_tcp` | `world` → arm chain | the frames of the cell's arm with the prefix; base x points at the ring centre |
| `bin_<k>` | `table` | centre of the inner floor of bin *k*, x along the ring |
| `bin_<k>_camera_link`, `bin_<k>_camera_optical_frame` | `bin_<k>` | as `camera_link` and `camera_optical_frame` of the cell, above bin *k* |

Ground truth is read as in the cell, by the harness only (`scripts/relay_benchmark.py`, judged in `scripts/relay_eval.py`): an object is where its pose falls inside the inner volume of a bin, on any face.

## Rubik's cube cell

Phase 7 (D-022, in progress). One arm, a nest that holds the bottom layer of the cube and one overhead camera are to solve a scrambled Rubik's cube whose layers turn only under the forces applied to it. A third system next to the cell and the ring, built from the same packages. Implemented so far: the cube model with its mechanism, the cube's logic, the mechanism test bench and the cell's layout (arm, nest, camera, MoveIt); the motions on the cube, the perception of the cube and the solve loop are not.

### Layout and frames

```
ros2 launch openfactory_bringup rubik.launch.py rviz:=false
```

The table, the UR5e, the gripper, the controllers and the MoveIt configuration are the cell's: `rubik_cell.urdf.xacro` has the same robot, joint and link names as `openfactory_cell.urdf.xacro`, so the interfaces of the cell above hold unchanged (`/move_action`, `/compute_cartesian_path`, `/compute_ik`, `/execute_trajectory`, `/apply_planning_scene`, the two controllers, `/camera/...`). In place of the bins there is a pedestal with a nest, and the camera hangs above the nest. Layout in `openfactory_description/config/rubik_cell.yaml`; in Gazebo the fixtures are a separate model (`rubik_fixtures.urdf.xacro`), for TF and MoveIt they are links of the robot URDF, as in the cell. `simulation.launch.py` and `move_group.launch.py` take the xacro and SRDF file names as arguments whose defaults are the cell's; `rubik.launch.py` passes its own.

| Frame | Parent | Notes |
|---|---|---|
| `pedestal` | `table` | centre of the pedestal's footprint on the table top, 0.50 m in front of the robot base |
| `nest` | `pedestal` | centre of the nest's inner floor, 0.08 m above the table top (world (0.50, 0, 0.83)), axes parallel to the table. A square seat 58.5 mm wide with walls 12 mm high: it holds the bottom layer of the cube (19 mm) with 0.75 mm of play per side |
| `camera_link`, `camera_optical_frame` | `nest` | the cell's camera 0.70 m above the nest floor, looking straight down, image right = world +x |

The pedestal raises the nest so that the gripper can set the cube down with its tool horizontal: the fingers then pass 6.5 mm above the nest walls and the palm hangs beside the pedestal, 78 mm above the table. `openfactory_manipulation/core/rubik/station.py` holds the geometry of the grasps (ROS-free): the tool axis through the cube centre, the fingertips 2 mm above the top of the bottom layer in the grasp from above, so that the pads hold the two upper layers only.

### The cube model

A 57 mm cube of 92 g, generated as SDF by `scripts/rubik_model.py` from `openfactory_description/config/rubik.yaml` and from a cube state, so that a cube can be spawned already scrambled. Model frame = cube frame: x towards the R face, y towards U, z towards F, origin at the centre.

| Link | What |
|---|---|
| `core` | the root, at the cube centre |
| `axle_<U\|R\|F\|D\|L\|B>` | six links on revolute joints `axle_<face>_joint` of the core, about the outward face normals, with the layer friction and damping of `rubik.yaml` |
| `cubie_<xyz>` | 26 free links named after their home slot, each letter `n`, `z` or `p` for −1, 0, +1 along x, y, z (`cubie_ppp` is the U-R-F corner): a 19 mm collision box with the friction of `rubik.yaml`, a black body and one coloured sticker per outer face |

The cubies do not collide with each other (one model, no `self_collide`); they collide with everything else.

### The mechanism (D-023)

`RubikCubeSystem` (`openfactory_simulation/src/rubik_cube_system.cc`), a Gazebo system plugin of the model, joins the links at run time:

- One axis of the cube is active. The nine cubies of each of its two outer layers are welded to that layer's axle, the eight between them to the core. A weld is a `DetachableJoint` entity: in the physics engine a fixed joint of the model tree.
- The two active axles turn under whatever torque reaches them through the cubies, against the joint friction and a detent the plugin applies as a joint torque: a spring towards the nearest quarter turn, capped, within a window. The other four axles carry nothing.
- From the wrench each weld transmits the plugin computes the torque the outside applies to every layer of the two other axes. When one exceeds `breakaway_torque` and both active layers are within `snap_tolerance` of a quarter turn, their angles are set to exactly the quarter turn (`JointPositionReset`), the welds are removed after that step and created for the new axis before the next one. If a layer moved in between, the old welds are put back. While an active layer is out of square the other axes stay rigid.
- Nothing moves a cubie except forces on the links. Idealised: no interlocking pieces inside (a cubie cannot be pulled out, a corner cannot be twisted), and the exact squaring (at most 0.1°) before an axis change.

The plugin publishes its state on the Gazebo topic `/model/rubik_cube/mechanism` (`gz.msgs.StringMsg`, JSON: active axis, axle angles, torque demand per axis, axis changes, `blocked`), 50 times per simulated second. It is simulator data: not bridged to ROS, read by the harness only.

### Cube logic

ROS-free, in `openfactory_manipulation/core/rubik/`, with pytest:

| Module | What |
|---|---|
| `cube.py` | the cube as 54 facelets in Kociemba's order (`URFDLB`), moves, validity and solvability of a state, seeded scrambles (ported from the `doppelbot` project) |
| `cubies.py`, `rotations.py` | the same cube as 26 cubies, each in a slot with one of the 24 rotations; conversion both ways |
| `orientation.py` | how the cube sits in the nest (one of 24 orientations) and what the cell's turn and flip do to state and orientation |
| `solver.py` | `solve(state)` through the `kociemba` library, imported only there |

### Ground truth

As in the cell, only the harness reads the simulator: `scripts/rubik_truth.py` turns the poses of the cube's links (`/world/<world>/dynamic_pose/info`) into the 54-facelet state, with the largest distance and angle of a cubie from the grid. `scripts/rubik_mechanism_test.py` drives the test bench (`ros2 launch openfactory_simulation rubik_bench.launch.py`, world `rubik_bench`: physics only, a cube whose core is fixed to the world, faces turned by torques on their centre cubies through Gazebo's `ApplyLinkWrench` system). That drive exists on the bench only.
