# Decisions

One entry per decision: context, decision, evidence, status. Newest last. Revisit an entry when its evidence changes.

## D-001 — Keep ROS 2 Jazzy on the existing VM

- **Context:** the project brief suggests Jazzy on Ubuntu 24.04 but allows any working distro.
- **Decision:** use the installed Ubuntu 24.04 + ROS 2 Jazzy; no reinstall, upgrade or migration.
- **Evidence:** audit 2026-10-28 (`docs/environment.md`): `ros2 doctor` passes, pub/sub works, Gazebo Harmonic, MoveIt 2 and UR packages are available as aarch64 binaries.
- **Status:** accepted.

## D-002 — English for repository docs

- **Context:** portfolio project for job applications.
- **Decision:** README, docs, code and commits in English; the assistant talks to the user in Italian.
- **Status:** accepted.

## D-003 — Repo reaches the VM through one shared `ros_2_projects` folder

- **Context:** the VM already shared only `doppelbot`; UTM allows a single shared folder, and sharing all of `~/Desktop/nico` would expose unrelated personal files.
- **Decision:** both projects move into `~/Desktop/nico/ros_2_projects/`, shared as `/mnt/ros_2_projects`; `/mnt/openfactory-ai` and `/mnt/doppelbot` are symlinks into it, so existing VM paths keep working. Colcon workspace outside the share in `~/openfactory_ws`.
- **Status:** accepted, verified 2026-10-28.
- **Update 2026-11-10:** on the Mac, `ros_2_projects` moved as a whole into `~/Desktop/nico/github_portfolio/` (one folder for all projects). The UTM share must point to `~/Desktop/nico/github_portfolio/ros_2_projects`; VM paths (`/mnt/ros_2_projects`, `/mnt/openfactory-ai`, `/mnt/doppelbot`) are unchanged.

## D-004 — Robot arm

- **Context:** the brief suggests a mature arm such as UR5e or Franka Panda.
- **Decision:** UR5e, from the Jazzy apt packages (`ur_description`, `ur_simulation_gz`, `ur_moveit_config`); Franka has no Jazzy apt package here.
- **Evidence:** smoke test 2026-10-29 (`docs/environment.md`): MoveIt 2 plans and executes through `scaled_joint_trajectory_controller` in Gazebo; full stack uses 1.46 GiB of 3.8 GiB.
- **Status:** accepted. Gripper: D-007.

## D-005 — Gazebo Harmonic with the Ogre 1 render engine, sensors on the VM display

- **Context:** the VM GPU is virtio (`virgl`) and exposes only OpenGL 2.1, no core profile. Gazebo Harmonic renders cameras with `ogre2` by default, which needs OpenGL 3.3+.
- **Decision:** Gazebo Harmonic (Sim 8.15, `ros-jazzy-ros-gz`) with `render_engine` set to `ogre` in the Sensors system and `--render-engine ogre` for the GUI. Any simulation with camera sensors runs with `DISPLAY` set to the VM graphical session (Xwayland `:0`); `--headless-rendering` is not usable. Physics-only runs can stay fully headless (`gz sim -s`).
- **Evidence:** probe 2026-10-28 (`docs/environment.md`, "Rendering probe"): `ogre2` segfaults with EGL headless, with EGL + `LIBGL_ALWAYS_SOFTWARE=1` and through Xwayland (Ogre-Next always selects the EGL hardware device, so llvmpipe's OpenGL 4.5 is never used); `ogre` through Xwayland publishes RGB and depth with correct values.
- **Consequences:** Ogre 1 is the older engine (simpler lighting, fewer features); third-party worlds that hardcode `ogre2` (e.g. the UR Gazebo launch, phase 1) must be overridden. Revisit if the VM gets a GPU with OpenGL 3.3+.
- **Status:** accepted for sensors and for the GUI (`gz sim --render-engine ogre` on the VM desktop, verified 2026-10-29).

## D-006 — Cell table from one layout file; standard joint trajectory controller

- **Context:** phase 1 needs the table in Gazebo (physics), TF and the MoveIt collision world. Links of one Gazebo model do not collide with each other, so a table inside the robot model would not stop the arm physically. The vendor simulation uses `ur_controllers/ScaledJointTrajectoryController`, which only makes sense with the real robot's speed scaling.
- **Decision:** table size/pose and robot mount live in `openfactory_description/config/cell.yaml`. The same xacro produces the robot + table URDF for `robot_state_publisher` and MoveIt, and the robot-only URDF spawned in Gazebo next to a separate static table model built from the same macro. A pytest checks both against `cell.yaml`. Arm motion uses the standard `joint_trajectory_controller`; MoveIt loads only the OMPL pipeline.
- **Consequences:** MoveIt sees the table as a robot link (allowed collisions with `base_link_inertia` and `shoulder_link` in the SRDF), not as a planning-scene collision object. Bins and camera mount (phases 2–3) are revisited when a scene node exists. The URDF given to MoveIt differs from the RSP one only in the Gazebo controllers path.
- **Evidence:** 2026-10-29 cell launch (`docs/environment.md`): TF matches `cell.yaml`, MoveIt rejects arm–table contact, RViz Plan & Execute runs through `joint_trajectory_controller`.
- **Status:** accepted.

## D-007 — Simple parallel-jaw gripper, friction grasp

- **Context:** phase 2 needs a gripper for cubes, cylinders and prisms of about 5 cm. `ros-jazzy-robotiq-description` exists in apt, but the 2F-85 needs an install, has a mimic linkage, and friction grasps with it in Gazebo are known to be fragile; the VM has 3.8 GB RAM.
- **Decision:** a project-defined gripper in `openfactory_description` (`urdf/gripper.xacro`, sizes in `config/gripper.yaml`): box palm on `tool0`, two prismatic box fingers along `tool0` y, opening 0–0.085 m, `right_finger_joint` mimics `left_finger_joint`, TCP frame `gripper_tcp`. Driven by `position_controllers/GripperActionController` (`gripper_controllers`, already installed) in its own `ros2_control` block of `gz_ros2_control`. Objects are held by friction only: no attach plugin, no `DetachableJoint`. If friction grasps prove unreliable, friction, effort, speed and object mass are tuned first; any attach mechanism needs the user's approval and a new entry (it bends rule 1).
- **Evidence:** VM check 2026-11-10: `gripper_controllers` 4.42.1 registers `position_controllers/` and `effort_controllers/GripperActionController`; `gz_ros2_control` 1.2.20 loads every `ros2_control` block and drives URDF mimic joints listed without command interfaces (upstream `component_parser.cpp`, `gz_system.cpp`). Position commands become joint velocity commands in Gazebo.
- **Gazebo joint limits (2026-11-10):** a Gazebo joint driven onto a position limit or onto its velocity limit stops following `gz_ros2_control` commands (cf. gz_ros2_control#165/#192). First runs: with the initial state on the stroke limit the fingers never moved; with a 0.1 m/s velocity limit a finger moving in the direction of gravity ran on to its end stop and stuck. Fix: joint limits are the commanded stroke ± `limit_margin` (5 mm), and the velocity limit (10 m/s) stays above every command `gz_ros2_control` issues (finger 0.1 × 500 Hz × error, mimic finger 500 Hz × lag). Finger speed is then set by the plugin's position gain, not by the URDF limit. Check: close, open, 0.02, close, open, all `reached_goal`, both fingers within 0.2 mm of each other, no drift to the end stops.
- **Consequences:** no vendor gripper mesh or driver; a Robotiq can replace it later behind the same `GripperCommand` action. MoveIt has a `gripper` group and end effector but does not plan the fingers.
- **Status:** proposed, until the phase 2 gripper test passes.

## D-008 — Bins and floor as fixture links, like the table

- **Context:** phase 2 needs an input and a destination bin in Gazebo (physics), TF and the MoveIt collision world, and the floor in MoveIt (the gripper extends the arm's reach below the table top). D-006 left open whether fixtures beyond the table become planning-scene objects published by a node.
- **Decision:** extend D-006. Bin layout (centre on the table, inner size, wall thickness) lives in `cell.yaml`; `urdf/bin.xacro` builds an open-top bin (floor + four walls in one link, frame at the inner floor centre). Gazebo spawns table + bins as one model fixed to the world (`fixtures.urdf.xacro`, replacing `table.urdf.xacro`); the cell URDF adds the same links plus a collision-only `floor` box when `include_fixtures:=true` (renamed from `include_table`). Fixture pairs that touch or never move relative to each other have collisions disabled in the SRDF; arm and gripper links never do. The planning scene stays reserved for objects (known pose in phase 2, perceived from phase 3).
- **Evidence:** pytest checks both URDF variants against `cell.yaml` (bin pose, outer envelope, free inner volume, bins inside the table top and apart). Cell launch 2026-11-10: TF `world → input_bin` (0.40, −0.30, 0.76), `world → output_bin` (0.40, 0.30, 0.76); with the gripper pointing down and `gripper_tcp` 4 cm above each bin floor, MoveIt finds a collision-free IK solution at the bin centre and none above a wall (forced solutions there collide `gripper_palm`/fingers with the bin).
- **Consequences:** moving a bin means editing `cell.yaml` and restarting the cell; no runtime reconfiguration, which the MVP does not need.
- **Status:** accepted.

## D-009 — Object catalog, known-pose input, ground truth outside ROS

- **Context:** phase 2 picks objects at a known pose before any perception exists, and needs the simulator's object poses to judge success without letting them reach perception or manipulation (rule 5).
- **Decision:**
  - One object catalog (`openfactory_description/config/objects.yaml`); Gazebo models are generated from it (`scripts/scene_objects.py`, pytest), so geometry has a single source.
  - Phase 2 object poses come from a versioned trial config and are given to manipulation as fixed infrastructure input, documented as such; the harness spawns the object at that pose before the cycle. This is the known-pose baseline, not perception.
  - The harness spawns, removes and observes objects through Gazebo Transport (`scripts/gz_scene.py`). Ground truth is read from `/world/cell/dynamic_pose/info` by the harness's evaluation code only and is not bridged to ROS.
- **Evidence:** VM check 2026-11-10: bridging `Pose_V` to `tf2_msgs/TFMessage` drops the entity names (empty `frame_id`/`child_frame_id`), so a ROS topic would need an extra relay node; the Gazebo message carries the names. Cube, cylinder and prism spawned in the input bin rest at the expected heights (0.785, 0.800, 0.7816 m) with 0.00 mm drift in 5 s; a prism spawned standing on its triangular face rests with its origin at 0.790 m, confirming the extruded polyline is centred on the object frame; `create/blocking` and `remove/blocking` succeed.
- **Consequences:** ground truth is not visible in RViz; trial logs carry the evaluated poses instead. CLAUDE.md rule 5 still holds: no ROS ground-truth topic exists, and any future one lives under `/ground_truth/`.
- **Status:** accepted for phase 2.

## D-010 — Phase 2 motion strategy: separate plan/execute, straight lines near objects

- **Context:** the deterministic pick-and-place cycle must be collision-aware (rule 4), split planning and execution time (E-001 note), and hold objects that are gripped by friction only (D-007).
- **Decision:**
  - Free-space moves (ready, pre-grasp, back to ready) are OMPL plans through `/move_action` with `plan_only`, executed separately through `/execute_trajectory`.
  - Every move near or with an object is a collision-checked straight `tool0` line from `/compute_cartesian_path` (approach, lift, transfer at lift height, descend, retreat) and must be ≥ 99 % feasible. The transfer is a line, not a free plan, so the tool stays vertical: in the first trial a free-space transfer let the cube pivot out between the pads (`grasp_slipped` after transfer).
  - The object enters the planning scene at its known pose (raised 2 mm so it does not touch the bin floor), is attached to `gripper_tcp` by id with the gripper links as touch links, and is detached and removed after release. Attaching by id already moves the object out of the world; removing it in the same diff makes the whole request fail.
  - Grasp evidence: the left finger must stop within 3 mm of half the grasp width after closing (`grasp_failed`) and after lift and transfer (`grasp_slipped`).
  - On a failure the cycle stops and recovers: open, clear the scene object, straight rise, joint move to ready. A free plan out of a bin is rejected by MoveIt's ValidateSolution (fingers graze the walls between OMPL's checked states).
- **Evidence:** `pick_place_once` 2026-11-10 in the running cell: cube, cylinder and prism each picked and placed at the first attempt (20.0, 20.8, 25.1 s, planning ≈ 0.3 s of it); empty-bin cycle ends in `grasp_failed` at `close` and recovers to ready.
- **Consequences:** the straight transfer assumes bins reachable along a line at lift height, true for this cell; later phases with arbitrary placements may need orientation path constraints instead. Objects land about 1 cm off the place point along the finger axis (release pushes them); the closing test measures it.
- **Status:** accepted for phase 2.
