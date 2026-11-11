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
- **Status:** accepted (E-002: 30/30 friction grasps, no slip).

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
  - Grasp evidence: the jaw opening (left + right finger) must stay within 4 mm of the object width after closing (`grasp_failed`) and after lift and transfer (`grasp_slipped`). One finger alone is not evidence: while holding, the commanded finger reaches its close target and the mimic finger yields, so the object shifts about 5 mm toward one pad with the same squeeze. A left-finger-only check flagged held objects as slipped and the recovery then dropped them (first harness smoke run).
  - On a failure the cycle stops and recovers: open, clear the scene object, straight rise, joint move to ready. A free plan out of a bin is rejected by MoveIt's ValidateSolution (fingers graze the walls between OMPL's checked states).
- **Evidence:** `pick_place_once` 2026-11-10 in the running cell: cube, cylinder and prism each picked and placed at the first attempt (20.0, 20.8, 25.1 s, planning ≈ 0.3 s of it); empty-bin cycle ends in `grasp_failed` at `close` and recovers to ready.
- **Consequences:** the straight transfer assumes bins reachable along a line at lift height, true for this cell; later phases with arbitrary placements may need orientation path constraints instead. Objects land about 1 cm off the place point along the finger axis (release pushes them); the closing test measures it.
- **Status:** accepted for phase 2.

## D-011 — Fixed overhead RGB-D camera modelled on the RealSense D415, with a simple depth error model

- **Context:** phase 3 needs one fixed RGB-D camera whose pose appears once in the description, TF and MoveIt, light enough for the 3.8 GB VM, with data that are not unrealistically clean. Camera parameters come from the D415 datasheet (Intel 337029-017, September 2023), read for the `synthetic-rgbd-manufacturing-perception` project.
- **Decision:**
  - Geometry of a D415 depth mode: 640 × 360 at 5 Hz, depth FOV H 65° (square pixels give V 39.4°; the datasheet's 71° is the FOV of the infrared imagers, which matters when simulating the stereo pair, not its output), Min-Z 0.24 m at this resolution, valid up to 2 m (range of the datasheet KPIs), baseline 55 mm, body 99 × 23 × 20 mm. One optical centre for colour and depth.
  - Mount: fixed, straight above the centre of the input bin, optical axis vertical, 0.70 m above the bin floor (world z 1.46 m): about 1.4 mm per pixel on the floor, the whole bin and its rim in view, the walls never hide the floor, every catalog object beyond Min-Z. Pose in `cell.yaml` (`camera_mount`), sensor in `config/camera.yaml`, macro `urdf/camera.xacro`: the body is a fixture link like the bins (D-008), a collision obstacle for MoveIt and Gazebo; the support (ceiling bracket) is not modelled.
  - The Gazebo `rgbd_camera` is a `<gazebo reference>` block of the macro, emitted only for the fixtures model. Messages carry `camera_optical_frame` through `gz_frame_id`. Image and depth are bridged lazily (rendered only while a ROS node subscribes); `camera:=false` drops the sensor for runs without a display (D-005).
  - Depth error model in a separate node (`openfactory_simulation/depth_sensor_model`), between the bridge and perception: Gaussian noise of 0.08 px on the disparity f B / z (σ_z = 1.4 mm at 0.70 m, 2.9 mm at 1 m), rounding to the 1 mm depth unit, NaN outside [Min-Z, 2 m]. Noise is seeded per frame from (seed, image stamp). The closing test runs with the model on.
- **Evidence:** E-003 part A (2026-11-11): `camera_info` equals the pinhole model of `camera.yaml`; the bin floor seen by the camera is the plane z = 0 of `input_bin` within 0.0001 mm and 0.0002°; a known point reprojects within 0.2 px; the measured depth noise on the floor is 1.452 mm against 1.448 mm from the model. URDF-to-SDF conversion keeps the sensor on the lumped `table` link at the mount pose; `gz sdf` only warns that `gz_frame_id` is not in the SDF schema. Cell with camera, depth model and perception: real-time factor 1.0, 1.5 GiB used.
- **Calibration finding:** Gazebo reports the principal point as (width / 2, height / 2) while pixel (u, v) looks along the ray of (u + 0.5, v + 0.5), half a pixel off the convention of integer pixel centres (0.7 mm on the bin floor). Perception adds a measured correction of (−0.5, −0.5) px to the `camera_info` principal point (`perception.yaml`); K itself is never hard-coded.
- **Limits:** the depth error is independent per pixel. It is not a stereo-matching simulation: no spatially correlated error, no holes on shiny or textureless surfaces, no flying pixels at depth edges, no invalid band at the left image border, no Min-Z from a disparity search range. The `synthetic-rgbd-manufacturing-perception` experiment found that such a noise-on-ideal-depth model is optimistic compared with simulated stereo matching, strongly so on metal; the catalog objects here are matte. Ogre 1 renders flat-shaded colour (D-005), so colour is used for debug views only. Delivered frame rate is 3.6–4.2 Hz, below the nominal 5 Hz.
- **Status:** accepted for phase 3.

## D-012 — Geometric perception baseline: top-view height map and model-based primitive recognition

- **Context:** phase 3 needs object hypotheses (class, pose, confidence) from camera data and TF only, cheap enough for the VM, as the baseline any learned perception is later compared with (rule 3).
- **Decision:**
  - NumPy and OpenCV only, both already installed; Open3D is not needed (the pip check deferred since phase 0 is dropped). Pure functions in `openfactory_perception/core`, a thin node around them.
  - Pipeline per depth frame: depth → points in the optical frame with K from `camera_info` (NaN, inf, 0 and out-of-range pixels rejected) → `input_bin` frame through TF at the image stamp → crop to the inner bin volume (5 mm from the walls, below 0.12 m) and remove the floor as the known plane z = 0 with a 6 mm threshold (about 4 σ of the depth noise) → mean-height map on a 2 mm top-view grid, which is also the downsampling → morphological closing and opening, connected components, blobs under 3 cm² dropped → per blob: top-layer height, minimum-area footprint rectangle, filled share of the rectangle, share of the footprint at top height.
  - Recognition is model-based: each catalog shape (`objects.yaml`, fixed infrastructure) predicts those features, including the effect of the floor threshold on the prism's slanted faces; the class is the shape with the smallest normalised residual and the confidence is exp(−residual / 2). Below 0.2 the cluster is published as `unknown`, never dropped silently. Pose: footprint centre, catalog height / 2, yaw from the footprint's long axis reduced by the shape's symmetry (cube 90°, prism 180°, cylinder none).
  - Output: `vision_msgs/Detection3DArray` in `world`, one per depth frame, stamped with the image stamp (`docs/architecture.md`). Colour is not used for recognition.
- **Scope:** separate objects resting in their catalog orientation. Touching or stacked objects, tumbled objects, and anything in the bin volume that is not a catalog object (the arm during a pick) are phase 4.
- **Evidence:** 63 pytest cases on synthetic z-buffered views of the catalog objects (each step, and the whole pipeline with and without the depth model); synthetic Monte Carlo before the first live run: 1800 random single-object views, 0 wrong classes, position error at most 1.1 mm, yaw at most 2.1°, 0 detections in 200 noisy empty bins, 9 ms per frame. Live: E-003 part B.
- **Status:** accepted for phase 3.

## D-013 — Camera-driven pick: the most confident hypothesis replaces the known pose

- **Context:** phase 3 must show the perception output driving a pick, without anticipating the grasp selection and recovery of phase 4.
- **Decision:** `openfactory_manipulation.camera_pick.perceive` moves the arm to the ready configuration (clear of the input bin), waits 0.5 s, takes the first `/perception/objects` message stamped after that, and selects the hypothesis with the highest confidence among catalog classes at or above 0.5. Its class and its pose, expressed in `input_bin`, are the input of the unchanged `PickPlace.run` cycle (D-010), which also places the object in the planning scene. The object rests on the bin floor, so its height comes from the catalog. `PickPlace.run` records the pose source (`known_pose` or `perception`) in its result (`object_pose`, formerly `known_pose`). No hypothesis ends the attempt as `no_detection`.
- **Consequences:** one grasp per object and no feasibility search: the closing test draws object centres from the middle of the bin (±75 mm × ±35 mm), where the palm, which goes below the bin rim, clears the walls for any yaw. Phase 4 replaces the selection with grasp candidates and hard feasibility filters and widens the region.
- **Evidence:** E-003 part C.
- **Status:** accepted for phase 3.

## D-014 — Gripper fingers longer than the bin walls

- **Context:** with the 60 mm fingers of D-007 the palm (120 × 60 mm) went below the rim of the 100 mm bin walls at every grasp, so an object could only be picked if its centre was about 60 mm from the walls along the finger axis and 30 mm across it. D-013 worked around it by drawing objects in the middle of the bin (±75 × ±35 mm). Phase 4 picks objects anywhere in the bin.
- **Decision:** fingers 110 mm long (`gripper.yaml`: `finger.size[2]` 0.06 → 0.11, `tcp_offset` 0.085 → 0.135, so `gripper_tcp` stays 15 mm above the fingertips). With the fingertips 5 mm above a bin floor the palm is 115 mm above it, 15 mm over the rim: only the fingers (20 × 10 mm each) enter a bin. Everything else of D-007 is unchanged: stroke, joint limits and margins, effort, friction grasp, no attach mechanism. `ready_joints` is unchanged (`gripper_tcp` now 0.24 m above the bin floors).
- **Evidence:** pytest in `openfactory_description` (the fingers are longer than any bin wall is high) and in `openfactory_manipulation` (`test_grasps.py`: a cube 5 mm from a wall has a feasible grasp along the wall with these fingers and none with the short ones). Live, 2026-11-11, same cycle and configs as E-002 and E-003: `phase2_pick_place_20261111T130001Z_s1` 3/3 (one per type), 0 of 570 executed states in collision, place error mean 10.2 mm, max 17.5 mm (E-002: 9.9 / 17.5); `phase3_camera_pick_20261111T130115Z_s1` 6/6 (two per type), 0 of 1126 states in collision, cycle 17.7–18.8 s (E-003: median 18.4 s). The fingers open, pre-close, close and release in every cycle, so the joint-limit trap of D-007 did not come back.
- **Consequences:** `tool0` works 50 mm higher for the same grasp. The numbers of E-002 and E-003 were measured with the short fingers; the smoke runs above show the same behaviour, not a re-run of those tests.
- **Status:** accepted (user's choice for phase 4, 2026-11-11).

## D-015 — A held prism can turn between the pads: measured, grasp height kept

- **Context:** E-003 part C lost 2 of 10 prisms that ended in the output bin on another rectangular face: the prism turns about the finger axis while it is carried, and the jaw opening cannot see it. Phase 4 had to find the cause and a fix that keeps the friction grasp of D-007 (no attach mechanism).
- **Measurement:** `scripts/grasp_hold_probe.py` (config `scripts/config/phase4_hold_probe.yaml`, long fingers of D-014) runs the unchanged known-pose cycle of D-010 on a prism at 12 yaws, 15° apart, and samples its ground-truth pose every 0.2 s; the roll about the finger axis while the prism is in the air is compared between variants of `pick_place.yaml`. Runs of 2026-11-11: `phase4_hold_probe_20261111T162928Z_s1000` (five variants × 12), `…165106Z` (baseline × 24), `…165940Z` (ridge grasp × 24); 108 transfers.

| Variant | Transfers | Roll above 9° | Largest roll | Ended on another face |
|---|---|---|---|---|
| Baseline: TCP at the object centre, lines at 0.1 scaling | 36 | 5 | 114° | 1 |
| Lines at half speed (0.05) | 12 | 1 | 10° | 0 |
| Lines three times as fast (0.3) | 12 | 1 | 16° | 0 |
| Grasp 5 mm higher | 12 | 1 | 100° | 1 |
| Grasp 10 mm higher | 12 | 1 | 77° | 1 |
| Ridge grasp, 25 mm higher | 24 | 21 | 40° | 0 |

- **Findings:**
  - It is not a slip under acceleration. In every variant about one transfer in ten shows it, whatever the speed, and the roll appears as a jump between two samples during the transfer, after which the prism stays where it went. In the other transfers the roll is below 1°.
  - The size of the jump is that of a pendulum hanging from one lower corner of a pad. The pad is 20 mm wide, its lower edge 6.7 mm above the prism base in the baseline grasp; the centre of mass is 14.4 mm above the base. Hanging from a pad corner 10 mm off the centre line, the centre of mass comes to rest below it after 77° with the grasp 10 mm higher (measured 77°), 105° with 5 mm (measured 100°) and 128° at the baseline height (measured up to 114°). So the hold sometimes degenerates to one contact point per pad, which carries no torque about the finger axis; this is how the simulator builds contacts between the box pad and the triangular end face of the extruded mesh, not a lack of squeeze (20 N per finger, friction 1.0, object 0.1 kg).
  - Holding only the ridge, where the prism is narrower than the pads, bounds the swing (no pad corner touches the face) and never changed the face, but the prism then swings in nearly every transfer (median 24°).
  - No prism was dropped and none left the output bin: 108 of 108 deposited, and the jaw-opening checks passed in all of them.
- **Decision:** keep the grasp at the object centre and the line speeds of D-010. The hold is rigid in about nine transfers out of ten; one transfer in 60 at this height ended on another face (95 % Wilson interval 0.3–8.9 %), which the ridge grasp (0 of 24, interval 0–13.8 %) does not beat with the data at hand, while it makes the object swing every time. Raising the grasp by 5 or 10 mm is worse and is ruled out.
- **Consequences:** the goal set for this probe (no prism on another face in 30 transfers) is not met; it is a documented limit of the friction grasp on this shape in this simulator. With the phase 4 place criterion (D-017: at rest inside the output bin, on any face) it is not a failure, and E-004 reports how many deposited objects are not upright. The cell still cannot see the turn: the held object is hidden by the gripper from the only camera. Not tried: torsional friction or a different collision shape for the pads or the prism end faces; either needs a look at what the physics engine supports and a new entry.
- **Status:** accepted for phase 4.
