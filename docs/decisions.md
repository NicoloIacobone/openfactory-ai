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

## D-004 — Robot arm

- **Context:** the brief suggests a mature arm such as UR5e or Franka Panda.
- **Decision:** UR5e, from the Jazzy apt packages (`ur_description`, `ur_simulation_gz`, `ur_moveit_config`); Franka has no Jazzy apt package here.
- **Evidence:** smoke test 2026-10-29 (`docs/environment.md`): MoveIt 2 plans and executes through `scaled_joint_trajectory_controller` in Gazebo; full stack uses 1.46 GiB of 3.8 GiB.
- **Status:** accepted. Gripper chosen in phase 2.

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
