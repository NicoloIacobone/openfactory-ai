# Environment

Audited environment and the stack selected from it. Re-audit and update this file whenever the VM or tool versions change.

## Audit — 2026-10-28 (phase 0, first pass)

Read-only inspection of the existing VM; nothing installed or changed.

### Host

| Item | Value |
|---|---|
| Mac | MacBook Air M2, 8 GB RAM |
| Virtualizer | UTM 5.0.8 (QEMU backend), "Shared Network" |
| Existing share | 9p/VirtFS tag `share` → `/mnt/doppelbot` (the `doppelbot` project); only one share is possible. Since replaced, see "Repo access from the VM" |

### VM

| Item | Value |
|---|---|
| OS | Ubuntu 24.04.5 LTS (noble), aarch64, kernel 6.8.0-147 |
| CPU | 4 cores |
| RAM | 3.8 GiB (≈2.9 GiB available idle), swap 3.8 GiB |
| Disk | 30 GB root, 16 GB free |
| GPU | Virtio 1.0 GPU, `/dev/dri/renderD128`, Mesa `libgl1-mesa-dri` 25.2.8; OpenGL version not measured (`mesa-utils` not installed) |
| Python | 3.12.3 (system), NumPy 1.26.4 |
| ROS 2 | Jazzy (`/opt/ros/jazzy`), `rmw_fastrtps_cpp` |
| `ros2 doctor` | all 5 checks passed (only minor "newer version available" warnings) |
| Pub/sub | `std_msgs/String` publish + `ros2 topic echo --once`: OK |
| Gazebo | not installed (`gz` absent; only `gz-cmake/math/utils` vendor packages pulled in by ROS) |
| MoveIt 2 | not installed |

### Binary availability (apt, aarch64)

| Package | Candidate |
|---|---|
| `ros-jazzy-ros-gz` (Gazebo Harmonic bridge + sim) | 1.0.24 |
| `ros-jazzy-gz-ros2-control` | 1.2.20 |
| `ros-jazzy-moveit` | 2.12.4 |
| `ros-jazzy-ur`, `ros-jazzy-ur-description` | 3.9.0, 3.5.1 |
| `ros-jazzy-ur-simulation-gz` | 2.5.0 |
| `ros-jazzy-ur-moveit-config` | 3.9.0 |
| Franka description for Jazzy | no apt package found |
| `python3-open3d` | none |

## Installation — 2026-10-28 (phase 0)

### apt sources fix

The first `apt install` failed with unmet dependencies (`krb5-multidev`, `libbsd-dev`): `/etc/apt/sources.list.d/ubuntu.sources` listed only `noble noble-security` (changed on 2026-10-27; the installer original `ubuntu.sources.curtin.orig` also had `noble-updates noble-backports`). Installed libraries came from `noble-updates`, so the matching `-dev` packages were not reachable.

Fix: `Suites: noble noble-updates noble-security` (`noble-backports` not needed). Previous file kept as `ubuntu.sources.bak-2026-10-28`. The 27 upgradable packages from `noble-updates` were left as they are (no `apt upgrade`).

### Installed

`sudo apt install ros-jazzy-ros-gz ros-jazzy-moveit mesa-utils`: 245 new packages, 0 upgraded, 0 removed; `dpkg --audit` clean.

| Package | Version |
|---|---|
| `ros-jazzy-ros-gz` / `ros-jazzy-ros-gz-bridge` | 1.0.24 |
| `ros-jazzy-gz-sim-vendor` | 0.0.13 (`gz sim --version`: Gazebo Sim 8.15.0, Harmonic) |
| `ros-jazzy-moveit` / `ros-jazzy-moveit-ros-move-group` | 2.12.4 |
| `mesa-utils` | 9.0.0-2 |

Not installed yet: `ros-jazzy-gz-ros2-control` (not pulled in by `ros-gz`) and the UR packages (phase 1).

After install: 2.8 GiB RAM available idle, 15 GB disk free.

Regression check: `doppelbot` rebuilt and tested with its venv active (its README procedure: `source ~/ros2_ws/.venv/bin/activate`, `python -m colcon`), 214 tests, 0 failures, 0 skipped. Building it with the system `colcon` instead rewrites its entry-point shebangs to `/usr/bin/python3` and breaks the `kociemba` solver.

### Rendering probe

OpenGL (`glxinfo -B` on the VM session, `DISPLAY=:0`, `XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.*`):

| Renderer | OpenGL |
|---|---|
| `virgl` (virtio GPU, default) | 2.1 compatibility, max core profile 0.0 |
| `llvmpipe` (`LIBGL_ALWAYS_SOFTWARE=1`) | 4.5 core |

Gazebo, physics only: `gz sim -s -r --iterations 2000 empty.sdf` runs in 4.3 s, 118 MB max RSS, no display needed.

Gazebo, RGB-D: a minimal world with a static `rgbd_camera` (320×240, 5 Hz) facing a 0.5 m box whose near face is 0.75 m away; one message read from `/rgbd/image` and `/rgbd/depth_image` with `gz topic -e -n 1`.

| Render engine | Context | Result |
|---|---|---|
| `ogre2` | `--headless-rendering` (EGL) | segfault in Ogre-Next while creating materials |
| `ogre2` | EGL + `LIBGL_ALWAYS_SOFTWARE=1` | segfault; EGL refuses software rendering on an explicitly selected device |
| `ogre2` | Xwayland `:0` + `LIBGL_ALWAYS_SOFTWARE=1` | same segfault (Ogre-Next still goes through EGL) |
| `ogre` | `--headless-rendering` | fails: Ogre 1 needs an X display |
| `ogre` | Xwayland `:0`, `virgl` | works: RGB `RGB_INT8` and depth `R_FLOAT32` at 320×240; center depth 0.750 m (expected 0.75), depth finite only on the box; RGB flat-shaded (2 distinct values); ~216 MB RSS; segfaults on shutdown after SIGTERM |

Consequence: decision D-005 in `docs/decisions.md`. Sensor simulations need the VM graphical session logged in.

### Deferred at phase 0 closure (2026-10-28)

Agreed with the user; each is checked in the phase that first needs it.

- Phase 1: Gazebo GUI with `--render-engine ogre` and RAM with simulator + MoveIt 2 + RViz: both checked 2026-10-29, see below.
- Phase 3: Open3D via pip on aarch64 / Python 3.12.

## Phase 1 installation — 2026-10-29

`sudo apt install ros-jazzy-ur-simulation-gz ros-jazzy-ur-moveit-config`: 28 new packages, 0 upgraded, 0 removed (`ur_simulation_gz` 2.5.0, `ur_moveit_config` 3.9.0, `ur_description` 3.5.1, `gz_ros2_control` 1.2.20). Then `ros-jazzy-ur-robot-driver` 3.9.0 (10 new packages): `ur_moveit_config/launch/ur_moveit.launch.py` always starts `ur_robot_driver`'s `wait_for_robot_description` node, but its `package.xml` does not declare the dependency (upstream packaging bug), so the launch aborts without it.

### UR5e smoke test (vendor launch, no project code)

`ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e gazebo_gui:=false` (Gazebo server only, `gz sim -s`; RViz from the MoveIt launch):

- Controllers: `joint_state_broadcaster` and `scaled_joint_trajectory_controller` active; `/clock` and `/joint_states` published.
- `MoveGroup` action from the command line: 4/4 joint goals planned with OMPL RRTConnect (including far ones), one executed through `scaled_joint_trajectory_controller` in Gazebo ("Goal reached, success!"); final joint error ≤ 0.01 rad.
- **Known issue, open:** every RViz MotionPlanning request fails with "Unable to solve the planning problem" (5 s timeout): `<random valid>` and interactive-marker goals, both "Plan" and "Plan & Execute". The same `move_group` solves 11/11 command-line `MoveGroup` requests. Ruled out on the command line: goal tolerance 1e-4, RViz-style workspace bounds (2 m cube in `world` or `base_link`), 10 planning attempts, start pose, URDF/SRDF name mismatch. Not yet checked: the RViz panel settings (Context and Planning tabs). Does not block phase 1, whose closing test uses a script; recheck once the project's own MoveIt launch replaces the vendor one. **Resolved 2026-10-29 with the project launch** (`openfactory_bringup cell.launch.py`): RViz "Plan & Execute" with `<random valid>` plans (OMPL RRTConnect) and executes. Root cause not isolated; the project launch differs in loading only the OMPL pipeline (the vendor one also loads CHOMP and Pilz), its own URDF/SRDF with matching robot names, and the standard `joint_trajectory_controller`.
- Harmless (vendor launch only; the project SRDF is named `ur`): URDF robot name `ur` vs SRDF `ur5e` logs "Semantic description is not specified for the same robot as the URDF"; groups and collision matrix still load (`/check_state_validity` returns valid).
- A launch that aborts leaves `gz sim` and the `spawner` processes orphaned; kill them before relaunching.

RAM with Gazebo (server) + MoveIt 2 + RViz + GNOME session: 1.46 GiB used, 2.44 GiB available, swap 0. Largest processes: `rviz2` 257 MB, `gz sim` 200 MB, `move_group` 105 MB, GNOME shell + Xwayland ~370 MB.

## Project cell launch — 2026-10-29 (phase 1, step 3)

`ros2 launch openfactory_bringup cell.launch.py` (Gazebo server, `move_group`, RViz), no new packages installed:

- `joint_state_broadcaster` and `joint_trajectory_controller` active; Gazebo real-time factor ≈ 1.00.
- TF matches `cell.yaml`: `world→table` (0.25, 0, 0.75), `world→base_link` (0, 0, 0.75).
- `/check_state_validity`: forearm pointing down into the table is invalid (contacts `forearm_link`, `wrist_*_link` with `table`); pointing up is valid.
- RViz Plan & Execute succeeds through `joint_trajectory_controller`.
- The world with the Sensors system (`render_engine` `ogre`) runs headless without `DISPLAY` while it has no sensors: `gz sim -s -r --iterations 2000`, 4.3 s, 120 MB max RSS.
- RAM with Gazebo server + MoveIt 2 + RViz + GNOME: 1.4 GiB used, 2.4 GiB available.
- Gazebo GUI (`headless:=false rviz:=false`, `gz sim -r --render-engine ogre` on the VM desktop): renders floor, table and arm, smooth according to the user; `gz sim gui` ≈ 400 MB RSS, `gz sim server` ≈ 190 MB; 1.5 GiB used, 2.3 GiB available.
- Harmless log lines: `MoveItConfigsBuilder` "Cannot infer URDF/SRDF" for `ur_moveit_config` (overridden), octomap "No 3D sensor plugin(s)", RViz `/recognize_objects` not available, duplicate `rviz2_moveit` rosout publisher.

## Selected stack

| Area | Choice | Why |
|---|---|---|
| OS / ROS | Ubuntu 24.04 + ROS 2 Jazzy, as installed | Already working and healthy; no migration needed |
| Simulator | Gazebo Harmonic (Sim 8.15) via `ros-jazzy-ros-gz`, render engine `ogre` (D-005) | Official Jazzy pairing, binary on aarch64; `ogre2` does not run on the virtio GPU |
| Planning | MoveIt 2 2.12.4, OMPL via `openfactory_moveit_config` | Binary on aarch64; E-001 10/10 |
| Robot | UR5e (D-004) | Full apt stack (description, Gazebo sim, MoveIt config) exists for Jazzy; Franka would need a source build |

Final choices are recorded in `docs/decisions.md` once installation and a phase-1 test confirm them.

## Repo access from the VM

The Mac is the source of truth. UTM 5.0.8 (QEMU backend) offers a single VirtFS shared folder, so both projects live in `~/Desktop/nico/github_portfolio/ros_2_projects/` (`doppelbot/`, `openfactory-ai/`) and that folder is the share:

| Mac | VM |
|---|---|
| `~/Desktop/nico/github_portfolio/ros_2_projects` | `/mnt/ros_2_projects` (9p, tag `share`, in `/etc/fstab`) |
| `~/Desktop/nico/github_portfolio/ros_2_projects/openfactory-ai` | `/mnt/openfactory-ai` → symlink to `/mnt/ros_2_projects/openfactory-ai` |
| `~/Desktop/nico/github_portfolio/ros_2_projects/doppelbot` | `/mnt/doppelbot` → symlink to `/mnt/ros_2_projects/doppelbot` |

The colcon workspace lives outside the share, at `~/openfactory_ws` in the VM.

The share is **read-only for the VM user**: files keep the Mac owner (uid 501, mode 755) and the VM user is uid 1000 (`access=client`), so `mkdir` fails with "Permission denied". Run outputs are written in the VM under `~/openfactory_data/` and copied to the repo's git-ignored `data/` from the Mac: `scp -r doppelvm:openfactory_data/trials/<run_id> data/trials/`.

Status: verified 2026-10-28 (mount, symlinks, `doppelbot` build and 214 tests pass through the new paths); original fstab kept as `/etc/fstab.bak`.

2026-11-10: on the Mac the share moved from `~/Desktop/nico/ros_2_projects` to `~/Desktop/nico/github_portfolio/ros_2_projects` (whole folder, same contents). The UTM shared folder must point to the new path; nothing changes inside the VM.
