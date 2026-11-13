# Environment

The current state of the Mac and the VM first, then the dated log of audits, installations and checks it comes from. Re-audit and update this file whenever the VM or tool versions change; the log is appended to, not rewritten.

## Current state (checked 2026-11-13)

| Item | Value |
|---|---|
| Mac | MacBook Air M2, 8 GB RAM: repo, git, editor; no ROS |
| Virtualizer | UTM 5.0.8 (QEMU backend), "Shared Network"; SSH alias `doppelvm` |
| VM | Ubuntu 24.04.5 LTS aarch64, kernel 6.8.0-147, 4 cores, 3.8 GiB RAM + 3.8 GiB swap, 30 GB disk (15 GB free); shared with the `doppelbot` project |
| GPU | virtio (`virgl`), OpenGL 2.1 only: Gazebo renders with Ogre 1, on the VM display (D-005) |
| Python | 3.12.3 (system), NumPy 1.26.4, OpenCV 4.6.0 |
| ROS 2 | Jazzy, `rmw_fastrtps_cpp` (default) |
| apt packages installed for this project | phase 0: `ros-jazzy-ros-gz` 1.0.24 (Gazebo Sim 8.15), `ros-jazzy-moveit` 2.12.4, `mesa-utils`; phase 1: `ros-jazzy-ur-simulation-gz` 2.5.0, `ros-jazzy-ur-moveit-config` 3.9.0, `ros-jazzy-ur-robot-driver` 3.9.0 (needed by the vendor UR launch only). Nothing since. |
| pip packages | none |

Memory with the full cell (Gazebo server with the camera, MoveIt, depth model, perception, benchmark harness, GNOME session; no RViz): 1.5 GiB used, 2.4 GiB available, real-time factor ≈ 1.0, load average 3–4 on 4 cores (phase 5). RViz adds about 260 MB, the Gazebo GUI about 400 MB. Idle VM: 1.0 GiB used.

Ring of N arms (phase 6; Gazebo server with N cameras, one `move_group`, N depth models, N perceptions, GNOME session; no RViz): N = 2, 1.7 GiB used, 2.1 GiB available, real-time factor 1.0; N = 3, 1.8–1.9 GiB used, 1.9–2.0 GiB available, real-time factor about 0.9. Each bin costs about 270 MB (perception 180–195 MB, depth model 80 MB) and about 0.6 of a core; an arm without its camera costs little. Details in the log, "Phase 6 ring".

### Stack

| Area | Choice | Why |
|---|---|---|
| OS / ROS | Ubuntu 24.04 + ROS 2 Jazzy, as installed | Already working and healthy; no migration needed |
| Simulator | Gazebo Harmonic (Sim 8.15) via `ros-jazzy-ros-gz`, render engine `ogre` (D-005) | Official Jazzy pairing, binary on aarch64; `ogre2` does not run on the virtio GPU |
| Planning | MoveIt 2 2.12.4, OMPL via `openfactory_moveit_config` | Binary on aarch64; E-001 10/10 |
| Robot | UR5e (D-004) | Full apt stack (description, Gazebo sim, MoveIt config) exists for Jazzy; Franka would need a source build |
| Gripper | Project parallel-jaw gripper, `position_controllers/GripperActionController` (D-007) | No install; E-002 30/30 |
| Camera | Gazebo `rgbd_camera` with D415 geometry, depth error model in `depth_sensor_model` (D-011) | Renders with Ogre 1 on the VM display; no install |
| Perception | NumPy + OpenCV geometric baseline, `vision_msgs` output (D-012) | Already installed; 9 ms per frame |
| Learned ranking | Logistic regression fitted with NumPy, inference in pure Python (D-019) | No install; selection time unchanged (0.035 s median) |

### Repo access from the VM

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

### Operating the VM over SSH

The assistant drives the VM from the Mac with full control (`CLAUDE.md`, "Environment"). What works, and the traps that each cost a failed round trip:

- One-shot commands: `ssh doppelvm 'source /opt/ros/jazzy/setup.bash && source ~/openfactory_ws/install/setup.bash && <command>'`. A non-interactive shell loads neither ROS nor `.bashrc`.
- The cell with the camera needs the VM desktop display even with `headless:=true` (D-005). Over SSH, set `DISPLAY=:0 XAUTHORITY=$(ls /run/user/$(id -u)/.mutter-Xwaylandauth.* | head -1)`. `camera:=false perception:=false` runs without a display.
- Long-running programs run detached, so that the SSH session can end: `setsid nohup ros2 launch openfactory_bringup cell.launch.py headless:=true rviz:=false > ~/cell.log 2>&1 < /dev/null &`, inside the sourced command above and with the display variables. `scripts/vm/launch.sh NAME <ros2 launch arguments>` does the sourcing and the display variables and writes `~/NAME.log`: `ssh doppelvm 'setsid nohup /mnt/openfactory-ai/scripts/vm/launch.sh ring openfactory_bringup ring.launch.py n:=2 rviz:=false > /dev/null 2>&1 < /dev/null &'`.
- Stop a cell or a ring with `ssh doppelvm /mnt/openfactory-ai/scripts/vm/stop_cell.sh`: it ends the launch and kills what it leaves behind. A detached launch ignores `kill -INT` (a background job of a non-interactive shell starts with SIGINT ignored), and after `kill <launch pid>` Gazebo, the bridge and `robot_state_publisher` can stay.
- A detached `setsid nohup python3 ...` does not inherit the sourced ROS environment (`No module named 'rclpy'`). Benchmarks go through `scripts/vm/run_benchmark.sh NAME [--script FILE] <benchmark args>`, which sources ROS and the workspace, runs `scripts/benchmark.py` (or `FILE`, e.g. `relay_benchmark.py`), writes `~/NAME.log` and the exit code to `~/NAME.done`; `scripts/vm/run_chain.sh CHAIN "NAME|args" ...` runs several in a row (`~/CHAIN.progress`, `~/CHAIN.done`). Wait on the `.done` file.
- Before a run, check that the cell is up (`ps -eo pid,etimes,comm | grep -E 'gz|move_group'`): the VM may have been restarted between sessions. Nodes read their YAML at start: after a config or code change, rebuild and relaunch the cell.
- Stop other processes by PID (`kill <pid>`). Never use `pkill -f` or `pgrep -f` with a pattern inside `ssh doppelvm '...'` (a script file such as `stop_cell.sh` may): the pattern matches the remote shell's own command line, so `pkill` kills the SSH session (exit 255) and `pgrep` always finds a match.
- A launch that aborts leaves `gz sim` and the `spawner` processes orphaned; kill them before relaunching (`stop_cell.sh`).
- The `ros2 control` command line is not installed (`ros2controlcli`); list controllers with `ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers`.
- `move_group` keeps the planning-scene objects of a process that ended. The bin-picking loop empties the world at every scene (D-017); other scripts must do the same.
- Reading Gazebo's `/world/cell/dynamic_pose/info` (about 1 kHz) continuously from Python starves the process; sample it a few times per second (`GzScene.model_poses` from a thread).
- The VM clock differs from the Mac's: compare elapsed times (`ps -o etimes`), not wall times.
- `doppelbot` shares the VM: build and test it only with its venv active (`source ~/ros2_ws/.venv/bin/activate && python -m colcon ...`); the system `colcon` rewrites its shebangs and breaks its `kociemba` solver.

## Log

### Audit — 2026-10-28 (phase 0, first pass)

Read-only inspection of the existing VM; nothing installed or changed.

#### Host

| Item | Value |
|---|---|
| Mac | MacBook Air M2, 8 GB RAM |
| Virtualizer | UTM 5.0.8 (QEMU backend), "Shared Network" |
| Existing share | 9p/VirtFS tag `share` → `/mnt/doppelbot` (the `doppelbot` project); only one share is possible. Since replaced, see "Repo access from the VM" |

#### VM

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

#### Binary availability (apt, aarch64)

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

### Installation — 2026-10-28 (phase 0)

#### apt sources fix

The first `apt install` failed with unmet dependencies (`krb5-multidev`, `libbsd-dev`): `/etc/apt/sources.list.d/ubuntu.sources` listed only `noble noble-security` (changed on 2026-10-27; the installer original `ubuntu.sources.curtin.orig` also had `noble-updates noble-backports`). Installed libraries came from `noble-updates`, so the matching `-dev` packages were not reachable.

Fix: `Suites: noble noble-updates noble-security` (`noble-backports` not needed). Previous file kept as `ubuntu.sources.bak-2026-10-28`. The 27 upgradable packages from `noble-updates` were left as they are (no `apt upgrade`).

#### Installed

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

#### Rendering probe

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

#### Deferred at phase 0 closure (2026-10-28)

Agreed with the user; each is checked in the phase that first needs it.

- Phase 1: Gazebo GUI with `--render-engine ogre` and RAM with simulator + MoveIt 2 + RViz: both checked 2026-10-29, see below.
- Phase 3: Open3D via pip on aarch64 / Python 3.12. Not checked: dropped on 2026-11-11, the perception baseline does not need it (D-012).

### Phase 1 installation — 2026-10-29

`sudo apt install ros-jazzy-ur-simulation-gz ros-jazzy-ur-moveit-config`: 28 new packages, 0 upgraded, 0 removed (`ur_simulation_gz` 2.5.0, `ur_moveit_config` 3.9.0, `ur_description` 3.5.1, `gz_ros2_control` 1.2.20). Then `ros-jazzy-ur-robot-driver` 3.9.0 (10 new packages): `ur_moveit_config/launch/ur_moveit.launch.py` always starts `ur_robot_driver`'s `wait_for_robot_description` node, but its `package.xml` does not declare the dependency (upstream packaging bug), so the launch aborts without it.

#### UR5e smoke test (vendor launch, no project code)

`ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e gazebo_gui:=false` (Gazebo server only, `gz sim -s`; RViz from the MoveIt launch):

- Controllers: `joint_state_broadcaster` and `scaled_joint_trajectory_controller` active; `/clock` and `/joint_states` published.
- `MoveGroup` action from the command line: 4/4 joint goals planned with OMPL RRTConnect (including far ones), one executed through `scaled_joint_trajectory_controller` in Gazebo ("Goal reached, success!"); final joint error ≤ 0.01 rad.
- **Known issue, open:** every RViz MotionPlanning request fails with "Unable to solve the planning problem" (5 s timeout): `<random valid>` and interactive-marker goals, both "Plan" and "Plan & Execute". The same `move_group` solves 11/11 command-line `MoveGroup` requests. Ruled out on the command line: goal tolerance 1e-4, RViz-style workspace bounds (2 m cube in `world` or `base_link`), 10 planning attempts, start pose, URDF/SRDF name mismatch. Not yet checked: the RViz panel settings (Context and Planning tabs). Does not block phase 1, whose closing test uses a script; recheck once the project's own MoveIt launch replaces the vendor one. **Resolved 2026-10-29 with the project launch** (`openfactory_bringup cell.launch.py`): RViz "Plan & Execute" with `<random valid>` plans (OMPL RRTConnect) and executes. Root cause not isolated; the project launch differs in loading only the OMPL pipeline (the vendor one also loads CHOMP and Pilz), its own URDF/SRDF with matching robot names, and the standard `joint_trajectory_controller`.
- Harmless (vendor launch only; the project SRDF is named `ur`): URDF robot name `ur` vs SRDF `ur5e` logs "Semantic description is not specified for the same robot as the URDF"; groups and collision matrix still load (`/check_state_validity` returns valid).
- A launch that aborts leaves `gz sim` and the `spawner` processes orphaned; kill them before relaunching.

RAM with Gazebo (server) + MoveIt 2 + RViz + GNOME session: 1.46 GiB used, 2.44 GiB available, swap 0. Largest processes: `rviz2` 257 MB, `gz sim` 200 MB, `move_group` 105 MB, GNOME shell + Xwayland ~370 MB.

### Project cell launch — 2026-10-29 (phase 1, step 3)

`ros2 launch openfactory_bringup cell.launch.py` (Gazebo server, `move_group`, RViz), no new packages installed:

- `joint_state_broadcaster` and `joint_trajectory_controller` active; Gazebo real-time factor ≈ 1.00.
- TF matches `cell.yaml`: `world→table` (0.25, 0, 0.75), `world→base_link` (0, 0, 0.75).
- `/check_state_validity`: forearm pointing down into the table is invalid (contacts `forearm_link`, `wrist_*_link` with `table`); pointing up is valid.
- RViz Plan & Execute succeeds through `joint_trajectory_controller`.
- The world with the Sensors system (`render_engine` `ogre`) runs headless without `DISPLAY` while it has no sensors: `gz sim -s -r --iterations 2000`, 4.3 s, 120 MB max RSS.
- RAM with Gazebo server + MoveIt 2 + RViz + GNOME: 1.4 GiB used, 2.4 GiB available.
- Gazebo GUI (`headless:=false rviz:=false`, `gz sim -r --render-engine ogre` on the VM desktop): renders floor, table and arm, smooth according to the user; `gz sim gui` ≈ 400 MB RSS, `gz sim server` ≈ 190 MB; 1.5 GiB used, 2.3 GiB available.
- Harmless log lines: `MoveItConfigsBuilder` "Cannot infer URDF/SRDF" for `ur_moveit_config` (overridden), octomap "No 3D sensor plugin(s)", RViz `/recognize_objects` not available, duplicate `rviz2_moveit` rosout publisher.

### Phase 2 gripper check — 2026-11-10

`ros2 launch openfactory_bringup cell.launch.py headless:=false`, no new packages:

- `gripper_controller` (`position_controllers/GripperActionController`) active; it logs a deprecation in favour of `parallel_gripper_action_controller` (package not installed). `gz_ros2_control` logs `right_finger_joint` mimicking `left_finger_joint`; Gazebo's own "physics engine does not support mimic constraints" error is harmless, `gz_ros2_control` drives the mimic joint itself.
- `GripperCommand` goals 0 → 0.0425 → 0.02 → 0 → 0.0425: all reached, fingers within 0.2 mm of each other, after the joint-limit fix in D-007.
- `/check_state_validity`: the home state is valid; arm and gripper pushed into the table are rejected (`gripper_palm`, fingers vs `table`). TF `tool0 → gripper_tcp` = (0, 0, 0.085).

### Phase 2 objects check — 2026-11-10

- Gazebo Transport Python bindings (`gz.transport13`, `gz.msgs10`) come with the ROS vendor packages (`ros-jazzy-gz-transport-vendor` 0.0.9, `ros-jazzy-gz-msgs-vendor` 0.0.8); nothing installed.
- `/world/cell/create` and `/create/blocking` (and `remove`) both answer; the harness uses the blocking ones.
- Results: D-009.

### Phase 2 closing run — 2026-11-10

`cell.launch.py headless:=true rviz:=false` + `phase2_pick_place_test.py` (30 cycles, E-002): 1.3 GiB used, 2.5 GiB available before and after; Gazebo real-time factor ≈ 1.0 during cycles. With the Gazebo GUI and RViz open, the harness smoke run saw cycles slow down by up to 3× in wall time.

### Phase 3 camera and perception — 2026-11-11

No package installed. Already present and used: `ros-jazzy-vision-msgs` 4.1.1, `ros-jazzy-ros-gz-bridge` 1.0.24, `sensor_msgs_py`, `tf2_ros`, `ament_cmake_python`, `python3-opencv` 4.6.0, NumPy 1.26.4. Open3D (deferred at phase 0) is not needed: the perception baseline uses NumPy and OpenCV only (D-012).

`ros2 launch openfactory_bringup cell.launch.py headless:=true rviz:=false`, started from a terminal of the VM desktop (the camera renders with Ogre 1 on the display, D-005):

- The `rgbd_camera` sensor in the URDF-spawned fixtures model works: `/camera/camera_info` (640 × 360, fx = fy = 502.30, cx 320, cy 180, no distortion) and both images carry `camera_optical_frame`; TF `world → camera_optical_frame` = (0.40, −0.30, 1.46), looking down. Gazebo only warns that `gz_frame_id` is not defined in SDF.
- `ros_gz_bridge` with `lazy: true` on the two image topics; `/clock` arrives at about 790 Hz.
- Frame rate delivered to perception: 3.6–4.2 Hz (nominal 5 Hz), intervals 0.2–1.0 s. Real-time factor 1.00.
- CPU (4 cores, `top`): `gz sim` 50–57 %, `scene_perception` 17–31 %, `depth_sensor_model` 17–22 %, `move_group` 17–25 %, bridge 8 %. The perception pipeline itself takes 9 ms per frame; the rest of the Python nodes' load is ROS message handling.
- RAM with Gazebo server + MoveIt + depth model + perception + GNOME, no RViz: 1.5 GiB used, 2.3 GiB available. Resident: `gz sim` 345 MB, `scene_perception` 174 MB, `move_group` 101 MB, `depth_sensor_model` 78 MB, bridge 55 MB.
- With `camera:=false` the fixtures model has no sensor and the cell runs without a display, as in phase 2.

### Phase 5 runs — 2026-11-12

No package installed. The learned grasp ranking is trained with NumPy 1.26.4 (`scripts/ranker_fit.py`, batch, seconds) and evaluated in pure Python inside `openfactory_manipulation`; scikit-learn and PyTorch are not installed and not needed. The VM was restarted before the phase 5 runs; the cell was relaunched over SSH with the display variables (D-005) and ran about 5.5 hours of benchmarks without a restart.

- RAM during a benchmark (Gazebo server + MoveIt + depth model + perception + harness + GNOME, no RViz): 1.5 GiB used, 2.4 GiB available; real-time factor ≈ 1.0, load average 3–4 on 4 cores. Resident: `gz sim` 357 MB, `scene_perception` 194 MB, harness 110 MB, `move_group` 105 MB.
- Trial logs of all phase 5 runs: 19 MB in `~/openfactory_data/trials`.

### Check — 2026-11-13

No package installed. Versions, kernel, disk and the installed apt packages are as in "Current state"; idle VM with no cell running: 1.0 GiB used, 2.8 GiB available. The benchmark runners used for the phase 5 runs (`~/p5_run.sh`, `~/p5_chain.sh`, VM only) are now versioned as `scripts/vm/run_benchmark.sh` and `scripts/vm/run_chain.sh` and run from the share (checked: ROS sourced, exit codes recorded, chain progress written).

### Phase 6 ring — 2026-11-13

No package installed. The ring of N arms (D-020) uses what the cell uses: `gz_ros2_control` 1.2.20 with several `ros2_control` systems in one model, one `move_group`, `xacro`, `ament_cmake_python`. `joint_state_publisher` is installed and not needed (one joint state broadcaster for every arm).

- `ring_simulation.launch.py n:=2 camera:=false` (no display needed): `joint_state_broadcaster` and a trajectory and a gripper controller per arm active; real-time factor 1.00; `gz sim` 245 MB; 1.2–1.3 GiB used.
- `self_collide` on the model of the arms: two arms sent into each other through their controllers stop each other (D-020); no spurious contact in the upright and ready configurations, fingers close fully.
- `ring.launch.py n:=2 rviz:=false` on the VM display: both perceptions publish at 2.5–3.5 Hz, real-time factor 1.00, 1.7 GiB used, 2.1 GiB available. Resident: `gz sim` 373 MB, `scene_perception` 178 MB each, `move_group` 98 MB, `depth_sensor_model` 80 MB each.
- `ring.launch.py n:=3 rviz:=false`: seven controllers active, three perceptions at 2.0–3.9 Hz, real-time factor 0.88–0.93 at rest, 1.8 GiB used and 2.0 GiB available (1.9 and 1.9 during a benchmark), load average above 7 on 4 cores. Resident: `gz sim` 414 MB, `scene_perception` 179–197 MB each, `move_group` 105 MB, `depth_sensor_model` 81 MB each. CPU: `gz sim` 108 %, each perception 33–35 %, each depth model 25 %, `move_group` 20 %.
- A turn of the relay loop takes about 28–30 s of wall time with N = 2 and about 40 s with N = 3 (same simulated work, slower simulation).
- Traps met: `rcl` refuses a generated parameter file with YAML anchors; a detached launch ignores SIGINT; `ros2 control` is not installed. All three are in "Operating the VM over SSH".
