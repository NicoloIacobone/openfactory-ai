# Environment

The current state of the Mac and the VM first, then the dated log of audits, installations and checks it comes from. Re-audit and update this file whenever the VM or tool versions change; the log is appended to, not rewritten.

## Current state (checked 2026-11-14)

| Item | Value |
|---|---|
| Mac | MacBook Air M2, 8 GB RAM: repo, git, editor; no ROS |
| Virtualizer | UTM 5.0.8 (QEMU backend), "Shared Network"; SSH alias `doppelvm` |
| VM | Ubuntu 24.04.5 LTS aarch64, kernel 6.8.0-147, 4 cores, 3.8 GiB RAM + 3.8 GiB swap, 30 GB disk (15 GB free); shared with the `doppelbot` project |
| GPU | virtio (`virgl`), OpenGL 2.1 only: Gazebo renders with Ogre 1, on the VM display (D-005) |
| Python | 3.12.3 (system), NumPy 1.26.4, OpenCV 4.6.0 |
| ROS 2 | Jazzy, `rmw_fastrtps_cpp` (default) |
| apt packages installed for this project | phase 0: `ros-jazzy-ros-gz` 1.0.24 (Gazebo Sim 8.15), `ros-jazzy-moveit` 2.12.4, `mesa-utils`; phase 1: `ros-jazzy-ur-simulation-gz` 2.5.0, `ros-jazzy-ur-moveit-config` 3.9.0, `ros-jazzy-ur-robot-driver` 3.9.0 (needed by the vendor UR launch only). Nothing since: the Gazebo system plugin of phase 7 builds against the headers of the `gz_*_vendor` packages that `ros-jazzy-ros-gz` brought in. |
| pip packages | none in the system Python. In a venv of this project, `~/openfactory_ws/.venv_solver` (phase 7, D-028): `kociemba` 1.2.1 with `cffi` 2.1.1, `future` 1.0.0, `pycparser` 3.11, from `requirements-vm.txt` |

Memory with the full cell (Gazebo server with the camera, MoveIt, depth model, perception, benchmark harness, GNOME session; no RViz): 1.5 GiB used, 2.4 GiB available, real-time factor ≈ 1.0, load average 3–4 on 4 cores (phase 5). RViz adds about 260 MB, the Gazebo GUI about 400 MB. Idle VM: 1.0 GiB used.

Rubik's cube cell (phase 7; Gazebo server with the camera, MoveIt, depth model, `cube_perception`, GNOME session; no RViz): 1.4 GiB used at launch, 2.5 GiB available; the Gazebo server grows by 1.3–1.7 MB at every change of axis of the cube (known issue below), so the benchmark relaunches the cell for every cube.

Ring of N arms (phase 6; Gazebo server with N cameras, one `move_group`, N depth models, N perceptions, benchmark harness, GNOME session; no RViz), during a benchmark: N = 2, 1.8 GiB used, 2.1 GiB available, real-time factor 0.7–1.0; N = 3, 2.0 GiB used, 1.9 GiB available, real-time factor 0.2–0.5; N = 4, 2.1–2.2 GiB used, 1.7–1.8 GiB available, real-time factor 0.3–0.4 (one sample per run, two runs each; the speed of the VM changes from run to run); load average 7–10 on 4 cores in all three. Each bin costs about 270 MB (perception 180–195 MB, depth model 80 MB) and about 0.6 of a core, nearly all of it spent by its two Python nodes on the 800 Hz `/clock`; an arm without its camera costs little. Memory is not what limits N here, CPU is. Details in the log, "Phase 6 ring".

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
| Cube solver | `kociemba` 1.2.1 (pip, GPLv2) in its own venv, on `PYTHONPATH` of the process that solves (D-028) | About 20 moves per cube; no apt package; the system Python stays as it is |

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
- A running `gz sim` keeps the system plugins it loaded: after rebuilding `openfactory_simulation` (the Rubik's cube mechanism, phase 7) stop and relaunch the simulation, or the old library keeps running.
- Gazebo Transport from Python: do not hold a subscription while calling a service (`node.request`). The request keeps the interpreter while it waits, the subscription callbacks then block the transport thread and the reply is never read: the call times out although the world served it (a model spawned by a call that "failed"). `GzScene.model_poses` subscribes and unsubscribes around one message for this reason; `scripts/rubik_mechanism_test.py` turns its subscriptions off around spawn and remove.
- Persistent wrenches of Gazebo's `ApplyLinkWrench` system (`/world/<world>/wrench/persistent`) add up: publish one, clear it (`wrench/clear`) before the next on the same link.
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
- During the relay benchmark (`~/p6_*.env`, sampled 5 minutes into each run of the first evaluation chain): N = 2, 1.79 GiB used, 2.11 GiB available, real-time factor 0.70, load average 7.1; N = 3, 1.97 GiB used, 1.93 GiB available, real-time factor 0.53 (0.50 at rest with the harness of the previous run just stopped), load average 6.8–8.7; N = 4, 2.18 GiB used, 1.72 GiB available, real-time factor 0.29, load average 8.4–9.2. No swap used.
- Where the CPU goes (N = 3, `top`): `gz sim` 124 %, each `scene_perception` 36 %, each `depth_sensor_model` 28 %, `move_group` 23 %, bridge 11 %. Lowering the cameras from 5 Hz to 2 Hz (`ring.yaml` `camera_rate_hz`, tried and reverted) leaves those numbers as they are while the views arrive at 0.6–0.9 Hz of wall time: the load of the Python nodes is not the frames. `/clock` arrives at 797 Hz, `/joint_states` at 414 Hz, `/tf` at 5 Hz; every node with `use_sim_time` handles each `/clock` message, in Python for these. `ros_gz_bridge` 1.0.24 has no throttle (its options are queue sizes, `lazy` and a QoS profile); Gazebo Transport's `SubscribeOptions` has `msgs_per_sec`.
- Consequence for timeouts: at half speed a wait of 10 s of wall time is 5 s for the simulated system. The wait for a camera view now runs on the node clock (D-021).
- Second evaluation chain, final code (`~/p6f_*.env`, same sampling; E-006): N = 2, 1.75 GiB used, 2.16 GiB available, real-time factor 0.96 (0.98 at rest), load average 7.6; N = 3, 1.96 GiB used, 1.94 GiB available, real-time factor 0.52 in one run and 0.20 in the other (0.86 and 0.62 at rest), load average 7.4–8.3; N = 4, 2.13 GiB used, 1.77 GiB available, real-time factor 0.42 (0.65 at rest), load average 9.7. No swap used. The speed is not a constant of N: with the same code and seed the ring of two arms made 113 handovers per hour of wall time in the first chain and 133 in the second, and the two runs with three arms 80 and 108, while the simulated time of 100 handovers was the same within 2 % (2761–2808 s) in all four. A single sample of the real-time factor is a poor measure: the run that showed 0.20 was the faster of the two.
- **Known issue, open: the Gazebo window with the ring.** `ring.launch.py n:=3 rviz:=false headless:=false` with the relay benchmark running: after three handovers, about six minutes from the launch, `gz sim gui` ended with a segmentation fault in its render thread (`MinimalScene` → `Ogre::SceneManager::renderSingleObject`, Ogre 1.9, "Address not mapped to object"). When the window dies `gz sim` stops the server as well and the launch shuts down, reporting that Gazebo "finished cleanly". 1.5 GiB of memory were available at that moment, 19 MB of swap used; the real-time factor with the window was 0.23. Seen once; a second identical run was stopped by the user after two handovers and three minutes, with no crash until then, so how often it happens is not known. With N = 2 and the window, two handovers ran before the window was closed. Without the window (`headless:=true`, the default) none of the evaluation runs of E-006 crashed. Not investigated further.

### Phase 7 probes and cube mechanism — 2026-11-14

No package installed. A Gazebo system plugin builds against what is there: `gz-sim8` and `gz-plugin2` headers and CMake configuration in `/opt/ros/jazzy/opt/gz_sim_vendor` and `gz_plugin_vendor` (`find_package(gz_sim_vendor)`, `find_package(gz-sim)`), gz-physics 7.8.0 with DART 6.13.2 (`gz_dartsim_vendor`), `g++` 13.2, `cmake` 3.28. Compiling the plugin takes a few seconds and about 550 MB.

Probes, with throwaway plugins and worlds in `~/rubik_probe` (physics only, no display), before the cube mechanism of D-023 was written:

| Question | Result |
|---|---|
| Can a plugin weld a free link of its own model to another link at run time? | Yes: an entity with `components::DetachableJoint` becomes a fixed joint of the model tree (a DART `WeldJoint`). Created in the first `PreUpdate`, the link never falls. |
| Can the weld be moved to another parent without a step in which the link is free? | Yes: `Physics::Update` creates new detachable joints before the step and removes old ones after it. Removal requested in one update and creation in the next: the link pose is identical to 17 digits across the change. |
| Can the plugin read the force and torque a weld carries? | Yes, with `components::Joint` and `components::JointTransmittedWrench` added to the weld entity: Physics fills the wrench as for a model joint. A torque-free 1 N force on the welded link reads as −1 N plus the link's weight, in the child link's frame. |
| Can a joint be set to an exact angle? | Yes: `components::JointPositionReset` to π/2 leaves the welded link at (1.2e-18, 0.019) m. |
| Are link poses of a model readable outside ROS? | Yes: `/world/<world>/dynamic_pose/info` carries every link by name, relative to its model. |
| Does the velocity command of a joint (`JointVelocityCmd`) work as a hold or a drive? | On a bare axle, yes, also after a spell of force commands. On an axle that carries welded links it does not move the joint (cause not found): the mechanism uses force commands only. |

Joint friction in DART is not what the SDF says once damping is there. With `<friction>` f and `<damping>` d on a revolute joint of inertia I, a joint at rest holds a torque of about f, but a turning one resists more: 4 f on the probe's axle (I = 1.7e-6 kg m², d = 0.005) and 1.44 f on a cube layer (I = 1.5e-5, d = 0.003), measured from the steady speed under a constant torque. The cube's parameters are therefore set from bench measurements, not from the SDF value.

Traps met (all in "Operating the VM over SSH"): a running `gz sim` keeps the plugin it loaded; no Gazebo Transport subscription during a service request from Python; persistent wrenches add up. A model spawned through `create` gets its plugin one step after its links: the cubies of a spawned cube fall g dt² = 9.8 µm before they are welded. A free cube falls as one; on the test bench, where the core is fixed to the world, the grid of cubies sits 9.8 µm below the core.

**Known issue, open: Gazebo leaks memory when a detachable joint is created and removed.** About 36 kB of the server per create-and-remove (53 kB with the wrench components), measured with a probe that welds and frees one link 10 000 times: 120 MB → 484 MB (651 MB); the same churn of plain entities, without `DetachableJoint`, leaks nothing, and neither the scene broadcaster nor the user-commands system matters. So it is in the physics side of gz-sim 8 / gz-physics 7.8 / DART, not in the project's plugin. The cube changes 26 welds at every change of axis: 1.2–1.4 MB each time (bench: +36 and +41 MB over 30 changes, +0 over 60 turns on one axis; 740 MB after about 400 changes). A solve needs a few tens of changes, so a long benchmark relaunches the simulation between sets of cubes and records the server's memory.

### Phase 7 cell — 2026-11-14

No package installed. `ros2 launch openfactory_bringup rubik.launch.py rviz:=false` on the VM display: the cell's controllers active, `move_group` ready, `cube_perception` publishing at 2.4 Hz; 1.4 GiB used, 2.5 GiB available at launch; Gazebo server 330–415 MB with a cube in the nest. The original cell (`cell.launch.py`) launches as before after `simulation.launch.py` and `move_group.launch.py` took the xacro and SRDF names as arguments.

- **Reach** (MoveIt collision-aware IK, nest at world (0.50, 0, 0.83)): the grasp from above with the fingers along x or y, either way round; the cube set down with the tool horizontal pointing back at the robot (−x) or along +y. Not reachable: the tool pointing away from the robot (+x), and only odd configurations along −y. Flips `over` and `side` of D-024 come from this.
- **Contact solver.** DART's default (Dantzig) puts the squeeze of a finger on one contact point per pad: a cube held by flat pads turns 17–22° between them when the tool tilts. With `<dart><solver><solver_type>pgs` in the world the grip is rigid, but loaded joints creep by microradians a step, the fingers sink 0.3 mm into the cube, and the cube mechanism failed to change axis under load. The cell keeps Dantzig; the gripper got ribs instead (D-024).
- **Rendering.** Sticker colours are the model's times 1.1, flat shaded. Stickers 0.4 mm proud of the body went missing in the camera image, one or two per view (depth buffer of Ogre 1); none at 0.8 mm and 1.0 mm in eight views.
- **Times** (wall, real-time factor about 1): a turn 9–12 s, a flip 13–20 s (more when the cube is turned in the air first), the scan of six faces 81 s of simulated time for five flips.
- **Memory.** A sequence of 40 flips and turns made 228 changes of axis and took the Gazebo server from 330 to 734 MB, most of them from knocks (the cube landing, the fingers closing). Since a change of axis needs 25 steps of demand in a row, a scan of six faces (five flips) makes none: 415 MB before and after.
- **Trap, found during the closing test:** `scripts/vm/stop_cell.sh` did not know `cube_perception`, and the node outlives its launch: after 26 relaunches of the cell (parts B to D) 26 of them were still running, each handling `/clock`: load average 31, 0.9 GiB available, the simulation at 0.44 of real time. Killed by PID and the script fixed on 2026-11-14 during the first cube of part E; the results are counted in simulated time and are not affected, the wall times of part D are (32 s per flip instead of 15–20). Any new node of a launch must be added to that script.
- **Traps:** a harness script that crashes with the cube between the fingers leaves it there, and the next spawn puts a second cube under it; start a probe by opening the gripper, going to ready and removing the cube. `ps` shows the Gazebo server as `ruby` with `gz sim` in its arguments.

### Phase 7 solver installation — 2026-11-14

Approved by the user on 2026-11-14 with this command, run as written:

```bash
# [VM]
python3 -m venv ~/openfactory_ws/.venv_solver
~/openfactory_ws/.venv_solver/bin/pip install -r /mnt/openfactory-ai/requirements-vm.txt
```

Installed in the venv only: `kociemba` 1.2.1 (a C extension built through `cffi`; `python3-dev` and `python3-venv` 3.12.3 were already there), `cffi` 2.1.1, `future` 1.0.0, `pycparser` 3.11; pip 24.0; 47 MB. Nothing in the system Python, in apt or in the `doppelbot` venv changed; 13 GB of disk free afterwards.

How the cell finds it: the path of the venv's site-packages is written once in `~/openfactory_ws/solver_path` (`/home/nico/openfactory_ws/.venv_solver/lib/python3.12/site-packages`) and `scripts/vm/run_rubik.sh` puts it on `PYTHONPATH`. For `ros2 run openfactory_manipulation rubik` by hand: `export PYTHONPATH=$(cat ~/openfactory_ws/solver_path):$PYTHONPATH` after sourcing ROS and the workspace. The venv is never activated, so `colcon` and the ROS Python are not affected (the trap `doppelbot` met with its venv does not arise).

Check: `test_rubik_cube.py` with the path set, 61 passed, the solver test among them (20 random scrambles solved in at most 25 moves, an impossible state refused). Without the path the same test is skipped and the cell ends as `solver_failed`.
