# Environment

Audited environment and the stack selected from it. Re-audit and update this file whenever the VM or tool versions change.

## Audit — 2026-10-28 (phase 0, first pass)

Read-only inspection of the existing VM; nothing installed or changed.

### Host

| Item | Value |
|---|---|
| Mac | MacBook Air M2, 8 GB RAM |
| Virtualizer | UTM 5.0.8 (QEMU backend), "Shared Network" |
| Existing share | 9p/VirtFS tag `share` → `/mnt/doppelbot` (the `doppelbot` project); only one share is possible |

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

### Not yet verified

- Gazebo installed and running: a minimal world, headless and with GUI; an RGB-D sensor rendering through the virtio GPU.
- RAM with simulator + MoveIt 2 + RViz running together (likely the binding constraint on 3.8 GiB).
- Open3D via pip on aarch64 / Python 3.12.
- Repo access from the VM (below).

## Selected stack (provisional)

| Area | Choice | Why |
|---|---|---|
| OS / ROS | Ubuntu 24.04 + ROS 2 Jazzy, as installed | Already working and healthy; no migration needed |
| Simulator | Gazebo Harmonic via `ros-jazzy-ros-gz` (candidate) | Official Jazzy pairing, binary on aarch64 |
| Planning | MoveIt 2 2.12 (candidate) | Binary on aarch64 |
| Robot | open until phase 1; UR5e is the strongest candidate | Full apt stack (description, Gazebo sim, MoveIt config) exists for Jazzy; Franka would need a source build |

Final choices are recorded in `docs/decisions.md` once installation and a phase-1 test confirm them.

## Repo access from the VM

The Mac is the source of truth. UTM 5.0.8 (QEMU backend) offers a single VirtFS shared folder, so both projects live in `~/Desktop/nico/ros_2_projects/` (`doppelbot/`, `openfactory-ai/`) and that folder is the share:

| Mac | VM |
|---|---|
| `~/Desktop/nico/ros_2_projects` | `/mnt/ros_2_projects` (9p, tag `share`, in `/etc/fstab`) |
| `~/Desktop/nico/ros_2_projects/openfactory-ai` | `/mnt/openfactory-ai` → symlink to `/mnt/ros_2_projects/openfactory-ai` |
| `~/Desktop/nico/ros_2_projects/doppelbot` | `/mnt/doppelbot` → symlink to `/mnt/ros_2_projects/doppelbot` |

The colcon workspace lives outside the share, at `~/openfactory_ws` in the VM.

Status: folders moved on the Mac 2026-10-28; UTM share and VM mount not verified yet.
