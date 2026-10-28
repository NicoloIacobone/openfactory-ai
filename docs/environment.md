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

### Not yet verified

- Gazebo running: a minimal world, headless and with GUI; an RGB-D sensor rendering through the virtio GPU; OpenGL version (`glxinfo`).
- RAM with simulator + MoveIt 2 + RViz running together (likely the binding constraint on 3.8 GiB).
- Open3D via pip on aarch64 / Python 3.12.

## Selected stack (provisional)

| Area | Choice | Why |
|---|---|---|
| OS / ROS | Ubuntu 24.04 + ROS 2 Jazzy, as installed | Already working and healthy; no migration needed |
| Simulator | Gazebo Harmonic (Sim 8.15) via `ros-jazzy-ros-gz` (installed, not yet run) | Official Jazzy pairing, binary on aarch64 |
| Planning | MoveIt 2 2.12.4 (installed, not yet run) | Binary on aarch64 |
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

Status: verified 2026-10-28 (mount, symlinks, `doppelbot` build and 214 tests pass through the new paths); original fstab kept as `/etc/fstab.bak`.
