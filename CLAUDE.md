# CLAUDE.md — openfactory-ai

Instructions for the coding assistant. **Reply to the user in Italian**; write code, comments, commit messages and docs in English.

OpenFactory AI is a ROS 2 digital twin of a robotic cell that perceives a changing scene, plans collision-free actions, executes them in simulation and recovers from failures. Long term: a production line, then a multi-robot factory.

**Current scope: a single-arm autonomous random bin-picking MVP.** One 6-DoF arm with a parallel-jaw gripper, one fixed RGB-D camera, one input bin with randomized rigid objects (cubes, cylinders, prisms), one fixed destination bin. No conveyors, scheduling, second robot or factory manager until the MVP Definition of Done (`README.md`) is met with logged evidence.

## Status

| Phase | Goal | State |
|---|---|---|
| 0 | Environment audit, stack choice | closed 2026-10-28: `docs/environment.md`, D-005 (Gazebo sensors need Ogre 1) |
| 1 | Supported arm in the simulator + MoveIt 2 | closed 2026-10-29: E-001 reach test 10/10, D-004 (UR5e), D-006 (cell layout) |
| 2 | Deterministic pick/place at a known pose | next |
| 3 | RGB-D camera, TF, calibration, geometric perception | — |
| 4 | Random bin picking MVP with metrics | — |
| 5 | AI and robustness: learned ranking vs baseline, recovery | — |
| 6 | Factory expansion | — |

A phase starts only after the previous one is closed, and closes only when its measurable test passes. When that happens, propose updating this table and `README.md`.

## Non-negotiable rules

1. **Closed loop, never animation.** Robot motion goes through ROS 2 planning and controllers into the simulator. No scripted joint playback, no teleported objects, no visual-only demos.
2. **Perception never commands joints.** It publishes object hypotheses and grasp candidates; planning and execution validate and act.
3. **Baseline before AI.** Deterministic pick/place (phase 2) and geometric perception/grasping (phases 3–4) must work and be measured before any learned module, even "just to try".
4. **Feasibility and safety are hard filters.** IK, reachability, collision and trajectory checks reject candidates *before* quality ranking. Planning is always collision-aware; MoveIt 2 is preferred. Another planner only with a reason recorded in `docs/decisions.md`, and it must still be collision-aware.
5. **No ground-truth leakage.** Simulator object poses are for evaluation and debug labels only. Perception and grasp selection consume camera data (RGB, depth, `camera_info`) and TF, never ground truth. Ground-truth topics live under `/ground_truth/` and nothing in perception or manipulation subscribes to them.
6. **Reproducible.** Scene randomization is seeded and versioned; trials write structured logs and metrics. A result that cannot be re-run from its seed and config does not count.
7. **Each phase leaves a runnable system.** Never add complexity to hide an unreliable single-cell pipeline.

## Environment

| Where | What |
|---|---|
| Mac (MacBook Air M2, 8 GB) | Source of truth for the repo, git, editor. No ROS. |
| VM (UTM 5.0.8, Ubuntu 24.04 aarch64, 3.8 GB RAM, 4 cores) | ROS 2 Jazzy, colcon, simulator, RViz. Same VM as the `doppelbot` project; SSH alias `doppelvm`. |

- **Audit before changing.** The stack in `docs/environment.md` is chosen from evidence. Before installing or changing anything in the VM, read it; if the environment looks different, re-audit and update it. Never install, upgrade or migrate only to match a document. Never propose an Ubuntu release upgrade or a ROS distro migration.
- **Memory is the main constraint.** One robot, small world, primitive objects, downsampled point clouds, headless simulator when the GUI is not needed, RViz only when needed. No heavy model training locally; keep batch benchmarks separate from interactive simulation. Prefer binary apt packages to source builds of large stacks.
- `ros2`, `colcon`, `apt`, `gz` and `rviz2` run only in the VM. Git runs only on the Mac.
- The repo lives on the Mac in `~/Desktop/nico/github_portfolio/ros_2_projects/openfactory-ai`. The VM mounts the UTM share `ros_2_projects` at `/mnt/ros_2_projects`; `/mnt/openfactory-ai` is a symlink into it. Details: `docs/environment.md`, "Repo access from the VM".

Commands in the VM over SSH (only commands that terminate: build, test, `ros2 topic echo --once`, `ros2 pkg list`):

```bash
ssh doppelvm 'source /opt/ros/jazzy/setup.bash && <command>'
```

`source` is always explicit: a non-interactive shell does not load `.bashrc`. GUI or long-running programs (simulator, RViz, launch files): give the user the command to run, with `2>&1 | tee ~/<name>.log` so the log can be read over SSH.

## Working with the user

- One phase at a time; do not anticipate later phases.
- Short plan first, then small patches. After each patch say which test to run, where (**[MAC]** or **[VM]**) and what the user should see.
- Label every proposed command **[MAC]** or **[VM]**.
- On an error: ask for the full command and output if missing, find the cause, propose the minimal fix and explain why.
- Install commands follow the official docs (ROS 2 Jazzy, Gazebo, MoveIt 2). If unsure about a command or version, say so.
- Ask before: installing packages, changing system configuration (Mac, VM, UTM), deleting files, changing an interface in `docs/architecture.md`, `git push`.
- Small, descriptive commits, one per logical change.
- Never in the repo: tokens, passwords, SSH keys, large binaries (bags, datasets, weights, meshes downloaded from third parties). Run outputs go in `data/` (git-ignored); the VM cannot write to the share, so it writes to `~/openfactory_data/` and the Mac copies with `scp`.

## Build and test

Standard colcon workflow, verified on the first packages (2026-10-29):

```bash
# [VM] build outside the shared folder
source /opt/ros/jazzy/setup.bash && cd ~/openfactory_ws
colcon build --symlink-install --base-paths /mnt/openfactory-ai/src && source install/setup.bash

# [VM] test
colcon test --base-paths /mnt/openfactory-ai/src && colcon test-result --verbose

# [VM] pure-logic tests of repo-level scripts
cd /mnt/openfactory-ai && python3 -m pytest -q -p no:cacheprovider tests
```

## Architecture in brief

Loop: RGB-D camera + robot state → `scene_perception` → `grasp_selector` → `task_planner` / MoveIt 2 → `robot_executor` → `scene_manager` (verify, update, recover) → back to perception. Components, interfaces and frames: `docs/architecture.md`.

- Packages under `src/` named `openfactory_<role>` (`description`, `simulation`, `moveit_config`, `perception`, `manipulation`, `bringup`). Create a package only when a concrete responsibility needs it.
- Standard messages first (`sensor_msgs`, `geometry_msgs`, `moveit_msgs`, `vision_msgs`). A custom `openfactory_interfaces` package only for a stable interface, after approval.
- Topic, frame and joint names as constants or ROS parameters, never scattered strings. Parameters in YAML under each package's `config/`.
- TF discipline: every pose carries `frame_id` and stamp; REP 103 (SI units, right-handed) and REP 105; camera data in `*_optical_frame`. `use_sim_time: true` for every node in simulation.
- Each package has its own launch files; `openfactory_bringup` composes them into the reproducible whole-system launch.
- Pure logic (geometry, grasp generation, metrics, log format) in ROS-free modules tested with pytest. Nodes stay thin.
- Real dependencies in `package.xml`; do not invent them. A new dependency is proposed and justified first.

## Docs to update with every change

| Change | Update |
|---|---|
| Topic, service, action, frame or component | `docs/architecture.md` |
| Stack, robot, planner, tool choice or assumption | `docs/decisions.md` (new entry) |
| Trial, benchmark, result | `docs/experiments.md` |
| VM or tool versions | `docs/environment.md` |

## Guides: read only the ones the task touches

These are plain links, not `@` imports, so they are **not** loaded automatically: open the matching guide with Read before working in that area. Do not turn them into `@` imports (they would load in every session). Keep this table in sync when guides are added, renamed or removed.

| Task touches | Read |
|---|---|
| Packages, colcon, launch, parameters, TF, nodes, ROS tests | `docs/agent-guides/ros2-workflow.md` |
| Simulator, worlds, models, sensors, ros_gz bridge, controllers, randomization | `docs/agent-guides/simulation.md` |
| Camera, calibration, point clouds, segmentation, pose estimation | `docs/agent-guides/perception.md` |
| Grasps, MoveIt 2, planning scene, gripper, execution, recovery | `docs/agent-guides/manipulation.md` |
| Trials, logs, metrics, benchmarks, comparisons, learned models | `docs/agent-guides/benchmarking.md` |
