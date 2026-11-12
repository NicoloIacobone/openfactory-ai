# OpenFactory AI

A ROS 2 digital twin of a robotic cell that **perceives, plans, acts and recovers in a closed loop**: a simulated RGB-D camera observes unordered objects, the system chooses a feasible grasp, plans a collision-aware motion, picks and places the object, verifies the outcome and replans after failures. Inspired by industrial 3D-vision bin-picking platforms (e.g. Mujin).

The first deliverable is an **autonomous random bin-picking cell**; it is designed to grow into a production line and later a multi-robot factory.

> Status: **phase 5, AI and robustness, is closed; phase 6, factory expansion, is next.** The cell empties on its own an input bin holding five cubes, cylinders and prisms at random poses, now including cylinders lying on their side and prisms standing on an end face, which its geometric perception does not recognise. It grasps those as clusters of unknown shape through the same hard filters (gripper geometry, walls, neighbours, then MoveIt IK and straight-line checks), and ranks the feasible grasps with a logistic model of grasp success trained on 320 simulated attempts collected in random order. On 40 held-out scenes the learned ranking succeeded on 198 of 203 attempts against 195 of 212 for the hand-designed score on the same scenes (paired difference +5.6 points, bootstrap 95 % interval +2.2 to +9.2), deposited 199 of 200 objects against 195, with 0 executed states in collision and every detected failure followed by an autonomous recovery (56 of 56 in all phase 5 runs). The phase 4 cell, which leaves unknown objects alone, deposits 128 of those 200 (`docs/experiments.md` E-005, which also reports the failures and the limits). Known limits: objects separate, never touching or piled; top-down grasps only, so an object close to a wall on the side the fingers close from is left; the ranking orders feasible grasps and cannot refuse one, so when the only feasible grasp is a bad one it is still tried; these scenes have one dominant failure mode, which the model found from data but a hand rule could also encode once known. Phase 4: 99/100 objects on 20 seeded scenes of upright objects, 0 collisions (E-004). Earlier phases: camera-driven pick of one object 28/30 with a geometric perception pipeline at 0.2 mm median error in simulation (E-003), 30/30 pick-and-place cycles at known poses (E-002), 10/10 collision-free reach goals (E-001).

## Roadmap

| Phase | Goal | Deliverable |
|---|---|---|
| 0 | Environment audit | Environment notes and documented stack choice |
| 1 | Robot simulation | Supported arm in the simulator and MoveIt 2 |
| 2 | Deterministic pick/place | Full non-AI manipulation cycle at a known pose |
| 3 | Camera and 3D perception | Camera-driven object selection, documented calibration and frames |
| 4 | Random bin picking | Autonomous MVP with metrics |
| 5 | AI and robustness | Baseline vs learned grasp scoring, recovery, quantitative experiments |
| 6 | Factory expansion | Conveyor, inspection, scheduling, second robot |

Each phase is gated by a stable, measured previous phase.

## MVP Definition of Done

- Robot motion goes through ROS 2 and a real planning/execution pipeline, not hard-coded animation.
- At least three object types are picked from randomized poses.
- The motion planner respects robot and scene geometry (MoveIt 2 if supported; any alternative documented).
- Perception consumes camera data; simulator ground truth is never its input.
- Failed attempts are detected and excluded from success metrics.
- Trials run automatically with reproducible randomization and stored logs/metrics.

## Repository layout

| Path | Contents |
|---|---|
| `src/` | ROS 2 packages `openfactory_*` (created as each responsibility appears) |
| `scripts/` | Scene randomization, benchmark runners, ground-truth evaluation |
| `tests/` | Unit, integration and regression tests |
| `docs/architecture.md` | Components, interfaces, frames |
| `docs/decisions.md` | Stack and design decisions with rationale |
| `docs/experiments.md` | Trials, benchmarks, results |
| `docs/environment.md` | Audited environment and setup |

## Setup

Tested on Ubuntu 24.04 aarch64 with ROS 2 Jazzy (details and versions: `docs/environment.md`).

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-moveit ros-jazzy-ur-simulation-gz ros-jazzy-ur-moveit-config
source /opt/ros/jazzy/setup.bash
mkdir -p ~/openfactory_ws && cd ~/openfactory_ws
colcon build --symlink-install --base-paths <repo>/src && source install/setup.bash
colcon test --base-paths <repo>/src && colcon test-result --verbose
python3 -m pytest -q <repo>/tests
```

On GPUs limited to OpenGL 2.1 (e.g. the UTM virtio GPU) Gazebo uses the Ogre 1 render engine (`docs/decisions.md`, D-005); the launch files already select it.

## Run

```bash
# Cell: Gazebo (headless by default) with the RGB-D camera, MoveIt 2 move_group, perception, RViz
ros2 launch openfactory_bringup cell.launch.py              # headless:=false for the Gazebo GUI, rviz:=false without RViz

# Phase 1 reach test (in another terminal, with the cell running): 10 seeded goals, logs in ~/openfactory_data/trials/
python3 <repo>/scripts/phase1_reach_test.py --seed 1

# One pick-and-place cycle: spawn an object at a known pose in the input bin, then pick and place it
python3 <repo>/scripts/gz_scene.py spawn cube
ros2 run openfactory_manipulation pick_place_once --object cube

# Phase 2 closing test (cell launched with headless:=true rviz:=false): 10 cycles per object
python3 <repo>/scripts/phase2_pick_place_test.py

# Camera-driven pick: spawn a seeded scene, look at what perception reports, pick the best hypothesis
python3 <repo>/scripts/randomize_scene.py --seed 1 --index 0 --objects 1 --spawn
ros2 topic echo --once /perception/objects
ros2 run openfactory_manipulation pick_perceived_once

# Phase 3 closing test: calibration check, perception accuracy on 44 seeded scenes, 30 camera-driven picks
python3 <repo>/scripts/phase3_camera_check.py
python3 <repo>/scripts/phase3_perception_test.py
python3 <repo>/scripts/phase3_camera_pick_test.py

# Bin picking: spawn five objects at seeded random poses, then let the cell empty the bin on its own
python3 <repo>/scripts/randomize_scene.py --seed 1000 --index 0 --objects 5 --wall-margin 0.008 --min-gap 0.010 --spawn
ros2 run openfactory_manipulation bin_picking

# Phase 4 closing test: 20 seeded scenes of five objects and 2 empty bins, judged from ground truth
python3 <repo>/scripts/benchmark.py                      # --set dev | confirm | confirm2 for the other scene sets

# Phase 5: tumbled objects, strategies compared on the same held-out scenes
python3 <repo>/scripts/randomize_scene.py --seed 51 --index 0 --objects 5 --wall-margin 0.008 --min-gap 0.010 --tumbled-share 0.5 --spawn
python3 <repo>/scripts/benchmark.py --config <repo>/scripts/config/phase5_tumbled.yaml --set eval --strategy learned   # | baseline | catalog_only
python3 <repo>/scripts/benchmark.py --config <repo>/scripts/config/phase5_tumbled.yaml --set train --strategy explore  # training data
python3 <repo>/scripts/train_grasp_ranker.py <train run dir> --out grasp_ranker.json                                  # batch, NumPy, seconds
python3 <repo>/scripts/compare_strategies.py <run dir> <run dir> ...                                                  # Wilson intervals, scene bootstrap
```

The cell launch also starts the RGB-D camera, its depth error model and `scene_perception`. The camera renders on a display even with `headless:=true`; `camera:=false perception:=false` runs the physics-only cell without one, and `depth_noise:=false` gives ideal depth.
