# OpenFactory AI

A ROS 2 digital twin of a robotic cell that **perceives, plans, acts and recovers in a closed loop**: a simulated RGB-D camera observes unordered objects, the system chooses a feasible grasp, plans a collision-aware motion, picks and places the object, verifies the outcome and replans after failures. Inspired by industrial 3D-vision bin-picking platforms (e.g. Mujin).

The first deliverable, an **autonomous random bin-picking cell**, is done. Next come cells with several arms (`docs/ideas.md`), then a production line and a multi-robot factory.

## Status

**Phase 5, AI and robustness, is closed; phase 6, multi-arm cells, is in progress:** the relay loop runs as a ring of N arms and N bins, N chosen at launch (`docs/architecture.md`, "Ring of N arms"); its closing test is E-006.

A UR5e with a parallel-jaw gripper empties on its own an input bin holding five cubes, cylinders and prisms at random poses, including cylinders lying on their side and prisms standing on an end face, which its geometric perception does not recognise. It grasps those as clusters of unknown shape through the same hard filters (gripper geometry, walls, neighbours, then MoveIt IK and straight-line checks), and ranks the feasible grasps with a logistic model of grasp success trained on 320 simulated attempts.

| Phase | Test | Result |
|---|---|---|
| 1 | 10 seeded joint goals, planned with MoveIt 2 and executed in Gazebo | 10/10, 0 states in collision (E-001) |
| 2 | Pick and place at known poses, 10 per object type | 30/30 (E-002) |
| 3 | Calibration, geometric perception against ground truth, camera-driven pick | 79/79 objects, 0.2 mm median position error; 28/30 picks (E-003) |
| 4 | Bin picking, 20 seeded scenes of five upright objects | 99/100 objects, 0 collisions; 99/100 and 97/100 on two confirmation seeds (E-004) |
| 5 | Tumbled objects; learned ranking vs hand-designed score on the same 40 held-out scenes | pick success 198/203 vs 195/212 (paired difference +5.6 points, bootstrap 95 % interval +2.2 to +9.2); 199/200 vs 195/200 objects deposited, against 128/200 for the phase 4 cell; recovery 56/56; 0 collisions (E-005) |

Every result in `docs/experiments.md` gives its run ids, commit, config, seeds, failures and the command that re-runs it.

Known limits:

- Objects are separate, never touching or piled.
- Top-down grasps only: an object close to a wall on the side the fingers close from is left.
- The ranking orders feasible grasps and cannot refuse one: when the only feasible grasp is a bad one, it is still tried.
- The phase 5 scenes have one dominant failure mode, which the model found from data but a hand rule could also encode once known.
- One camera, above the input bin: the output bin and the held object are not observed.

## Roadmap

| Phase | Goal | Deliverable |
|---|---|---|
| 0 | Environment audit | Environment notes and documented stack choice |
| 1 | Robot simulation | Supported arm in the simulator and MoveIt 2 |
| 2 | Deterministic pick/place | Full non-AI manipulation cycle at a known pose |
| 3 | Camera and 3D perception | Camera-driven object selection, documented calibration and frames |
| 4 | Random bin picking | Autonomous MVP with metrics |
| 5 | AI and robustness | Baseline vs learned grasp scoring, recovery, quantitative experiments |
| 6 | Multi-arm cells | N arms passing objects around a ring of bins (conveyors later), then one arm scrambling a Rubik's cube and another solving it (`docs/ideas.md`) |
| later | Factory expansion | Conveyor-fed line, inspection, scheduling |

Each phase is gated by a stable, measured previous phase.

## MVP Definition of Done (met in phase 4, E-004)

- Robot motion goes through ROS 2 and a real planning/execution pipeline, not hard-coded animation.
- At least three object types are picked from randomized poses.
- The motion planner respects robot and scene geometry (MoveIt 2 if supported; any alternative documented).
- Perception consumes camera data; simulator ground truth is never its input.
- Failed attempts are detected and excluded from success metrics.
- Trials run automatically with reproducible randomization and stored logs/metrics.

## Repository layout

| Path | Contents |
|---|---|
| `src/` | ROS 2 packages `openfactory_*` (description, simulation, moveit_config, perception, manipulation, bringup) |
| `scripts/` | Scene randomization, benchmark runners, ground-truth evaluation, training of the learned ranking; `scripts/vm/` launches, stops and benchmarks detached in the VM |
| `tests/` | Pure-logic tests of the scripts |
| `docs/architecture.md` | Components, interfaces, frames |
| `docs/decisions.md` | Stack and design decisions with rationale |
| `docs/experiments.md` | Trials, benchmarks, results |
| `docs/environment.md` | Audited environment and setup |
| `docs/ideas.md` | Ideas for the next phases, before they have a plan |

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

# Bin picking: spawn five objects at seeded random poses (each cylinder or prism tumbled with
# probability 0.5), then let the cell empty the bin on its own
python3 <repo>/scripts/randomize_scene.py --seed 51 --index 0 --objects 5 --wall-margin 0.008 --min-gap 0.010 --tumbled-share 0.5 --spawn
ros2 run openfactory_manipulation bin_picking

# Benchmark: seeded scenes through the cell's own loop, judged from ground truth (E-005 evaluation set)
python3 <repo>/scripts/benchmark.py --config <repo>/scripts/config/phase5_tumbled.yaml --set eval --strategy learned
```

```bash
# Ring of N arms (phase 6): arm i picks from bin i and places into bin (i + 1) mod N, one arm at a time
ros2 launch openfactory_bringup ring.launch.py n:=3 rviz:=false
python3 <repo>/scripts/relay_benchmark.py --arms 3 --set dev    # spawns seeded objects in every bin, runs 20 handovers, judges from ground truth
ros2 run openfactory_manipulation relay --arms 3                # the loop alone, for ever, on objects already in the bins
```

The commands that reproduce each phase's results are in `docs/experiments.md` (E-001 to E-005). The cell launch also starts the RGB-D camera, its depth error model and `scene_perception`. The camera renders on a display even with `headless:=true`; `camera:=false perception:=false` runs the physics-only cell without one, and `depth_noise:=false` gives ideal depth.
