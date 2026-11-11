# Experiments

Summaries of trials and benchmarks. Raw logs stay in `data/` (git-ignored); every entry names the run id, git commit, config and seed list so it can be re-run. Method and metrics: `docs/agent-guides/benchmarking.md`.

## Template

```
## E-NNN — <title> (<date>)

- Question:
- Strategies:
- Seeds: <tuning set> / <evaluation set>   Objects: <mix>   Trials: N
- Commit / config:
- Results: table with rates (95% CI), cycle time, failures by stage
- Conclusion:
```

## E-001 — Phase 1 closing test: MoveIt reach in Gazebo (2026-10-29)

- Question: does the cell plan collision-free motions with the table in the scene and execute them through ROS 2 control in Gazebo, ending within tolerance?
- Strategies: `moveit_joint_goal` (OMPL RRTConnect via `move_group`, `joint_trajectory_controller`, velocity/acceleration scaling 0.3).
- Seeds: 1   Goals: 10 joint goals sampled in `scripts/config/phase1_reach.yaml` ranges, accepted only if MoveIt reports them collision-free and `tool0` is ≥ 0.10 m above the table top.
- Commit / config: `5937ab1`, `scripts/config/phase1_reach.yaml` (sha256 `ba74ce5fd846`); run `phase1_reach_20261029T104240Z_s1`, logs in `data/trials/` (copied from the VM's `~/openfactory_data/trials/`). Cell: `ros2 launch openfactory_bringup cell.launch.py` (Gazebo headless, RViz on).
- Re-run: `python3 scripts/phase1_reach_test.py --seed 1` in the VM with the cell launched.

| Metric | Value |
|---|---|
| Goals planned and executed (MoveIt `SUCCESS`) | 10/10 |
| Final max \|goal − final\| over joints | max 0.00099 rad, mean 0.00083 rad (pass ≤ 0.01) |
| Executed states re-checked by MoveIt (`/joint_states` every 0.1 s) | 412, 0 in collision |
| Goal `tool0` height above floor | 0.89–1.69 m (table top 0.75 m) |
| Time per goal (plan + execute, wall) | mean 4.1 s, range 3.1–6.3 s |
| RAM (Gazebo server + MoveIt + RViz + GNOME) | 1.4 GiB used, 2.4 GiB available |

- Limits: `planning_time` in the `MoveGroup` result is 0 for plan-and-execute requests, so planning and execution time are not split (logged `execution_s` equals `total_s`). The floor is not in the MoveIt scene; the bare arm cannot reach it (lowest `tool0` behind the table ≈ 0.07 m above the floor), but it must be added with the gripper in phase 2. Gazebo self-reported real-time factor ≈ 1.0.
- Conclusion: phase 1 measurable test passes.

## E-002 — Phase 2 closing test: deterministic pick and place at known poses (2026-11-10)

- Question: does the cell pick each MVP object type from a known pose in the input bin and place it in the output bin through MoveIt 2 and the controllers, collision-free, with failures detected from more than one source?
- Strategy: `known_pose_top_down` (`openfactory_manipulation`, D-010): OMPL free moves to ready and pre-grasp, straight `tool0` lines for approach, lift, transfer, descend and retreat, friction grasp with the parallel gripper (D-007), object attached in the planning scene while held.
- Objects and known poses (`input_bin` frame): cube (0, 0, yaw 0), cylinder (0.05, 0.03, 0), prism (−0.04, −0.02, 0.5). Seed 1 (no randomness in phase 2). Trials: 10 per type, interleaved, 30 in total.
- Success: manipulation reports every stage OK (MoveIt `SUCCESS`, jaw opening within 4 mm of the object width after close, lift and transfer) **and** ground truth shows the object resting upright in the output bin **and** no executed state (sampled from `/joint_states` every 0.1 s, arm + fingers) is in collision for MoveIt.
- Commit / config: `70d9d44`, `scripts/config/phase2_pick_place.yaml` (sha256 `2abd09041321`), `openfactory_manipulation/config/pick_place.yaml`; run `phase2_pick_place_20261110T191532Z_s1`, logs in `data/trials/` (copied from the VM). Cell: `ros2 launch openfactory_bringup cell.launch.py headless:=true rviz:=false`.
- Re-run: `python3 scripts/phase2_pick_place_test.py` in the VM with the cell launched.

| Object | Success (95% Wilson CI) | Cycle time median (min–max) | Place error mean / max |
|---|---|---|---|
| cube | 10/10 (0.72–1.00) | 18.6 s (18.6–21.5) | 8.9 / 9.7 mm |
| cylinder | 10/10 (0.72–1.00) | 18.1 s (18.0–18.1) | 17.4 / 17.5 mm |
| prism | 10/10 (0.72–1.00) | 18.4 s (18.4–18.5) | 3.4 / 11.8 mm |
| **all** | **30/30 (0.89–1.00)** | **18.4 s** | **9.9 / 17.5 mm** |

| Metric | Value |
|---|---|
| Failures by reason / stage | none |
| Planning time per cycle (sum of plans) | mean 0.12 s (0.09–0.17) |
| Execution time per cycle | mean 18.3 s (17.8–21.3) |
| Perception time | — (no perception in phase 2) |
| Executed states re-checked | 5490, 0 in collision |
| Jaw opening after transfer | cube 49.8–49.9 mm, cylinder 50.0 mm, prism 60.0 mm (widths 50 / 50 / 60 mm) |
| RAM (Gazebo server + MoveIt, no RViz, GNOME) | 1.3 GiB used, 2.5 GiB available |

- Limits: one known pose per object type; no candidate search (`candidates_total` 1); the collision re-check covers arm and gripper against fixtures, not the held object (removed from the planning scene before the check). Place error is systematic along the finger axis (release pushes the object, and the mimic finger lets it sit off-centre in the jaws); harmless for the bin target, relevant if precise placement is ever needed. The prism is a box in the planning scene.
- Development runs before this one (not counted): the first harness smoke run failed 2/3 with false `grasp_slipped` from a one-finger hold check; fixed by checking the jaw opening (D-010).
- Conclusion: phase 2 measurable test passes; this is the known-pose baseline that camera-driven picking will be compared against.

## E-003 — Phase 3 closing test: camera calibration, perception accuracy, camera-driven pick (2026-11-11)

- Question: does the cell locate and recognise objects in the input bin from the RGB-D camera and TF alone, is the camera calibration what the description says, and does a pick driven by that perception work as well as the known-pose baseline (E-002)?
- Strategies: perception `geometric_height_map` (D-012); pick `camera_top_down` (D-013): the most confident hypothesis replaces the known pose of `known_pose_top_down`, same cycle (D-010).
- Camera: D415-like depth stream with the depth error model on, seed 1 (D-011). Thresholds were written in the configs before the first live run; perception parameters were not changed after the development seeds.
- Seeds: tuning set seed 1000 (10 scenes + 2 empty; 6 picks) / evaluation set seed 1. Objects: cube, cylinder, prism, separate, resting in their catalog orientation, random position and yaw.
- Commit / config: `ae9d454`; `scripts/config/phase3_perception.yaml` (sha256 `b57ff41da49d`), `scripts/config/phase3_camera_pick.yaml` (sha256 `946aa64d8144`), `openfactory_perception/config/perception.yaml`, `openfactory_manipulation/config/pick_place.yaml`. Cell: `ros2 launch openfactory_bringup cell.launch.py headless:=true rviz:=false` from the VM desktop. Logs in `data/trials/` (copied from the VM).
- Re-run, in the VM with the cell launched: `python3 scripts/phase3_camera_check.py`, `python3 scripts/phase3_perception_test.py`, `python3 scripts/phase3_camera_pick_test.py`.

### Part A — calibration (`phase3_camera_check_20261111T102320Z`): pass

| Check | Measured | Limit |
|---|---|---|
| `camera_info` fx, fy against `camera.yaml` (502.30 px) | relative error 0 | 0.5 % |
| Principal point against the image centre (320, 180) | 0 px | 1 px |
| Image size, frame of `camera_info` and depth | 640 × 360, `camera_optical_frame` | — |
| Bin floor (28 743 pixels, ideal depth, through TF) against z = 0 of `input_bin` | offset 0.00002 mm, tilt 0.0002°, RMS 0.00002 mm | 2 mm, 0.5° |
| Known point: cube top-face centre from ground truth, projected with TF and K, against where the face is seen (3 poses) | 0.00, 0.17, 0.12 px | 2 px |
| Depth at the known point | below 0.001 mm | 2 mm |
| Depth noise on the bin floor against the model (5 frames) | 1.450 mm against 1.448 mm (ratio 1.002) | ± 25 % |

Without perception's principal-point correction the known point is seen (−0.50, −0.50) px from its projection at the image centre: Gazebo's pixel (u, v) looks along the ray of (u + 0.5, v + 0.5). The correction is in `perception.yaml` (D-011); the reprojection figures above include it.

### Part B — perception accuracy against ground truth: pass

44 scenes: 14 with one object, 13 with two, 13 with three, 4 empty; 79 objects. One depth frame per scene, the first stamped after the objects had settled.

| Metric | Depth model on (`phase3_perception_eval_20261111T102341Z_s1`) | Ideal depth (`phase3_perception_eval_ideal_20261111T102443Z_s1`) | Pass |
|---|---|---|---|
| Recall (95% Wilson CI) | 79/79 (0.95–1.00) | 79/79 | ≥ 0.95 |
| Precision | 79/79 (0.95–1.00) | 79/79 | ≥ 0.95 |
| Correct class | 79/79 | 79/79 | ≥ 0.95 |
| Position error xy, median / 95th percentile / max | 0.22 / 0.68 / 1.19 mm | 0.06 / 0.65 / 0.79 mm | ≤ 3 / ≤ 8 mm |
| Yaw error modulo symmetry (cube, prism; n = 50), median / 95th percentile / max | 0.13 / 0.85 / 1.13° | 0.01 / 0.30 / 0.95° | ≤ 3 / ≤ 8° |
| Detections in the 4 empty bins | 0 | 0 | 0 |
| Confidence of the detections, median (min–max) | 0.95 (0.83–1.00) | 0.94 (0.80–0.99) | — |
| Image stamp to hypotheses received | median 20 ms, max 30 ms | median 17 ms | — |

| Object (depth model on) | n | Class | xy median / max | Yaw median / max | Confidence |
|---|---|---|---|---|---|
| cube | 24 | 24 | 0.08 / 0.31 mm | 0.07 / 0.35° | 0.90–0.98 |
| cylinder | 29 | 29 | 0.41 / 0.70 mm | — | 0.83–0.93 |
| prism | 26 | 26 | 0.26 / 1.19 mm | 0.23 / 1.13° | 0.88–1.00 |

### Part C — pick and place from the camera hypothesis (`phase3_camera_pick_20261111T102553Z_s1`): pass

30 attempts, 10 per type, interleaved; object centre drawn in the middle of the bin (±75 mm × ±35 mm, D-013), any yaw. Success as in E-002: every manipulation stage OK **and** ground truth shows the object resting upright in the output bin **and** no executed state in collision.

| Object | Success (95% Wilson CI) | Cycle time median (min–max) | Place error mean / max |
|---|---|---|---|
| cube | 10/10 (0.72–1.00) | 18.2 s (17.4–18.8) | 10.2 / 17.3 mm |
| cylinder | 10/10 (0.72–1.00) | 18.9 s (17.8–20.0) | 14.0 / 17.5 mm |
| prism | 8/10 (0.49–0.94) | 18.3 s (17.1–19.3) | 6.2 / 11.9 mm |
| **all** | **28/30 (0.79–0.98)**, pass ≥ 27 and ≥ 8 per type | **18.4 s** | **10.4 / 17.5 mm** |

| Metric | Value |
|---|---|
| Failures by reason | `place_failed` 2 (both prism), no failed stage |
| Selected hypothesis against ground truth | class 30/30; xy median 0.07 mm, max 0.55 mm; yaw median 0.06°, max 0.87°; confidence median 0.95 |
| Perception time per cycle (asking for a view to object chosen) | median 0.15 s (0.03–0.36) |
| Planning time per cycle | median 0.12 s |
| Execution time per cycle | median 18.1 s (16.9–19.7) |
| Executed states re-checked | 5686, 0 in collision |
| Attempts the manipulation reported as done while ground truth disagrees | 2 |
| RAM (Gazebo server + MoveIt + depth model + perception, no RViz) | 1.5 GiB used, 2.3 GiB available, before and after |

- The two failures are not perception errors (selected pose within 0.08 mm and 0.3 mm of ground truth). Both prisms ended inside the output bin, lying on another of their three rectangular faces (object frame 11 mm above the floor instead of 22 mm: a 120° roll), which the success criterion rejects as not upright. A replay of the two scenes with the object pose sampled from the simulator showed the mechanism: during the transfer the prism turns between the pads about the finger axis (4° in one replay, up to 42° in the other, then hanging at 14°) and at release it drops back upright or onto the next face. In the replays both landed upright, so the outcome is not repeatable. The jaw opening does not change when the prism turns, so the grasp and hold checks cannot see it: the manipulation reported success both times and only the ground-truth evaluation caught it.
- Comparison with the known-pose baseline (E-002, 30/30, median cycle 18.4 s): same cycle time, perception adds 0.15 s. E-002 used one pose per type (prism yaw 0.5 rad) and did not expose the prism turning in the grasp; the random yaw here did.
- Limits: separate objects in their catalog orientation only; one frame per scene; the depth error model is per-pixel noise on ideal depth, not simulated stereo (D-011), so these accuracies are an upper bound for a real D415; pick region limited to the middle of the bin, one grasp per object, no retry; the prism passes at its threshold (8/10) with a wide interval; perception sees the arm if it is inside the bin volume, so views are taken at ready only.
- Development runs before these (not counted, code not yet committed): `phase3_perception_dev_20261111T101505Z_s1000` 19/19 objects, xy median 0.9 mm before the principal-point correction; `phase3_camera_pick_20261111T101647Z_s1000` 6/6; `phase2_pick_place_20261111T101530Z_s1` 3/3 (E-002 cycle with the camera body as a MoveIt obstacle, 0 of 578 states in collision). The first calibration run failed its reprojection check at 15 px because the check script counted the bin-wall pixels at the cube's top height; fixed in the script, not in the camera.
- Open for phase 4: a held prism can turn about the finger axis during the transfer (grasp point, transfer acceleration or a place check from the camera); verification of the output bin by perception, so the cell itself detects a wrong place; grasp candidates near the walls; clutter.
- Conclusion: phase 3 measurable test passes on all three parts.
