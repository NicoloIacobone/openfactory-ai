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

## E-004 — Phase 4 closing test: autonomous random bin picking (2026-11-11)

- Question: does the cell, on its own, empty an input bin holding several objects at random poses, close to each other and to the walls, into the output bin, with collision-aware motions, and does it know when a pick failed and when the bin is empty?
- Strategy: `geometric_top_down_ranked`: perception `geometric_height_map` (D-012, unchanged since E-003), grasp selection with hard filters and a baseline score (D-016), bin-picking loop that verifies each pick with the next view and remembers what it placed (D-017), gripper with long fingers (D-014).
- Scenes: 5 objects each, types drawn by the scene seed, resting in their catalog orientation, random position and yaw anywhere in the bin (footprint circles at least 8 mm from the walls and 10 mm from each other), plus 2 empty bins per set. The harness spawns a scene and never touches it again; the cell runs `BinPicking.step()` until it ends the scene itself.
- Success of an attempt: the cell reports it (every stage and finger check, and the object gone from the input bin in its next view) **and** ground truth shows that object at rest inside the output bin, on any face (D-017) **and** no executed state (`/joint_states` every 0.1 s, arm and fingers, re-checked by MoveIt) is in collision. Completion: objects deposited / objects spawned.
- Seeds: tuning seed 1000 (6 scenes + 1 empty) / evaluation seed 1 (20 + 2), fixed with the thresholds before any run; confirmation seeds 2 and 3 (20 + 2 each), added during the evaluation as described below. Camera depth model on, seed 1.
- Commit / config: `17df986`; `scripts/config/phase4_bin_picking.yaml` (sha256 `32f3cdb6ad03`), `openfactory_manipulation/config/pick_place.yaml`, `openfactory_perception/config/perception.yaml`. Cell: `ros2 launch openfactory_bringup cell.launch.py headless:=true rviz:=false` on the VM display. Logs in `data/trials/` (copied from the VM).
- Re-run, in the VM with the cell launched: `python3 scripts/benchmark.py` (evaluation set), `--set dev`, `--set confirm`, `--set confirm2`.

### Result with the final code

| Metric | Evaluation, seed 1 (`phase4_bin_picking_eval_20261111T194656Z_s1`) | Confirmation, seed 2 (`…confirm_20261111T191258Z_s2`) | Confirmation, seed 3 (`…confirm2_20261111T202034Z_s3`) | Pass |
|---|---|---|---|---|
| Completion (95% Wilson CI) | 99/100 (0.95–1.00) | 99/100 (0.95–1.00) | 97/100 (0.92–0.99) | ≥ 0.85 |
| Completion cube / cylinder / prism | 30/30, 36/36, 33/34 | 20/21, 48/48, 31/31 | 31/31, 34/35, 32/34 | each ≥ 0.80 |
| Pick success (attempts) | 99/99 (0.96–1.00) | 99/100 (0.95–1.00) | 97/98 (0.94–1.00) | ≥ 0.80 |
| Executed states in collision | 0 of 17 995 | 0 of 18 201 | 0 of 17 731 | 0 |
| Attempts the cell believed while ground truth disagrees | 0 | 0 | 0 | ≤ 2 |
| Failures the cell reported while the object did arrive | 0 | 0 | 0 | — |
| Detected failures followed by a return to ready with a view | none to recover | 1/1 | 1/1 | ≥ 0.90 |
| Attempts on the 2 empty bins | 0 | 0 | 0 | 0 |
| Scenes cleared; end reason correct | 19/20; 22/22 | 19/20; 22/22 | 17/20; 22/22 | — |
| Scene ends | 21 `bin_empty`, 1 `no_feasible_grasp` | 22 `bin_empty` | 20 `bin_empty`, 2 `no_feasible_grasp` | — |
| Failures by reason (stage) | none | 1 `grasp_slipped` (after transfer) | 1 `grasp_slipped` (after transfer) | — |
| Candidates; rejected by walls / neighbours / MoveIt | 1162; 173 / 73 / 0 | 1395; 277 / 79 / 0 | 1178; 242 / 40 / 0 | — |
| Other objects disturbed by a pick | 0 | 0 | 0 | — |
| Cycle time per attempt, median (min–max) | 18.1 s (15.4–20.9) | 18.3 s (15.5–20.9) | 18.2 s (15.7–20.6) | — |
| of which perception / planning with selection / execution, median | 0.78 / 0.13 / 17.1 s | 0.81 / 0.13 / 17.3 s | 0.73 / 0.12 / 17.3 s | — |
| Target seen by perception: class; xy error median / max | 99/99; 0.26 / 0.92 mm | 100/100; 0.33 / 0.80 mm | 98/98; 0.22 / 0.86 mm | — |
| Deposited objects resting upright (criterion of E-003) | 96/99 | 99/99 | 95/97 | reported |
| Distance of the deposited object from its place point, median / 95th / max | 4.7 / 12.1 / 19.4 mm | 4.6 / 5.1 / 18.0 mm | 4.7 / 6.0 / 35.5 mm | — |

Development set with the same code (`phase4_bin_picking_dev_20261111T190309Z_s1000`): 29/29 attempts, 29/30 objects, 0 of 5235 states in collision. RAM with Gazebo, MoveIt, depth model and perception, no RViz: 1.4–1.5 GiB used, 2.3–2.4 GiB available, during every run.

All three sets pass every threshold. Over the three: 295 of 297 attempts, 295 of 300 objects, 0 of 53 927 executed states in collision, the cell's verdict equal to ground truth on all 297 attempts.

### The five objects that were not deposited

- Three were never attempted and the scene ended as `no_feasible_grasp`, which was true: a prism 48 mm from a wall with its only finger direction pointing at it (seed 1 scene 7) and one like it (seed 3 scene 17), and a cylinder in a corner 35 and 38 mm from two walls (seed 3 scene 3). The fingers need 45 to 48 mm on the side they close from (D-016).
- Two slipped out of the fingers during the transfer and fell on the table between the bins: a cube (seed 2 scene 10) and a prism (seed 3 scene 8). The cell saw the jaws close past the object width (`grasp_slipped`), recovered, kept the place point as taken, and emptied the rest of the bin. It reported the bin as empty, which it was; it has no means to see an object on the table. 2 slips in 297 attempts with this code, 3 in the 579 attempts of all the full runs of this entry after the first development run.
- The 5 deposited objects not resting upright are all prisms (5 of 96 deposited prisms): the turn between the pads measured in D-015.

### How the evaluation went (runs before the final code, all reported)

The thresholds were not changed at any point. The code was, three times, after a run failed; each fix is a fault of the loop, not a tuning of the grasp or perception parameters.

| Run | Commit | Result | What it showed |
|---|---|---|---|
| `phase4_bin_picking_dev_20261111T171027Z_s1000` | `7d37b14` + uncommitted | 12/29 attempts, 27/30 objects | Obstacles left in `move_group` by an earlier manual run blocked 4 of the 6 place points; the descent was refused and the recovery dropped the object. Fix: the world is emptied at every new scene; with no feasible place point the object is put back. |
| `phase4_bin_picking_dev_20261111T172211Z_s1000` | `6418c47` + uncommitted | 29/29, 29/30 | Passes; code committed as `4fbd04e`, `8ffad06`. |
| `phase4_bin_picking_eval_20261111T173304Z_s1` | `8ffad06` | **failed**: 50/52 attempts, 52/100 objects, 40 states in collision | First run on the evaluation seed. 49/50 on the first ten scenes; then a cube slipped (detected), the next prism was set down on it, its release stalled and the recovery opened the fingers fully into the bin wall. MoveIt could not plan from that state and the remaining 12 scenes ended as `no_view`. Fix (`429e974`): a lost object keeps its place point, the recovery opens only as far as the cycle. |
| `phase4_bin_picking_eval_20261111T175731Z_s1` | `429e974` | passes: 99/99, 99/100 | Same seed again, so no longer a first look at these scenes; a confirmation set on seed 2 was added for that reason. |
| `phase4_bin_picking_confirm_20261111T183107Z_s2` | `429e974` | **failed**: 70/73 attempts, 73/100 objects, 21 states in collision | First look at seed 2. Three times the fingers, opening after the release, entered the 15 mm margin of a placed neighbour; no retreat could be planned and the arm stayed in the bin for the scenes that followed (10 ended as `no_view`). Fix (`17df986`): the retreat with open fingers is checked before a place point is chosen, the loop can leave straight up after dropping its object beliefs, the margin is 8 mm. A second confirmation set on seed 3 was added. |
| dev, seed 2, seed 1, seed 3 in this order | `17df986` | all pass | The table above. Seed 3 is the only set the final code met for the first time: 97/98 attempts, 97/100 objects. |

- Comparison with E-003 part C (one object in the middle of the bin, 28/30, median cycle 18.4 s): same cycle time with five objects anywhere in the bin; the selection of a grasp among about 12 candidates costs 0.04 s (median), the view 0.8 s including the wait at ready.
- Limits: objects in their catalog orientation and not touching (the user's scope for phase 4); top-down grasps only, so an object within about 45 mm of a wall on the side the fingers close from is left in the bin; the output bin and the table are not observed, so the cell does not know how a deposited object lies nor where a slipped one went; friction grasps slipped in 3 of 579 attempts and prisms turn between the pads (D-015); the depth error model is per-pixel noise, not simulated stereo (D-011). The put-back and the rule that a failed grasp is not repeated ran once, in a two-minute check on one scene of seed 2 while the second fault was being fixed (the prism was set down where it had been picked and the scene ended as `no_feasible_grasp`); in the runs with the final code no attempt failed on an object still in the input bin, so they did not run there. Evaluation seed 1 was run three times, the last two after fixes prompted by other runs; seed 3 is the cleanest evidence.
- Conclusion: phase 4 measurable test passes on the evaluation set and on two confirmation sets.

## E-005 — Phase 5: tumbled objects, learned grasp ranking vs baseline, recovery (2026-11-12)

- Question: on scenes where the geometric model is incomplete (tumbled objects that perception cannot name), does the cell still empty the bin safely, does it recover from the failures it meets, and does a grasp ranking learned from simulated outcomes pick better than the hand-designed score of D-016 on the same held-out scenes?
- Scenes (D-018): as in E-004 (5 objects, separate, anywhere in the bin), but every cylinder lies on its side and every prism stands on an end face with probability 0.5. Perception (D-012) is unchanged and publishes them as `unknown` clusters.
- Strategies (D-019), all with the same perception, hard filters and loop: `catalog_only` (the phase 4 cell: unknown clusters are never tried, reference); `baseline` (unknown clusters get candidates across the long and the short side, ranked by the D-016 score); `explore` (same candidates, uniform random order, training data only); `learned` (same candidates, ranked by the learned P(success)).
- Success of an attempt and the other criteria: as in E-004 (cell verdict, object deposited on any face, no executed state in collision).
- Seeds: development 5100 (6 scenes + 1 empty), training 5000 (60 scenes, `explore` only), held-out 51 (`eval`) and 52 (`confirm`), 20 scenes + 2 empty each. Thresholds (`thresholds` in the config) fixed with the code before the first held-out run; the comparison rule (learned becomes the default only if its pick success is higher on both held-out seeds) fixed in D-019 before training.
- Commit / config: code frozen at `df5384b` (later commits until the learned runs change docs only, so the logs record `8757d0b` and `55a5110` too); `scripts/config/phase5_tumbled.yaml` (sha256 `c012c51389d5`), `openfactory_manipulation/config/pick_place.yaml`. Cell: `ros2 launch openfactory_bringup cell.launch.py headless:=true rviz:=false` on the VM display (relaunched after a VM restart; nothing else changed). Logs in `data/trials/`.
- Re-run, in the VM with the cell launched: `python3 scripts/benchmark.py --config scripts/config/phase5_tumbled.yaml --set eval|confirm|train|dev --strategy baseline|learned|explore|catalog_only`.

### Part A — the baseline on tumbled scenes (written before any training)

| Metric | `eval`, seed 51 (`phase5_tumbled_eval_baseline_20261112T164804Z_s51`) | `confirm`, seed 52 (`…confirm_baseline_20261112T172359Z_s52`) | Pass |
|---|---|---|---|
| Completion (95 % Wilson CI) | 100/100 (0.96–1.00) | 95/100 (0.89–0.98) | ≥ 0.80 |
| Completion cube / cylinder / prism | 37/37, 28/28, 35/35 | 35/35, 31/31, 29/34 | each ≥ 0.70 |
| Pick success (attempts) | 100/109 (0.85–0.96) | 95/103 (0.85–0.96) | ≥ 0.75 |
| Executed states in collision | 0 of 19 192 | 0 of 18 230 | 0 |
| Attempts the cell believed while ground truth disagrees; failures it reported while the object arrived | 0; 0 | 0; 0 | ≤ 3 |
| Detected failures followed by a return to ready with a view | 9/9 | 8/8 | ≥ 0.90 |
| Attempts on the 2 empty bins | 0 | 0 | 0 |
| Scenes cleared; end reason correct | 20/20; 22/22 | 18/20; 22/22 | — |
| Scene ends | 22 `bin_empty` | 20 `bin_empty`, 2 `failure_budget` | — |
| Class of the target as perception should see it (catalog type upright, `unknown` tumbled) | 109/109 | 103/103 | — |
| Cycle time per attempt, median (min–max); perception / planning with selection / execution | 18.1 s (7.7–21.1); 0.77 / 0.12 / 17.3 s | 18.1 s (8.2–20.8); 0.77 / 0.13 / 17.2 s | — |
| Other objects disturbed; objects lost out of both bins | 0; 0 | 0; 0 | — |

Attempts by what the target really was and how it was held:

| Target (ground truth) | Grasp | `eval` | `confirm` |
|---|---|---|---|
| cube | catalog | 37/37 | 35/35 |
| cylinder, standing | catalog | 12/12 | 11/11 |
| cylinder, lying | `unknown`, across the short side (the diameter) | 16/16 | 20/20 |
| prism, ridge up | catalog | 16/16 | 17/17 |
| prism, standing on an end face | `unknown`, across the short side (face against edge) | 19/19 | 12/12 |
| prism, standing on an end face | `unknown`, across the long side | **0/9** (7 `grasp_failed`, 2 `grasp_slipped`) | **0/8** (5 `grasp_failed`, 3 `grasp_slipped`) |

- Both held-out sets pass every threshold with the baseline: the cell now picks the tumbled objects that the phase 4 cell could not try. All 17 failures are the same thing: a standing prism held across the long side of its footprint, which is a wedge (the pads close on the slanted faces, D-018). The D-016 score cannot see it: it prefers that side whenever it leaves more room around the fingers.
- The failures are detected (jaws closing past the object, or opening during lift/transfer) and recovered from every time; the rule that a failed grasp is not repeated then makes the cell try the short side, which holds. On `eval` that cost 9 extra attempts and nothing else. On `confirm` it cost 5 objects: in scenes 7 and 9 three standing prisms failed one after another and the loop stopped on its budget of 3 consecutive failures (D-017), leaving four standing prisms and one ridge-up prism (never attempted) in the bin. The cell reported `failure_budget`, which was true.
- Development (seed 5100, not counted): the first run (`…dev_baseline_20261112T161340Z_s5100`, 29/30) also counted a held standing prism as `grasp_failed` because its measured width is 3–4 mm under the real one; fixed by the measured-width tolerance of D-018 (`67dcdc9`), then `…dev_baseline_20261112T162601Z_s5100` 29/30 objects, 29/32 attempts, and `…dev_explore_20261112T163645Z_s5100` 30/30, 30/31 with the code that was frozen. A one-scene smoke run before them: `…dev_baseline_20261112T161031Z_s5100`, 5/5.

### Part B — training data and model

- Data: `phase5_tumbled_train_explore_20261112T175806Z_s5000` (seed 5000, 60 scenes, `explore`, code `df5384b`, log commit `55a5110`): 320 attempts, 287 successful, 287/300 objects deposited, 0 of 55 500 executed states in collision, verdict equal to ground truth on all 320, recovery 33/33. 48 scenes `bin_empty`, 12 `no_feasible_grasp` (objects near the walls, D-016).
- What the random order tried: standing prism across the long side 0/31; across the short side 32/33; lying cylinder across the diameter 60/61; every catalog grasp 194/194, plus one prism that spawned standing and was tried as a catalog prism (below). The two failures off the wedge are both in scene 43. A lying cylinder slipped out during the transfer (`grasp_slipped`, detected), fell back into the input bin without touching anything and was picked at the next attempt. A standing prism there failed as a wedge, which tipped it over; the next view still showed a cluster, the grasp across its short side failed (`grasp_failed`), and the view after that showed a prism lying ridge-up, recognised as a catalog prism and picked. Recovery by looking again.
- Model (`openfactory_manipulation/config/grasp_ranker.json`, report `grasp_ranker_report.json`; `python3 scripts/train_grasp_ranker.py data/trials/phase5_tumbled_train_explore_20261112T175806Z_s5000 --out …`): L2 weight 0.3 chosen from {0.3, 1, 3, 10, 30, 100} by 5-fold cross-validation grouped by scene (it is the smallest of the grid; stronger weights score worse on log-loss, weaker ones were not tried). Out-of-fold log-loss 0.065 against 0.332 for a constant prediction at the success rate, Brier 0.009 against 0.093, AUC 0.97. The largest negative weights (standardised features) are `unknown_width_ratio` −2.5, `width_ratio` −2.4, `width` −2.1 and `class_unknown` −1.3; the largest positive is the perception score +1.3.
- Sanity check on the development candidates (seed 5100, not used for training): P(success) 0.10–0.24 for an `unknown` cluster across its long side, 0.94–0.99 across its short side, above 0.999 for catalog cubes and cylinders.

### Part C — learned ranking vs baseline on the held-out seeds

Runs: `phase5_tumbled_eval_learned_20261112T202750Z_s51` and `phase5_tumbled_confirm_learned_20261112T210142Z_s52` (model of part B, code `df5384b`, log commit `cf65645`, which adds only the model, a test and docs), against the baseline runs of part A on the same scenes. `scripts/compare_strategies.py` on the run directories gives every number below.

| Metric | `eval` baseline | `eval` learned | `confirm` baseline | `confirm` learned |
|---|---|---|---|---|
| Pick success (95 % Wilson CI) | 100/109 (0.85–0.96) | **99/101 (0.93–0.99)** | 95/103 (0.85–0.96) | **99/102 (0.92–0.99)** |
| on `unknown` targets | 35/44 | 34/36 | 32/40 | 36/38 |
| on catalog targets | 65/65 | 65/65 | 63/63 | 63/64 |
| Completion | 100/100 | 99/100 | 95/100 | 100/100 |
| Failures by reason | 7 `grasp_failed`, 2 `grasp_slipped` | 2 `grasp_failed` | 5 `grasp_failed`, 3 `grasp_slipped` | 1 `grasp_failed`, 2 `grasp_slipped` |
| Scene ends | 22 `bin_empty` | 21 `bin_empty`, 1 `no_feasible_grasp` | 20 `bin_empty`, 2 `failure_budget` | 22 `bin_empty` |
| Executed states in collision | 0 of 19 192 | 0 of 18 133 | 0 of 18 230 | 0 of 18 391 |
| Undetected failures; false alarms | 0; 0 | 0; 0 | 0; 0 | 0; 1 |
| Detected failures recovered | 9/9 | 2/2 | 8/8 | 3/3 |
| Cycle time per attempt, median; grasp selection, median | 18.1 s; 0.036 s | 17.9 s; 0.035 s | 18.1 s; 0.036 s | 18.2 s; 0.035 s |
| Thresholds of part A | pass | pass | pass | pass |

Paired differences, learned − baseline, per scene (bootstrap over scenes, 10 000 resamples, 95 % interval):

| | `eval` (20 scenes) | `confirm` (20 scenes) | both (40 scenes) |
|---|---|---|---|
| Pick success | +0.063 (+0.020, +0.107) | +0.048 (−0.010, +0.111) | **+0.056 (+0.022, +0.092)** |
| Failed attempts per scene | −0.35 (−0.60, −0.10) | −0.25 (−0.60, +0.05) | **−0.30 (−0.50, −0.13)** |
| Completion | −0.01 (−0.03, 0.00) | +0.05 (0.00, +0.13) | +0.02 (−0.01, +0.06) |

Over the 40 scenes: learned 198/203 attempts (0.94–0.99), 199/200 objects; baseline 195/212 (0.88–0.95), 195/200 objects. Reference `catalog_only` (the phase 4 cell, `…eval_catalog_only_20261112T194140Z_s51`, `…confirm_catalog_only_20261112T200350Z_s52`): 65/65 and 62/63 attempts but only 65/100 and 63/100 objects; 17 scenes of each set end as `only_unknown`, with every tumbled object left in the bin.

- The learned ranking does what the data showed: it chose the short side of a standing prism 35 times (34 held) and the long side only 3 times, each time because the wall filter had rejected the short side. The baseline chose the long side 17 times out of 48 on the same scenes. Its pick success is higher on both held-out seeds, which is the rule of D-019, so it is the default from commit `5f1a5fa`. The difference is significant over the 40 scenes and on `eval`, not on `confirm` alone.
- Its 5 failures: 3 times the short side of a standing prism was rejected by the wall filter and the wedge was the only feasible candidate (P(success) 0.04–0.16): a ranking orders, it never refuses, so the cell tried it. Twice the next view made the short side feasible and the prism was picked; once (`eval` scene 16) it did not, and the prism was left (`no_feasible_grasp`, true). One face-against-edge grasp failed (`confirm` scene 9: the jaws stopped at 55 mm, detected, picked at the next attempt). One cube slipped during the transfer and fell into the output bin: the cell reported `grasp_slipped` while the cube was deposited, the only false alarm of the phase 5 runs with the frozen code.
- Why scene 16 differed: the baseline met the same prism with its short side clear. The minimum rectangle around a triangle has three equivalent orientations; perception picks one, and the two runs did not see bit-identical depth images, so the choice differed. The paired differences carry that run-to-run noise too.
- The learned score added no measurable time: grasp selection 0.035 s median with it, 0.036 s without. Training takes seconds on the VM.

### Part D — recovery

Every detected failure in the phase 5 runs with the frozen code, held-out and training: 9 + 8 (baseline), 2 + 3 (learned), 1 (`catalog_only`), 33 (training) = **56 of 56** followed by the cell back at ready with a new view on its own (95 % Wilson CI 0.94–1.00; threshold ≥ 0.90 with at least 20 failures). Of the 56, 51 were wedges, 1 a face-against-edge miss, 3 slips in the transfer and 1 a grasp on a prism that the wedge before it had tipped over (training scene 43). No recovery needed the straight-up escape (0 escapes), no object was lost out of both bins and no object was disturbed by a pick. What did end scenes early was the loop's own budget: 2 `failure_budget` scenes with the baseline on `confirm`, none with the learned ranking.

### Part E — the phase 4 evaluation set with the final code

`phase4_bin_picking_eval_20261112T213704Z_s1`: `scripts/config/phase4_bin_picking.yaml`, seed 1, the 20 + 2 scenes of E-004, with the final defaults (`5f1a5fa`: unknown-cluster grasps on, learned ranking). 99/99 attempts, 99/100 objects (cube 30/30, cylinder 36/36, prism 33/34), 0 of 17 921 executed states in collision, 0 undetected failures, 0 false alarms, 0 attempts on the empty bins, 21 `bin_empty` and 1 `no_feasible_grasp` (scene 7, the prism by the wall that E-004 also left), cycle time 17.9 s median. Every E-004 threshold passes, with the same result as the E-004 evaluation run (99/99, 99/100).

### Conclusion

- The cell now picks objects its perception cannot name, through the same hard filters, and on tumbled scenes deposits 195/200 objects with the baseline score and 199/200 with the learned ranking, where the phase 4 cell deposits 128/200. Every phase 5 run with the frozen code had 0 executed states in collision and 0 undetected failures, and all 56 detected failures were recovered autonomously.
- The learned ranking beats the baseline on pick success on both held-out seeds (the rule of D-019), by 5.6 points over the 40 scenes (bootstrap 95 % interval 2.2 to 9.2), and it is the default. It got there by learning the one failure mode these scenes have (a standing prism held across its long side is a wedge) from random-order outcomes, without being told; that is also the limit of the result: a richer scene family is needed to show more than that.
- Phase 5 measurable test (set in the plan before the runs): 0 collisions in every run, ≤ 3 undetected failures per set (0), recovery ≥ 0.90 with ≥ 20 failures (56/56), completion of the default strategy ≥ 0.80 on `eval` and `confirm` (0.99 and 1.00), comparison run and reported with uncertainty on two unseen seeds with 100 objects per strategy and set, phase 4 evaluation set still passing: **pass**.
- Limits, besides those of D-018 and D-019: one camera, so the output bin and the held object are still unseen (E-004); simulation is not bit-repeatable between runs, so a paired scene is close but not identical for the two strategies; the training and test scenes come from the same generator.

## E-006 — Phase 6, relay loop: a ring of N arms passing objects round N bins (2026-11-13)

- Question: do N arms keep objects going round N bins on their own (arm *i* from bin *i* to bin *(i + 1) mod N*, D-020, D-021), with every motion planned against the other arms, for N = 2 and N = 3 over 100 handovers each, and does a ring of 4 arms still run on this VM?
- Strategy: `relay_round_robin`: one arm at a time, the others parked; each turn is one attempt of the phase 5 cell (`BinPicking.step`, learned ranking, unknown-cluster grasps on) with the place points and obstacles taken from the view of the destination bin, and the object that waited longest first.
- Scene: 3 objects per bin (6 with N = 2, 9 with N = 3, 12 with N = 4), types drawn by the seed, catalog orientation, random position and yaw (bin *k* holds scene *k* of the seed). The harness spawns them once and never touches them again.
- Success of a handover: the cell reports it (pick verified, object seen within 30 mm of its place point in the next view of the destination bin) **and** ground truth shows that object at rest inside the destination bin **and** no executed state (`/joint_states` every 0.1 s, every arm, re-checked by MoveIt once per arm group, so arm against arm too) is in collision.
- Thresholds, from the plan approved before any code and in `scripts/config/phase6_relay.yaml` from its first version: handover success ≥ 0.90; 0 executed states in collision; ≤ 1 object lost out of the bins; ≤ 3 failures the cell did not detect; the loop reaches 100 handovers without stopping itself; every arm completes handovers. N = 4 is a smoke run (one lap at least, RAM and real-time factor reported), not a threshold.
- Seeds: development 6000 (20 handovers) / evaluation 61 (100 handovers) / confirmation 62 (100 handovers, added after the first evaluation run with three arms failed, as told below). Camera depth model on.
- Commit / config: final code `a455245`, `scripts/config/phase6_relay.yaml` (sha256 `7ebf741ad418`), `openfactory_manipulation/config/pick_place.yaml`. First evaluation chain: `f134106`, config sha256 `91dfafd804f1` (the same file without the `confirm` set). Ring: `ros2 launch openfactory_bringup ring.launch.py n:=N rviz:=false` on the VM display. Logs in `data/trials/`.
- Re-run, in the VM with the ring of the same N launched: `python3 scripts/relay_benchmark.py --arms 2 --set eval`, `--arms 3 --set eval`, `--arms 3 --set confirm`, `--arms 4 --set dev --handovers 8`.

### Result with the final code

| Metric | N = 2, seed 61 (`phase6_relay_n2_eval_20261113T183530Z_s61`) | N = 3, seed 61 (`phase6_relay_n3_eval_20261113T161244Z_s61`) | N = 3, seed 62 (`phase6_relay_n3_confirm_20261113T173527Z_s62`) | Pass |
|---|---|---|---|---|
| Handover success (95 % Wilson CI) | 99/100 (0.95–1.00) | 100/100 (0.96–1.00) | 97/100 (0.92–0.99) | ≥ 0.90 |
| Per arm | 49/50, 50/50 | 34/34, 33/33, 33/33 | 33/33, 33/33, 31/34 | each ≥ 1 handover |
| Loop end | `handovers_done` | `handovers_done` | `handovers_done` | reaches 100 turns |
| Executed states in collision | 0 of 26 584 | 0 of 43 676 | 0 of 31 929 | 0 |
| Objects lost out of the bins; other objects disturbed by a turn | 0; 0 | 0; 0 | 0; 0 | ≤ 1 lost |
| Handovers the cell believed while ground truth disagrees; failures it reported while the object arrived | 0; 0 | 0; 0 | 0; 0 | ≤ 3 |
| Failures by reason (stage) | 1 `grasp_slipped` (after transfer) | none | 1 `grasp_slipped` (after transfer), 2 `grasp_failed` (close) | — |
| Detected failures followed by the arm parked and a new view | 1/1 | none to recover | 3/3 | — |
| Turns skipped; arms not parked after a turn | 0; 0 | 0; 0 | 0; 0 | — |
| By what the target was: cube / cylinder / prism / two objects seen as one cluster | 20/20, 40/40, 39/40, — | 35/35, 25/25, 40/40, — | 32/33, 50/50, 15/15, 0/2 | — |
| Class of the target as perception saw it; xy error median / max | 100/100; 0.31 / 0.88 mm | 100/100; 0.26 / 7.09 mm | 98/98; 0.36 / 6.37 mm | — |
| Handed-over object seen from its place point, median / 95th / max | 4.2 / 5.3 / 20.1 mm | 4.6 / 10.9 / 21.3 mm | 4.2 / 9.0 / 27.5 mm | — |
| Picks of an object that another arm had set down | 94 | 91 | 89 | — |
| Candidates; rejected by walls / neighbours / gripper width / earlier failure | 1249; 21 / 0 / 9 / 5 | 1070; 20 / 0 / 0 / 0 | 1390; 54 / 12 / 33 / 29 | — |
| Simulated time from the first to the last handover | 2807 s | 2761 s | 2761 s | — |
| Turn time on the wall clock, median (min–max) | 27.1 s (23.2–30.0) | 44.1 s (32.8–83.5) | 31.9 s (17.8–43.1) | — |
| of which views / planning / execution, median | 4.5 / 0.23 / 17.5 s | 7.0 / 0.40 / 28.8 s | 5.5 / 0.29 / 21.0 s | — |
| Handovers per hour of wall time | 133 | 80 | 108 | — |
| Objects that went round in the last 20 handovers | **3 of 6** | 8 of 9 | **5 of 9** | reported |

All three runs pass every threshold. Over the three: 296 of 300 handovers, 0 of 102 189 executed states in collision, the cell's verdict equal to ground truth on all 300 turns, no object out of the bins, all 4 failures detected and recovered from.

N = 4, smoke (`phase6_relay_n4_dev_20261113T192518Z_s6000`, seed 6000, 8 handovers): 8/8, two laps, every arm 2/2, 0 of 3579 states in collision, turn time 45.4 s median, 79 handovers per hour of wall time. It runs; it is not a full set and counts for no threshold.

VM during the runs (`~/p6f_*.env`, one sample 5 minutes into each): N = 2, 1.75 GiB used, 2.16 GiB available, real-time factor 0.96; N = 3, 1.96 GiB used, 1.94 GiB available, real-time factor 0.52 and 0.20; N = 4, 2.13 GiB used, 1.77 GiB available, real-time factor 0.42; load average 7.4–9.7 on 4 cores, no swap. One sample of the real-time factor says little: the run that showed 0.20 was the faster of the two with three arms. In simulated time a turn takes the same 28 s with two and with three arms; the wall-clock rates differ with the CPU the VM gets (`docs/environment.md`).

### Objects that drop out of the loop

The thresholds count handovers, and a loop can make 100 of them with fewer and fewer objects. The last row of the table is computed from `attempts.jsonl` (distinct targets of the successful handovers in the last 20 turns); the views in `turns.jsonl` show why the others stopped moving. No arm tried and failed on them again and again: the hard filters refused them, which is what they are for.

| Run | Out of the loop at the end | Cause |
|---|---|---|
| N = 2, seed 61, `f134106` (first chain) | none of 6 | — |
| N = 2, seed 61, `a455245` | 3 prisms of 6 objects | One prism set down at turn 76 at a corner place point of bin 1, 50 mm from the end wall: never picked again, every candidate `wall_collision`. At turn 80 a prism slipped during the transfer and fell back into bin 0 against another prism; the two are seen from then on as one `unknown` cluster, wider than the gripper |
| N = 3, seed 61, `a455245` | 2 prisms of 9 objects | Each set down at a corner place point (turns 61 and 83) and never picked again, every candidate `wall_collision` |
| N = 3, seed 62, `a455245` | 2 prisms, and a cube and a prism as one cluster, of 9 objects | At turn 2 a cube slipped and fell back into bin 2 against a prism set down there at turn 1 (the cube was never handed over in this run). The two are seen as one `unknown` cluster of 109 × 64 mm, tried across its short side at turns 11 and 35 (`grasp_failed`, detected) and excluded after that. Two prisms set down at or near corner place points (turns 16 and 73), every candidate `wall_collision` |

- A prism has one finger direction, along its 60 mm length. The six place points of D-017 are 50 mm from the end walls and 55 mm from the side walls, chosen for a bin that nobody picked from. A prism set down at a corner point with its length towards the near wall is the case E-004 already left in the bin (a prism 48 mm from a wall with its only finger direction pointing at it). Cubes and cylinders at the same points were always picked again. The yaw at which an object is set down is whatever it had in the gripper: the loop does not choose it.
- A cluster of two touching objects is the phase 4 limit "objects not touching": nothing in the loop separates them.
- So in 3 of the 4 runs of 100 handovers the ring ended with fewer objects going round than it started with (6 → 3, 9 → 7, 9 → 5). How long a ring keeps going before it ends as `no_work` was not measured: no run was longer than 100 handovers.
- The same two runs on seed 61 with N = 2 are not copies of each other: the first made 100/100 with all 6 objects going round to the end, the second met one slip and one prism by a wall. The simulation is not bit-repeatable (E-005) and a loop feeds every difference back into the scene.

### How the evaluation went (all runs reported)

The thresholds were never changed. The code was changed once after an evaluation run failed.

| Run | Commit | Result | What it showed |
|---|---|---|---|
| `phase6_relay_n2_dev_20261113T130439Z_s6000` | `b476917` + uncommitted | 4/4 | Smoke run of the harness |
| `phase6_relay_n2_dev_20261113T130654Z_s6000` | same | 20/20, 0 of 5607 states | One cube made 18 of the 20 handovers and four objects never moved: the ranking prefers the object just set down, alone on the roomiest point. Fix: the object that waited longest goes first (D-021) |
| `phase6_relay_n2_dev_20261113T131850Z_s6000` | same | stopped by hand after 9 turns, not counted | Every state of two turns "in collision": the validity check of the whole robot as one group set a padded obstacle of the destination bin against the wall of that bin, which is a robot link. Fix in the harness: one check per arm group (D-021) |
| `phase6_relay_n2_dev_20261113T132416Z_s6000` | same | 20/20, 6/6 objects moved, 0 of 6407 | Passes |
| `phase6_relay_n3_dev_20261113T133800Z_s6000` | same | 20/20, 9/9 objects moved, 0 of 8003 | Passes; code committed up to `f134106` |
| `phase4_bin_picking_eval_20261113T135351Z_s1` (2 scenes) | same | 10/10 attempts, 10/10 objects, 0 of 1922 | The single-arm cell with the shared code changed for the ring: as in E-004 |
| `phase6_relay_n2_eval_20261113T140116Z_s61` | `f134106` | passes: 100/100, 50 laps, 0 of 31 151, 113 handovers per hour | First look at seed 61 |
| `phase6_relay_n3_eval_20261113T150030Z_s61` | `f134106` | **failed**: stopped itself as `no_view` after 50 turns, 50/50 handovers, 0 of 24 521 | With three cameras the VM simulated at half speed and the 10 s wait for a view, counted on the wall clock, ran out. Fix (`a455245`): the wait is counted on the simulation clock (D-021). Nothing about grasps, place points or scenes changed |
| `phase6_relay_n4_dev_20261113T154854Z_s6000` | `f134106` | 8/8, 2 laps, 0 of 4418 | Smoke; real-time factor 0.29 |
| `phase6_relay_n3_dev_20261113T160519Z_s6000` | `f134106` + the fix | 6/6 | Check of the fix |
| the three runs of the table, then `phase6_relay_n4_dev_20261113T192518Z_s6000` | `a455245` | all pass; smoke 8/8 | Seed 61 with N = 3 had been seen for 50 turns by the earlier code; seed 62 is the set the final code met for the first time: 97/100 |

- Conclusion: the relay-loop test passes for N = 2 and N = 3 on the evaluation seed and for N = 3 on a confirmation seed, and a ring of 4 arms runs on the VM. What the test does not show, and the runs do: the loop slowly loses objects to poses its arms cannot grasp, so "for ever" is not demonstrated beyond 100 handovers. Phase 6 was closed on this result by the user (2026-11-13); what is left is in `docs/ideas.md`, idea 1.
- Limits, besides those of D-020 and D-021: one arm moves at a time, so the handover rate does not grow with N (28 s of simulated time per turn for any N); the simulation slows down with every bin on this VM (CPU, not memory); slips are rare (2 in 300 turns) but in a loop each one can take two objects out; N = 4 was run for two laps only; the three sets share one object generator and three object types.

## E-007 — Phase 7: a Rubik's cube solved by one arm, a nest and one camera (2026-11-14)

- Question: can the cell, starting from a scrambled cube whose state it does not know, read it with its camera, solve it with physical turns of its layers and know that it has (D-022 to D-027)? Asked in parts, from the cube alone to the whole loop: A the cube mechanism on a test bench; B turns by the arm; C flips by the arm; D the scan of the six faces; E the closing test, scrambled cubes solved; F a cube the arm scrambles itself, then solves.
- Strategy: `rubik_scan_solve_verify`: scan (five flips, six views), `kociemba` solution, flips and turns planned for the fewest seconds of flips, a view after every operation, scan again on a mismatch and at the end (D-026, D-027).
- Scene: one cube, 57 mm, 92 g, 26 cubies on the mechanism of D-023, scrambled with 25 seeded random moves and spawned in the nest in a seeded orientation among the 24 (part F: spawned solved). The harness never touches it afterwards.
- Success: per part below. Everywhere: ground truth is read from the simulator's link poses after every operation, for evaluation only; a cubie is on the grid within 0.3 mm and 0.6°, the cube in its seat within 2 mm and 2°; executed joint states (every 0.1 s) are re-checked by MoveIt against the fixtures and the seated cube, and only the fingers on the cube are not a collision.
- Thresholds, from the plan approved before any code (`scripts/config/phase7_rubik.yaml`): A every check passes; B and C at least 57 of 60; D 20 of 20 states read, none wrong accepted; E at least 9 of 10 cubes on the evaluation seed and 4 of 5 on the confirmation seed, 0 executed states in collision, 0 cubes called solved that are not; F one run.
- Seeds: development 7100 / evaluation 71 / confirmation 72 (bench: 7000). Camera depth model on.
- Commit / config: see each part. Cell: `ros2 launch openfactory_bringup rubik.launch.py rviz:=false` on the VM display, relaunched for every cube of parts D to F (the Gazebo server leaks memory at every change of axis of the cube, `docs/environment.md`). Logs in `data/trials/`.

### Development runs (not results: they shaped the cell)

All on seed 7100, 2026-11-14, with the code of commit `bac8f6d` or just before it.

| Run | What | Outcome |
|---|---|---|
| `phase7_rubik_mixed_dev_20261114T022832Z_s7100` | 40 flips and turns in a seeded mixed sequence, no camera in the loop | 40 of 40 as the logical model predicts (state, orientation, seated, square); 0 of 6519 states in collision; cube at most 1.0 mm off its seat; 16.5 s per operation |
| `p7_scan_dev` | scan of cube 0 | state and orientation read correctly; 5 flips, 81 s of simulated time; server memory unchanged |
| `p7_solve_devinv` | cube 0, solved by undoing its scramble move by move (`--dev-solver inverse`, a development aid) | solved in the simulator and reported `solved`: 25 turns, 38 flips, 0 failures of the cell, 0 of 9830 states in collision, the cell's belief equal to ground truth after each of the 63 operations; 954 s simulated |
| `p7_solve_devkoc` | cube 1, with the real solver (`kociemba` 1.2.1 read from the `doppelbot` venv already in the VM, nothing installed) | solved in the simulator and reported `solved`: a 21-move solution, 21 turns and 32 flips, 0 failures, 0 of 8709 states in collision, belief equal to ground truth after each of the 53 operations, cube at most 0.75 mm off its seat; 822 s simulated; server memory 416 → 583 MB |

What the earlier development runs found is in D-024 (the grasp) and D-023 (the change of axis under load): flat pads let the cube swing in a flip, pads on the neighbouring slabs or above the centre of a face twist layers, a cube set down 3 mm off rides the nest wall, the mechanism missed a change of axis when the layers that were free were pushed while the welds changed.

### Part A — the cube mechanism on the test bench (`phase7_mechanism_20261114T032645Z_s7000`): pass

Commit `bac8f6d`, `rubik.yaml` sha256 `d61b159e7153` at the first run and the same mechanism parameters since. No robot: a cube whose core is fixed to the world in the `rubik_bench` world, a scrambled state from seed 7000; faces turned by torques applied to their centre cubies through Gazebo's `ApplyLinkWrench` system (0.065 N m until 6° from the target, then the detent finishes). This drive exists on the bench only. Re-run: `ros2 launch openfactory_simulation rubik_bench.launch.py`, then `python3 scripts/rubik_mechanism_test.py --turns 400`.

| Check | Result |
|---|---|
| Every face by a quarter turn, a half turn and a quarter turn back: the simulator's state is the logical model's after each | 18 of 18 |
| 80 % of the breakaway torque (0.032 N m) for 1 s on U, R and F, one on the free axis and two on the others: nothing turns | 3 of 3, less than 0.01 rad |
| U left 30° out of square; 0.2 N m (five times the breakaway) on R and on F for 1 s each: neither turns, U does not move, the plugin reports the axes blocked; U back to square; then R turns | 4 of 4 |
| 400 seeded random turns, then one more on another axis: state after every turn | 401 of 401, 326 changes of axis in the whole run |
| Cubies welded to the core after all of it: distance from where they were spawned on the grid | 5.3e-14 m (threshold 1 µm) |
| Largest distance of a cubie from its slot while a layer rests on its detent | 0.029 mm, 0.046° |

A turn takes 0.34 s of simulated time on the bench. The same run at commit `4c35c84`, before the change of axis was made robust to load (`phase7_mechanism_20261114T010056Z_s7000`): the same numbers, drift 7.7e-14 m. The Gazebo server grew from 118 to 658 MB over the 326 changes of axis (1.65 MB each; known issue, `docs/environment.md`).

The spawn artefact of the bench: the plugin of a spawned model starts one step after its links, so every cubie falls g dt² = 9.8 µm before it is welded; with the core fixed to the world the whole grid sits that much lower, and each cubie carries that offset round with its layers. It is measured after the spawn and taken out of the drift; a free cube falls as one and has none.

### Part B — turns by the arm (`phase7_rubik_turn_eval_20261114T033803Z_s71`): pass

Commit `bac8f6d`, `phase7_rubik.yaml` sha256 `5a555f0e3151`, seed 71. A cube scrambled with 25 moves, face F down; 60 turns drawn by the seed (24 clockwise quarter turns, 19 anticlockwise, 17 half turns), made by `CubeHandler.turn` on its own. No camera in this part: the cell's verdict is its fingers and the execution result; ground truth after every turn. Re-run, with the cell launched: `python3 scripts/rubik_cell_test.py --part turn --set eval`.

| Measure | Result | Threshold |
|---|---|---|
| Turns after which the simulator's cube is in the state and the orientation the logical model predicts, seated and square, with no state in collision | 60 of 60 (95 % Wilson interval 94–100 %) | ≥ 57 |
| Failures the cell reported | 0 | |
| Failures it did not report | 0 | |
| Executed states in collision | 0 of 8665 | 0 |
| Cube centre from its seat after a turn | median 0.70 mm, max 0.99 mm (the nest leaves 0.75 mm per side) | ≤ 2 mm |
| Cube from the nest axes | at most 0.65° | ≤ 2° |
| A cubie from its slot | at most 0.026 mm | ≤ 0.3 mm |
| Time of a turn (wall, real-time factor about 1) | median 15.3 s, max 22.8 s | |

Every turn was of the same face, the one the cube was spawned with down: a turn does not change which face is down. Other faces come down in parts C to F. The Gazebo server grew from about 415 to 797 MB in this run: the mechanism still changes axis a few times per turn (the fingers closing on the cube ask for another axis long enough), and each change leaks.

### Part C — flips by the arm (`phase7_rubik_flip_eval_20261114T035437Z_s71`): pass

Code of commit `bac8f6d` (run at `44e70b6`, documents only in between), same config and seed. The same cube; 60 flips drawn by the seed: 22 `over` and 38 `side`, with the cube turned 0, 1, 2 or 3 quarter turns in the air first (between 5 and 11 of each of the eight combinations). Every face was down at the start of a flip at least 7 times. Re-run: `python3 scripts/rubik_cell_test.py --part flip --set eval`.

| Measure | Result | Threshold |
|---|---|---|
| Flips after which the simulator's cube is in the orientation the logical model predicts, its state unchanged, seated and square, with no state in collision | 60 of 60 (95 % Wilson interval 94–100 %) | ≥ 57 |
| Failures the cell reported / did not report | 0 / 0 | |
| Executed states in collision | 0 of 11 952 | 0 |
| Cube centre from its seat after a flip | median 0.29 mm, max 0.79 mm | ≤ 2 mm |
| Cube from the nest axes | at most 0.28° | ≤ 2° |
| A cubie from its slot | at most 0.021 mm | ≤ 0.3 mm |
| Time of a flip (wall) | median 19.7 s, max 27.0 s | |

### Part D — the scan of the six faces (`p7_scan_eval`): pass

Code of commit `bac8f6d` (run at `5fd34f7`, documents only in between), same config, seed 71: cubes 0 to 19, each scrambled with its own 25 moves and spawned in its own orientation (16 different ones among the 20), each in a fresh launch of the cell. The cell looks at the face that is up, flips the cube five times, and reads state and orientation from the six views; the camera is in the loop here (depth model on). Re-run: `scripts/vm/run_rubik.sh p7_scan_eval scan eval 0 19` in the VM.

| Measure | Result | Threshold |
|---|---|---|
| Cubes whose state and orientation the cell read as the simulator has them | 20 of 20 (95 % Wilson interval 84–100 %) | 20 |
| Wrong states accepted as valid | 0 | 0 |
| Scans repeated | 0: every cube was read at the first scan | |
| Stickers assigned to a centre that was not clearly the nearest | 0 of 1080 | |
| Flips | 100 of 100 left the cube seated and square in the simulator; 0 failures reported by the cell | |
| Executed states in collision | 0 of 32 800 | 0 |
| Views (120): lowest score; cube centre seen from its seat; yaw seen | 0.78 (limit 0.5); at most 1.4 mm (limit 2 mm); at most 0.1° | |
| Cube centre as seen against the simulator, horizontally | at most 0.74 mm apart | |
| Simulated time of a scan | median 75 s (73–84 s) | |
| Gazebo server memory during a scan | unchanged: flips make no change of axis | |

### How the closing test went (all runs reported)

- `p7_solve_eval_void1` (2026-11-14, evaluation cube 0 only): **void, an infrastructure fault**. The cell had been relaunched 26 times for parts B to D, and every `cube_perception` of those launches was still running: `scripts/vm/stop_cell.sh` did not stop that node (`docs/environment.md`, "Phase 7 cell"). Load average 31 on 4 cores, 0.9 GiB available, the simulation at 0.44 of real time. The cube was scanned correctly and 15 of the 21 moves of its solution were made (38 operations, each one verified and right in the simulator); then, while the 26 stale nodes were being killed from outside, the execution of the arc of a flip was aborted twice in a row (`execution_failed` at `tip`). The first time the cube, let go where it was, fell back into its seat and the cell tried again; the second time it fell off the pedestal: `cube_lost`, reported by the cell, cube not solved. The run was stopped there, the script fixed, the VM checked clean (no stale node, 0.7 GiB used) and the closing test started again from cube 0 with the same code, seeds and config. What it shows about the cell is kept under "Limits" below: a flip that fails while the cube is tipped drops the cube.
- `p7_solve_eval_run1` (2026-11-14, evaluation cubes 0 to 3, code of commit `bac8f6d`, clean VM): **failed, stopped after four cubes**. Cubes 0 and 1 solved (54 operations each, 804 and 811 s simulated). Cube 2 lost at its 46th operation, a half turn: the cube did not turn, the fingers were forced open (`grasp_slipped`, reported by the cell), the cube ended tilted on the rim of the nest. Cube 3 lost at its first move after the scan, a quarter turn: the cell's fingers reported nothing, the next view showed no cube in the nest (`cube_lost`); it had been dragged off the pedestal. Two cubes lost of four: at least 9 of 10 was out of reach, so the run was stopped. In both the cell said so: no cube was called solved. Cause and fix: D-023, update of 2026-11-14 (resting layers are now latched). Parts A to D were run again with the fixed mechanism; their results above are those of the code before the fix and are kept as they were measured, the new ones follow.

### Parts A, B and C again, with the final mechanism (commit `dd267d5`): pass

Same configs, seeds and commands as above; the only change is the latch of resting layers (D-023, update of 2026-11-14).

| Part | Run | Result | Before the fix |
|---|---|---|---|
| A, mechanism bench | `phase7_mechanism_20261114T105909Z_s7000` | every check passes: 18 of 18 face turns, 3 of 3 below breakaway, 4 of 4 locking, 401 of 401 states over 400 random turns, 326 changes of axis; cubies welded to the core 1.2e-14 m from the grid; a resting layer at most 0.029 mm from its slot | the same counts, 5.3e-14 m |
| B, 60 turns by the arm | `phase7_rubik_turn_eval_20261114T111033Z_s71` | 60 of 60; 0 failures reported or unreported; 0 of 7477 states in collision; cube at most 1.00 mm from its seat | 60 of 60, 0 of 8665 |
| C, 60 flips by the arm | `phase7_rubik_flip_eval_20261114T112459Z_s71` | 60 of 60; 0 failures reported or unreported; 0 of 10 092 states in collision; cube at most 0.82 mm from its seat | 60 of 60, 0 of 11 952 |

Times with a clean VM: 12.6 s per turn and 17.0 s per flip on average (wall).

### Part E — the closing test, evaluation seed (`p7_solve_eval`): 10 of 10

Final code (`dd267d5`), clean VM, ten cubes of seed 71, each scrambled with 25 seeded moves and spawned in a seeded orientation, the cell relaunched for every cube: `scripts/vm/run_rubik.sh p7_solve_eval solve eval 0 9`. Cube 0 records commit `dd267d5`, cubes 1 to 9 record `1e6cff3`: that commit was made while the run was going and changes `docs/experiments.md` only.

| | Result | Threshold |
|---|---|---|
| Cubes solved (the cell says `solved`, and the simulator's cube is solved, square and seated, with no executed state in collision) | **10 of 10** (Wilson 95 %: 0.72–1.00) | ≥ 9 |
| Cubes called solved that are not | 0 | 0 |
| Executed joint states in collision | 0 of 75 947 | 0 |
| Operations | 531: 207 turns and 324 flips (100 of them in the two scans of each cube) | |
| Operations the cell reported as failed | 0 | |
| Operations that left the cube other than the cell believed (state, orientation, square, seated) | 0 of 531 | |
| Scans per cube | 2 (the first and the final one): no mismatch, no rescan, no realign | |
| Solution | 20 or 21 moves (mean 20.7), every move made once | |
| Operations per cube | 51 to 54 (mean 53.1) | |
| Cube away from its seat after an operation | at most 1.01 mm and 1.39° | 2 mm, 2° |
| Simulated time per cube | mean 793 s (769–825 s); wall 826 s; 14.6 min with the relaunch | |
| Time per operation (wall) | turn 10.5 s, flip 15.3 s | |
| Gazebo server memory over one cube | from 414 MB to 518–586 MB | |

The seed is the one of the failed run `p7_solve_eval_run1` (above): the cubes that run lost, 2 and 3, are solved here, and so are the six it never reached. The confirmation seed (72) had not been used before.

### Part E — the closing test, confirmation seed (`p7_solve_confirm`): 5 of 5

Same code and config, five cubes of seed 72, a seed no run had used: `scripts/vm/run_rubik.sh p7_solve_confirm solve confirm 0 4` (commit recorded: `1e6cff3`, the code of `dd267d5`).

| | Result | Threshold |
|---|---|---|
| Cubes solved | **5 of 5** (Wilson 95 %: 0.57–1.00) | ≥ 4 |
| Cubes called solved that are not | 0 | 0 |
| Executed joint states in collision | 0 of 38 733 | 0 |
| Operations | 276: 106 turns and 170 flips (50 of them in the scans) | |
| Operations the cell reported as failed | 0 | |
| Operations that left the cube other than the cell believed | 0 of 276 | |
| Scans per cube | 2: no rescan, no realign | |
| Solution | 21 or 22 moves (mean 21.2), every move made once | |
| Operations per cube | 53 to 57 (mean 55.2) | |
| Cube away from its seat after an operation | at most 0.98 mm and 1.24° | 2 mm, 2° |
| Simulated time per cube | mean 819 s (803–837 s); wall 843 s | |
| Gazebo server memory over one cube | from 414 MB to 519–590 MB | |

**Part E passes**: 10 of 10 and 5 of 5, no executed state in collision, no cube called solved that is not. In these fifteen cubes nothing went wrong, so the recovery paths of D-027 (retry, realign, scan again) were not exercised by the closing test. In the recorded runs they ran only before the fix (above): a failed flip was tried again once, and three cubes ended as `cube_lost`, each reported by the cell. No recorded run has made a `realign` or a scan again after a mismatch: those two paths of `RubikSolver` are written and have never run on a cube, and no test covers them.

### Part F — a cube the arm scrambles itself, then solves (`p7_scramble_eval`): pass

Same code and config (commit recorded: `1e6cff3`, the code of `dd267d5`); cube 0 of seed 71 spawned **solved**, in its seeded orientation: `scripts/vm/run_rubik.sh p7_scramble_eval scramble eval 0 0`. The cell scans it, makes the 25 seeded moves of the scramble with its own flips and turns, solves the state it then believes (kept from the scan through every verified operation) and scans again.

| | Result |
|---|---|
| Outcome | the cell says `solved`; the simulator's cube is solved, square and seated |
| Scramble | 25 moves in 56 operations (25 turns, 31 flips); the simulator's state after it is the seeded scramble |
| Solution | 21 moves in 43 operations (21 turns, 22 flips) |
| Scans | 2 (before the scramble and at the end), 10 flips |
| Operations | 109: 46 turns, 63 flips; 0 reported as failed; 0 that left the cube other than the cell believed |
| Executed joint states in collision | 0 of 17 813 |
| Cube away from its seat after an operation | at most 1.00 mm and 1.03° |
| Time | 1552 s simulated, 1922 s wall |
| Gazebo server memory | from 414 MB to 764 MB: 46 moves in one launch (`docs/environment.md`, the leak at every change of axis) |

One run, as planned: it shows that the loop holds for twice the moves of a solve; it is not a rate.

### Part D again, with the final mechanism (`p7_scan_eval2`): pass

Same 20 cubes and command as part D above, run id `p7_scan_eval2`, clean VM (commit recorded: `1e6cff3`, the code of `dd267d5`).

| Measure | Result | Before the fix |
|---|---|---|
| Cubes whose state and orientation the cell read as the simulator has them | 20 of 20 | 20 of 20 |
| Wrong states accepted as valid; scans repeated; stickers not clearly nearest to one centre | 0; 0; 0 of 1080 | the same |
| Flips | 100 of 100 left the cube seated and square; 0 failures reported by the cell | the same |
| Executed states in collision | 0 of 15 172 | 0 of 32 800 (the VM was loaded, more states per flip) |
| Views after a flip (100): lowest score | 0.78 | 0.78 |
| Simulated time of a scan | median 80 s (79–82 s); wall 82–86 s | median 75 s |
| Gazebo server memory during a scan | 414 MB, at most 6 MB more | unchanged |

### The cell and the ring after phase 7 (smoke runs, not results)

Phase 7 changed code the other two systems run (`gripper.xacro`, `simulation.launch.py`, `move_group.launch.py`, `moveit_interface.py`, `pick_place_once.py`, `gz_scene.py`, `stop_cell.sh`). With the final code (commit recorded: `1e6cff3`): `colcon test` 601 tests, 0 failures, 1 skipped; `pytest tests` 178 passed; and two short runs, each in a fresh launch.

| System | Run | Outcome |
|---|---|---|
| Single-arm cell, `cell.launch.py` | `phase4_bin_picking_dev_20261114T164348Z_s1000` (`--set dev --scenes 2`) | 9 of 9 picks, 9 of 10 objects deposited, 0 of 1604 states in collision, 0 undetected failures; scene 0 `bin_empty`, scene 1 `no_feasible_grasp` with one cylinder by a wall left in the bin: the same ends and counts as the phase 4 runs of these two scenes (`…20261111T172211Z…`, `…20261111T190309Z…`) |
| Ring of two arms, `ring.launch.py n:=2` | `phase6_relay_n2_dev_20261114T164827Z_s6000` (`--set dev --handovers 6`) | 6 of 6 handovers, three laps, each arm 3 of 3, 0 of 1602 states in collision, no object lost |

Both scripts exit 1 on a shortened run by design (the thresholds are for full sets).

### Conclusion of E-007 (2026-11-14)

With the final code every part passes its threshold: A the mechanism bench; B 60 of 60 turns; C 60 of 60 flips; D 20 of 20 scans; **E 10 of 10 cubes on the evaluation seed and 5 of 5 on the confirmation seed**; F the cube scrambled by the arm solved. Over parts E and F: 16 cubes, 916 operations, 0 of 132 493 executed states in collision, 0 cubes called solved that are not, 0 operations whose outcome the cell misjudged. Phase 7 was closed on this result by the user's decision (2026-11-14).

- What the numbers do not say. The evaluation seed had been met by the code before the fix (run 1 above, four cubes, two lost); the confirmation seed is the clean evidence. Fifteen cubes bound the success rate from below at 0.80 only (Wilson 95 % of 15 of 15). Nothing failed, so the recovery of D-027 was not measured (above).
- Limits: the cube is a mechanism, not a contact model of a real cube: its pieces cannot come apart, a layer is squared exactly before another axis engages, and resting layers are latched until the torque on them exceeds the breakaway (D-023). The cube starts in the nest; a cube that leaves its seat is lost (`cube_lost`) and a flip that fails while the cube is tipped drops it. One camera above the nest: a solve takes two scans of five flips, about 160 s of its 13 minutes. Colours are flat-shaded renders with per-pixel depth noise (D-011, D-025). The Gazebo server leaks memory at every change of axis, so a long session needs a relaunch.

## Filmed runs (2026-11-14, not results)

The first takes of the filming tool (D-029): real runs through the benchmark harnesses, recorded on simulated time by a film camera created in the running world, no window. They add no evidence to E-006 and E-007 (development seeds, the cubes and scenes of earlier development runs); they are here because each video shows a run that was judged from ground truth. Code: `src/` as in `f88de0c`; `take.json` records `147a00f`, the commit of the filming tool, with the changes of the following commit to `scripts/film.py`, `film_shots.py` and `config/film.yaml` not yet committed for the second and third take.

| Take | Run | Outcome | Video |
|---|---|---|---|
| `rubik` | `film_rubik` (`rubik_benchmark.py --part solve --set dev --index 0`) | solved, by the cell and by ground truth: 25-move scramble, 20-move solution, 50 operations (20 turns, 30 flips) and two scans, 0 states in collision; 674.6 s simulated in 2238.5 s of wall time | 681.5 s, 1280×720, 262.6 MB |
| `ring`, first | `phase6_relay_n3_dev_20261114T210346Z_s6000` (`--arms 3 --set dev --handovers 12`) | 12 of 12 handovers, four laps, 0 of 10 345 states in collision, no undetected failure; the script exits 1 on a shortened run by design, so the take is recorded as failed | 311.9 s, 70.3 MB; superseded |
| `ring` | `phase6_relay_n3_dev_20261114T212614Z_s6000` (`--arms 3 --set dev`, the full set) | passed: 20 of 20 handovers, 0 of 16 806 states in collision, 0 undetected failures, 0 false alarms, no object lost | 526.6 s, 1280×720, 118.3 MB |

```bash
# [VM] one take, detached; then wait for ~/NAME.done
scripts/vm/run_benchmark.sh film_rubik --script film.py take rubik
scripts/vm/run_benchmark.sh film_ring --script film.py take ring
# [MAC]
python3 scripts/film_edit.py fetch rubik && python3 scripts/film_edit.py edit rubik --speed 8   # 85 s
python3 scripts/film_edit.py fetch ring && python3 scripts/film_edit.py edit ring --speed 6     # 88 s
```

With the film camera at 1280×720 the Rubik's cube cell ran at 0.30 of real time over the take and the ring of three arms made 41 handovers per hour of wall time (80 and 108 without it, E-006); simulated times are what they were (a handover of the ring about 26 s). No timeout was met at that speed.

