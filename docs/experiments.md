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
