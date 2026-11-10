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
