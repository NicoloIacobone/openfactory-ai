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
