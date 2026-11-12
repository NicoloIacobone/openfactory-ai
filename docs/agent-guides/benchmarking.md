# Benchmarking

Applies when a task touches trials, logging, metrics, benchmark scripts, strategy comparisons, or training/evaluating learned models.

## Trial log

One JSON object per attempt (JSONL) under `data/trials/<run_id>/` (git-ignored), with at least:

`run_id`, `trial_id`, `attempt`, `git_commit`, `seed`, `config` (path + hash), `strategy`, `object_category`, `selected_grasp`, `candidates_total`, `candidates_rejected` (by reason), `planner_result`, `execution_result`, `success`, `failure_reason`, timings (`perception_s`, `planning_s`, `execution_s`, `total_s`).

The log format lives in one pure module with tests; every producer uses it.

## Metrics

| Metric | Definition |
|---|---|
| Pick success rate | successful grasps / attempted grasps |
| Completion rate | objects deposited in the destination bin / objects per trial |
| Cycle time | mean and median per task, split perception / planning / execution |
| Recovery rate | detected failures followed by an autonomous return to a useful state / detected failures |
| Collision rate | attempts with unsafe contact during approach, grasp, lift or place / attempts |
| Planning failure rate | candidates rejected or failed in IK/planning / candidates evaluated |
| Generalization | the metrics above on arrangements, occlusions, lighting or shapes not used for tuning |
| Perception quality | instance/pose accuracy and confidence calibration, reported next to pick success |

## Fair comparisons

- Same held-out seed list and object mix for every strategy. Tuning/training seeds and evaluation seeds are disjoint and stored in versioned config files.
- Report the number of trials and uncertainty (e.g. Wilson interval for rates, bootstrap for times).
- Break failures down by stage.
- Key comparisons: known-pose deterministic vs camera-driven; geometric vs learned grasp ranking; optionally single-view vs improved 3D/multi-view representation.

## Scripts

- `scripts/benchmark.py [--set dev|eval]`: runs the seeded scenes of `scripts/config/phase4_bin_picking.yaml` through the cell's own loop, writes `attempts.jsonl`, `scenes.jsonl` and `summary.json`. Re-runnable, no manual steps. Ground-truth judgement in `scripts/bin_picking_eval.py` (pure, tested).
- `scripts/benchmark.py --config scripts/config/phase5_tumbled.yaml --set eval --strategy baseline|learned|explore|catalog_only`: phase 5, same harness; run ids `phase5_tumbled_<set>_<strategy>_...`.
- `scripts/train_grasp_ranker.py RUN_DIR ... --out MODEL.json`: batch fit of the learned ranking (pure logic in `scripts/ranker_fit.py`, numpy, scene-grouped cross-validation); refuses evaluation runs. `scripts/compare_strategies.py RUN_DIR ...`: strategies on the same scenes, Wilson intervals and a scene bootstrap of the paired differences.
- `scripts/randomize_scene.py`: see the simulation guide.
- Results summary (tables, plots) in `docs/experiments.md` with the run id, commit and config; raw logs stay in `data/`.

## Learned models (phase 5 only)

- Only after the baseline numbers exist in `docs/experiments.md`.
- Phase 5 (D-019): training data from the `train` seed only, collected with the `uniform_random` strategy; the model file goes in `openfactory_manipulation/config/` with its provenance (runs, data hash, commits) and a `_report.json` next to it.
- Labels come from simulated trial outcomes; training runs as a batch job, not inside interactive simulation, and not heavy training on the Mac/VM. Remote compute only if needed and agreed.
- Scoped AI: rank, predict failure or uncertainty, support recovery. No end-to-end network commanding the robot.
