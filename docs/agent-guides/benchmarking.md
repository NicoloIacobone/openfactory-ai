# Benchmarking

Applies when a task touches trials, logging, metrics, benchmark scripts, strategy comparisons, or training/evaluating learned models.

## Trial log

One JSON object per attempt (JSONL) under `data/trials/<run_id>/` (git-ignored), with at least:

`run_id`, `trial_id`, `attempt`, `git_commit`, `seed`, `config` (path + hash), `strategy`, `object_category`, `selected_grasp`, `candidates_total`, `candidates_rejected` (by reason), `planner_result`, `execution_result`, `success`, `failure_reason`, timings (`perception_s`, `planning_s`, `execution_s`, `total_s`).

The log format lives in one pure module with tests (`scripts/trial_log.py`); every producer uses it.

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
| Handover success (ring) | handovers whose object rests in the destination bin, reported by the cell, with no state in collision / handovers tried |
| Objects lost (ring) | objects that end outside every bin |
| Perception quality | instance/pose accuracy and confidence calibration, reported next to pick success |

## Fair comparisons

- Same held-out seed list and object mix for every strategy. Tuning/training seeds and evaluation seeds are disjoint and stored in versioned config files.
- Report the number of trials and uncertainty (e.g. Wilson interval for rates, bootstrap for times and paired differences).
- Break failures down by stage.
- Made so far: known pose vs camera-driven (E-002, E-003), baseline vs learned grasp ranking (E-005). Open: single-view vs multi-view or a richer 3D representation.

## Scripts

- `scripts/benchmark.py [--config FILE] [--set NAME] [--strategy NAME]`: runs the seeded scenes of a set of the config (default `scripts/config/phase4_bin_picking.yaml`) through the cell's own loop, writes `attempts.jsonl`, `scenes.jsonl` and `summary.json`. Re-runnable, no manual steps. Ground-truth judgement in `scripts/bin_picking_eval.py` (pure, tested).
- A config with `strategies` (`phase5_tumbled.yaml`) names benchmark strategies, each a combination of `unknown_grasps` and a `ranking.strategy`: `catalog_only` (phase 4 cell), `baseline`, `explore` (ranking `uniform_random`, for training data) and `learned`. Run ids are `<run_name>_<set>_<strategy>_...`. The benchmark name `explore` and the ranking name `uniform_random` are kept apart on purpose: renaming either would break the re-run commands and the config hash recorded in E-005.
- `scripts/train_grasp_ranker.py RUN_DIR ... --out MODEL.json`: batch fit of the learned ranking (pure logic in `scripts/ranker_fit.py`, NumPy, scene-grouped cross-validation); refuses evaluation runs. `scripts/compare_strategies.py RUN_DIR ...`: strategies on the same scenes, Wilson intervals and a scene bootstrap of the paired differences.
- In the VM, long runs go through `scripts/vm/run_benchmark.sh NAME [--script FILE] <benchmark args>` (sources ROS, `~/NAME.log`, exit code in `~/NAME.done`) and `scripts/vm/run_chain.sh` for several in a row (`docs/environment.md`, "Operating the VM over SSH").
- `scripts/relay_benchmark.py --arms N [--config FILE] [--set dev|eval|confirm] [--handovers K]`: the relay loop of the ring (E-006, default `scripts/config/phase6_relay.yaml`). Objects are spawned once in every bin (bin *k* holds scene *k* of the seed), then the ring works alone; writes `attempts.jsonl` (one record per handover tried, the trial-log format plus `arm`, `source_bin`, `destination_bin`, `arrival`), `turns.jsonl` (one per turn, skipped ones included), `scene.json`, `summary.json`. Ground-truth judgement in `scripts/relay_eval.py` (pure, tested). In the VM: `scripts/vm/run_benchmark.sh NAME --script relay_benchmark.py --arms N ...`. The launched ring must have the same N.
- `scripts/rubik_mechanism_test.py [--turns N] [--parts faces,below,locking,drift] [--seed S]`: test bench of the Rubik's cube mechanism (phase 7, E-007 part A), with `rubik_bench.launch.py` running. Every turn is compared with the logical model through `scripts/rubik_truth.py` (pure, tested); writes `turns.jsonl` and `summary.json`. In the VM: `scripts/vm/run_benchmark.sh NAME --script rubik_mechanism_test.py`.
- `scripts/rubik_cell_test.py --part turn|flip|mixed [--set dev|eval] [--count N] [--ops "turn:1 flip:over:0"]`: seeded turns and flips of the Rubik's cube cell (E-007 parts B, C), each compared with the logical model through ground truth: state, orientation, cube seated and square, no executed state in collision (the fingers on the cube are not a collision). Config `scripts/config/phase7_rubik.yaml`. In the VM: `scripts/vm/run_benchmark.sh NAME --script rubik_cell_test.py --part turn --set eval`.
- `scripts/rubik_benchmark.py --part scan|solve|scramble --set S --index I --run-id NAME`: one cube of a seed through the cell's own loop (E-007 parts D, E, F); `--summarize --run-id NAME` writes the summary. Judged in `scripts/rubik_eval.py` (pure, tested). One cube per launch of the cell: `scripts/vm/run_rubik.sh NAME PART SET FIRST LAST` relaunches it for every cube, writes `~/NAME.log`, `~/NAME.progress` and `~/NAME.done`. `--dev-solver inverse` undoes the scramble instead of calling `kociemba`: a development aid, never a result.
- `scripts/randomize_scene.py`: see the simulation guide.
- `scripts/film.py take TAKE`: a video of a real run, with a benchmark harness as its scenario (D-029); see the simulation guide, "Filming".
- Results summary (tables, plots) in `docs/experiments.md` with the run id, commit, config and the command that re-runs it; raw logs stay in `data/` and the VM's `~/openfactory_data/trials/`.
- A result that changes what the README states (its results table, known limits, status) updates `README.md` in the same commit; check it again before every push (`CLAUDE.md`, "Docs to update with every change").
- When a phase closes, its runs are published as an archive in a GitHub release (D-030): every run of `~/openfactory_data/trials/` with a `MANIFEST.md` (build date, commit, `docs/experiments.md` as the index of the runs that count), and the link in the README and in `docs/experiments.md`.

## Learned models

- Only after the baseline numbers exist in `docs/experiments.md`.
- Protocol of D-019: training data from the training seed only, collected with `explore`; code, thresholds and model frozen before the held-out seeds. The model file goes in `openfactory_manipulation/config/` with its provenance (runs, data hash, commits) and a `_report.json` next to it.
- Labels come from simulated trial outcomes; training runs as a batch job, not inside interactive simulation, and not heavy training on the Mac/VM. Remote compute only if needed and agreed.
- Scoped AI: rank, predict failure or uncertainty, support recovery. No end-to-end network commanding the robot.
