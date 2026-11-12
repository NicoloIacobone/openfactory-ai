#!/usr/bin/env python3
"""Train the grasp-ranking model of the `learned` strategy (phase 5, D-019) from trial logs.

A batch job, outside the simulation: it reads attempts.jsonl of the given runs (training scenes
only), fits ranker_fit.train and writes the model JSON that openfactory_manipulation loads,
plus a report next to it. Run in the VM (numpy) or anywhere with numpy:
    python3 scripts/train_grasp_ranker.py RUN_DIR [RUN_DIR ...] --out MODEL.json
"""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import ranker_fit as rf
import trial_log as tl


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('runs', nargs='+', type=Path, help='run directories with attempts.jsonl')
    parser.add_argument('--out', type=Path, required=True, help='model file to write')
    parser.add_argument('--folds', type=int, default=5)
    args = parser.parse_args()

    records, digest = [], hashlib.sha256()
    for run in args.runs:
        data = (run / 'attempts.jsonl').read_bytes()
        digest.update(data)
        records += tl.read(run / 'attempts.jsonl')
    sets = sorted({r['run_id'] for r in records})
    if any('_eval_' in run or '_confirm' in run for run in sets):
        raise SystemExit('evaluation runs are not training data')
    meta = {'runs': sets, 'data_sha256_12': digest.hexdigest()[:12],
            'commits': sorted({r['git_commit'] for r in records})}
    model, report = rf.train(records, k=args.folds, meta=meta)
    args.out.write_text(json.dumps(model.to_dict(), indent=2) + '\n')
    report_path = args.out.with_name(args.out.stem + '_report.json')
    report_path.write_text(json.dumps({**meta, **report}, indent=2) + '\n')
    print(json.dumps({key: report[key] for key in (
        'attempts', 'failures', 'scenes', 'chosen_l2', 'constant_log_loss', 'validation')},
        indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
