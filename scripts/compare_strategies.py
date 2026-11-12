#!/usr/bin/env python3
"""Compare bin-picking strategies run on the same seeded scenes (phase 5, E-005).

Each run directory holds attempts.jsonl and scenes.jsonl written by scripts/benchmark.py. Runs
compared together must cover the same scenes (same seed and trial ids). Per run: pick success
and completion with 95 % Wilson intervals, failures by reason, success on targets perception
named and on `unknown` clusters, cycle time. Per pair (each run against the first): the
difference of the per-scene numbers, with a bootstrap interval over scenes, since attempts of
one scene are not independent. ROS-free.

    python3 scripts/compare_strategies.py RUN_DIR RUN_DIR [...] [--out FILE.json]
"""

import argparse
import json
import random
import statistics
import sys
from collections import Counter
from pathlib import Path

import trial_log as tl


def _rate(successes, n):
    low, high = tl.wilson_interval(successes, n)
    return {'successes': successes, 'n': n, 'rate': successes / n if n else None,
            'ci95': [low, high]}


def _target_class(record):
    return record['selected_grasp']['class_id']


def run_metrics(attempts, scenes):
    """Metrics of one run from its attempt and scene records."""
    filled = [s for s in scenes if s['n_objects'] > 0]
    named = [r for r in attempts if _target_class(r) != 'unknown']
    unknown = [r for r in attempts if _target_class(r) == 'unknown']
    deposited = sum(s['deposited'] for s in filled)
    times = [r['total_s'] for r in attempts]
    return {
        'strategy': attempts[0]['strategy'] if attempts else None,
        'attempts': len(attempts),
        'pick_success': _rate(sum(r['success'] for r in attempts), len(attempts)),
        'pick_success_named': _rate(sum(r['success'] for r in named), len(named)),
        'pick_success_unknown': _rate(sum(r['success'] for r in unknown), len(unknown)),
        'completion': _rate(deposited, sum(s['n_objects'] for s in filled)),
        'attempts_per_deposit': len(attempts) / deposited if deposited else None,
        'failures': dict(Counter(r['failure_reason'] for r in attempts if not r['success'])),
        'end_reasons': dict(Counter(s['end_reason'] for s in scenes)),
        'states_invalid': sum(r['evaluation']['states_invalid'] for r in attempts),
        'undetected_failures': sum(r['detection'] == 'undetected_failure' for r in attempts),
        'cycle_s_median': statistics.median(times) if times else None,
    }


def per_scene(attempts, scenes):
    """{trial_id: {'objects', 'deposited', 'attempts', 'failed'}} of the scenes with objects."""
    out = {s['trial_id']: {'objects': s['n_objects'], 'deposited': s['deposited'],
                           'attempts': 0, 'failed': 0}
           for s in scenes if s['n_objects'] > 0}
    for r in attempts:
        if r['trial_id'] in out:
            out[r['trial_id']]['attempts'] += 1
            out[r['trial_id']]['failed'] += not r['success']
    return out


def _percentiles(values, low=2.5, high=97.5):
    ordered = sorted(values)
    pick = lambda q: ordered[min(len(ordered) - 1, max(0, round(q / 100 * (len(ordered) - 1))))]
    return [pick(low), pick(high)]


def paired(reference, other, n_boot=10000, seed=0):
    """Difference other - reference over the scenes both runs played (per_scene dicts):
    completion and pick success as ratios of sums, failed attempts per scene; each with a
    bootstrap 95 % interval that resamples scenes."""
    common = sorted(set(reference) & set(other))
    if not common:
        raise ValueError('the runs share no scene')
    if any(reference[t]['objects'] != other[t]['objects'] for t in common):
        raise ValueError('the runs did not play the same scenes')

    def stats(trials):
        objects = sum(reference[t]['objects'] for t in trials)
        att_ref = sum(reference[t]['attempts'] for t in trials)
        att_oth = sum(other[t]['attempts'] for t in trials)
        ok_ref = att_ref - sum(reference[t]['failed'] for t in trials)
        ok_oth = att_oth - sum(other[t]['failed'] for t in trials)
        return {
            'completion': (sum(other[t]['deposited'] for t in trials)
                           - sum(reference[t]['deposited'] for t in trials)) / objects,
            'pick_success': ((ok_oth / att_oth if att_oth else 0.0)
                             - (ok_ref / att_ref if att_ref else 0.0)),
            'failed_per_scene': (sum(other[t]['failed'] for t in trials)
                                 - sum(reference[t]['failed'] for t in trials)) / len(trials),
        }

    point = stats(common)
    rng = random.Random(seed)
    draws = [stats([rng.choice(common) for _ in common]) for _ in range(n_boot)]
    return {'scenes': len(common),
            **{key: {'difference': value, 'ci95': _percentiles([d[key] for d in draws])}
               for key, value in point.items()}}


def _load(run):
    return tl.read(run / 'attempts.jsonl'), tl.read(run / 'scenes.jsonl')


def _fmt(rate):
    if rate['rate'] is None:
        return '—'
    return (f"{rate['successes']}/{rate['n']} ({rate['ci95'][0]:.2f}–{rate['ci95'][1]:.2f})")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()

    loaded = {run.name: _load(run) for run in args.runs}
    names = [run.name for run in args.runs]
    result = {'runs': {name: run_metrics(*loaded[name]) for name in names}, 'pairs': {}}
    reference = per_scene(*loaded[names[0]])
    for name in names[1:]:
        result['pairs'][f'{name} - {names[0]}'] = paired(reference, per_scene(*loaded[name]))

    for name in names:
        m = result['runs'][name]
        print(f"{name}: strategy {m['strategy']}, pick {_fmt(m['pick_success'])}, "
              f"named {_fmt(m['pick_success_named'])}, unknown {_fmt(m['pick_success_unknown'])}, "
              f"completion {_fmt(m['completion'])}, failures {m['failures']}")
    for pair, stats in result['pairs'].items():
        print(pair, json.dumps({k: v for k, v in stats.items()}, indent=None))
    if args.out:
        args.out.write_text(json.dumps(result, indent=2) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
