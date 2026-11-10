"""Trial log format shared by every benchmark producer (docs/agent-guides/benchmarking.md).

One JSON object per attempt in data/trials/<run_id>/attempts.jsonl (the VM writes to
~/openfactory_data/trials/), plus summary.json. ROS-free.
"""

import hashlib
import json
import math
import statistics
from pathlib import Path

REQUIRED_FIELDS = (
    'run_id', 'trial_id', 'attempt', 'git_commit', 'seed', 'config', 'strategy',
    'object_category', 'selected_grasp', 'candidates_total', 'candidates_rejected',
    'planner_result', 'execution_result', 'success', 'failure_reason',
    'perception_s', 'planning_s', 'execution_s', 'total_s',
)
TIMINGS = ('perception_s', 'planning_s', 'execution_s', 'total_s')


def config_hash(path):
    """First 12 hex digits of the file's sha256."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()[:12]


def config_ref(path):
    return {'path': str(path), 'sha256_12': config_hash(path)}


def git_commit(repo_root):
    """HEAD commit read from the .git directory (git itself runs only on the Mac)."""
    git_dir = Path(repo_root) / '.git'
    head = (git_dir / 'HEAD').read_text().strip()
    if not head.startswith('ref: '):
        return head
    ref = head[len('ref: '):]
    ref_file = git_dir / ref
    if ref_file.exists():
        return ref_file.read_text().strip()
    packed = git_dir / 'packed-refs'
    if packed.exists():
        for line in packed.read_text().splitlines():
            if line.endswith(' ' + ref):
                return line.split()[0]
    return 'unknown'


def validate(record):
    """Raise ValueError unless the record has every required field with a sane value."""
    missing = [name for name in REQUIRED_FIELDS if name not in record]
    if missing:
        raise ValueError(f'missing fields: {missing}')
    if not isinstance(record['success'], bool):
        raise ValueError('success must be a bool')
    if record['success'] != (record['failure_reason'] is None):
        raise ValueError('a record fails exactly when it has a failure_reason')
    if set(record['config']) != {'path', 'sha256_12'}:
        raise ValueError('config must be {path, sha256_12}')
    for name in TIMINGS:
        value = record[name]
        if value is not None and (not isinstance(value, (int, float)) or value < 0):
            raise ValueError(f'{name} must be None or a non-negative number')
    return record


def append(path, record):
    """Validate and append one record to a JSONL file."""
    validate(record)
    with open(path, 'a') as log:
        log.write(json.dumps(record) + '\n')


def read(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def wilson_interval(successes, n, z=1.96):
    """Wilson score interval for a binomial rate; (0, 1) when n == 0."""
    if n == 0:
        return 0.0, 1.0
    p = successes / n
    denom = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return max(0.0, centre - half), min(1.0, centre + half)


def _timing_stats(values):
    values = [v for v in values if v is not None]
    if not values:
        return None
    return {'mean': statistics.fmean(values), 'median': statistics.median(values),
            'min': min(values), 'max': max(values)}


def _count(values):
    counts = {}
    for value in values:
        if value is not None:
            counts[value] = counts.get(value, 0) + 1
    return counts


def summarize_group(records):
    n = len(records)
    successes = sum(r['success'] for r in records)
    low, high = wilson_interval(successes, n)
    return {
        'attempts': n,
        'successes': successes,
        'success_rate': successes / n if n else None,
        'success_rate_ci95': [low, high],
        'failures_by_reason': _count(r['failure_reason'] for r in records),
        'failures_by_stage': _count(r['execution_result'].get('failed_stage')
                                    for r in records if not r['success']),
        **{name: _timing_stats(r[name] for r in records) for name in TIMINGS},
    }


def summarize(records, group_by='object_category'):
    """Overall summary plus one per value of `group_by`."""
    groups = {}
    for record in records:
        groups.setdefault(record[group_by], []).append(record)
    return {
        'overall': summarize_group(records),
        f'by_{group_by}': {key: summarize_group(group) for key, group in sorted(groups.items())},
    }
