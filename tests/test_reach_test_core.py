import random

import pytest

import reach_test_core as core

JOINTS = ['a', 'b']
RANGES = {'a': [-1.0, 1.0], 'b': [0.0, 0.5]}


def test_sampling_is_seeded_and_in_range():
    first = [core.sample_joint_goal(random.Random(7), JOINTS, RANGES) for _ in range(2)]
    assert first[0] == first[1]
    rng = random.Random(3)
    for _ in range(100):
        a, b = core.sample_joint_goal(rng, JOINTS, RANGES)
        assert -1.0 <= a <= 1.0 and 0.0 <= b <= 0.5


def test_max_abs_error():
    assert core.max_abs_error([0.0, 1.0], [0.002, 0.995]) == pytest.approx(0.005)
    with pytest.raises(ValueError):
        core.max_abs_error([0.0], [0.0, 1.0])


@pytest.mark.parametrize('code, error, invalid, expected', [
    (1, 0.001, 0, None),
    (1, 0.05, 0, 'goal_error'),
    (1, None, 0, 'goal_error'),
    (1, 0.001, 2, 'collision'),
    (-1, None, 0, 'planning_failed'),
    (-12, None, 0, 'planning_failed'),
    (-4, 0.3, 0, 'execution_failed'),
    (-6, None, 0, 'timeout'),
    (-25, None, 0, 'moveit_error_-25'),
])
def test_failure_reason(code, error, invalid, expected):
    assert core.failure_reason(code, error, 0.01, invalid) == expected


def _record(success, error, reason=None):
    return {'success': success, 'final_max_abs_error': error, 'failure_reason': reason,
            'executed_states_checked': 10, 'executed_states_invalid': 0,
            'planning_s': 0.1, 'total_s': 5.0}


def test_summary_passes_only_with_all_goals_successful():
    ok = [_record(True, 0.002), _record(True, 0.004)]
    summary = core.summarize(ok, 2)
    assert summary['passed'] and summary['final_error_max'] == pytest.approx(0.004)
    assert summary['executed_states_checked'] == 20
    assert not core.summarize(ok, 3)['passed']

    mixed = ok + [_record(False, None, 'planning_failed')]
    summary = core.summarize(mixed, 3)
    assert not summary['passed']
    assert summary['failures_by_reason'] == {'planning_failed': 1}


def test_git_commit_reads_refs(tmp_path):
    git = tmp_path / '.git'
    (git / 'refs' / 'heads').mkdir(parents=True)
    (git / 'HEAD').write_text('ref: refs/heads/main\n')
    (git / 'refs' / 'heads' / 'main').write_text('abc123\n')
    assert core.git_commit(tmp_path) == 'abc123'

    (git / 'refs' / 'heads' / 'main').unlink()
    (git / 'packed-refs').write_text('# pack-refs\ndef456 refs/heads/main\n')
    assert core.git_commit(tmp_path) == 'def456'

    (git / 'HEAD').write_text('fedcba\n')
    assert core.git_commit(tmp_path) == 'fedcba'
