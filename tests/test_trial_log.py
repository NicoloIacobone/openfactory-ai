import pytest

import trial_log as tl


def _record(**overrides):
    record = {
        'run_id': 'r', 'trial_id': 0, 'attempt': 0, 'git_commit': 'abc', 'seed': 1,
        'config': {'path': 'c.yaml', 'sha256_12': '0123456789ab'}, 'strategy': 's',
        'object_category': 'cube', 'selected_grasp': {}, 'candidates_total': 1,
        'candidates_rejected': {}, 'planner_result': {}, 'execution_result': {},
        'success': True, 'failure_reason': None,
        'perception_s': None, 'planning_s': 0.3, 'execution_s': 19.0, 'total_s': 20.0,
    }
    record.update(overrides)
    return record


def test_validate_accepts_a_complete_record():
    assert tl.validate(_record()) is not None


@pytest.mark.parametrize('overrides', [
    {'success': 1},
    {'success': False},                                   # failure without a reason
    {'failure_reason': 'grasp_failed'},                   # success with a reason
    {'config': {'path': 'c.yaml'}},
    {'planning_s': -1.0},
    {'total_s': 'fast'},
])
def test_validate_rejects_bad_records(overrides):
    with pytest.raises(ValueError):
        tl.validate(_record(**overrides))


def test_validate_reports_missing_fields():
    record = _record()
    del record['seed']
    with pytest.raises(ValueError, match='seed'):
        tl.validate(record)


def test_append_and_read_round_trip(tmp_path):
    path = tmp_path / 'attempts.jsonl'
    tl.append(path, _record(trial_id=0))
    tl.append(path, _record(trial_id=1))
    assert [r['trial_id'] for r in tl.read(path)] == [0, 1]
    with pytest.raises(ValueError):
        tl.append(path, _record(success=False))
    assert len(tl.read(path)) == 2


def test_wilson_interval():
    low, high = tl.wilson_interval(10, 10)
    assert high == pytest.approx(1.0) and low == pytest.approx(0.7225, abs=1e-4)
    low, high = tl.wilson_interval(5, 10)
    assert (low, high) == pytest.approx((0.2366, 0.7634), abs=1e-4)
    assert tl.wilson_interval(0, 0) == (0.0, 1.0)


def test_summarize_by_category():
    records = [
        _record(object_category='cube'),
        _record(object_category='cube', success=False, failure_reason='grasp_slipped',
                execution_result={'failed_stage': 'hold_after_lift'}, total_s=12.0),
        _record(object_category='prism', perception_s=None),
    ]
    summary = tl.summarize(records)
    assert summary['overall']['attempts'] == 3 and summary['overall']['successes'] == 2
    cube = summary['by_object_category']['cube']
    assert cube['success_rate'] == 0.5
    assert cube['failures_by_reason'] == {'grasp_slipped': 1}
    assert cube['failures_by_stage'] == {'hold_after_lift': 1}
    assert cube['total_s']['median'] == pytest.approx(16.0)
    assert cube['perception_s'] is None


def test_git_commit_reads_refs(tmp_path):
    git = tmp_path / '.git'
    (git / 'refs' / 'heads').mkdir(parents=True)
    (git / 'HEAD').write_text('ref: refs/heads/main\n')
    (git / 'refs' / 'heads' / 'main').write_text('abc123\n')
    assert tl.git_commit(tmp_path) == 'abc123'
    (git / 'refs' / 'heads' / 'main').unlink()
    (git / 'packed-refs').write_text('# pack-refs\ndef456 refs/heads/main\n')
    assert tl.git_commit(tmp_path) == 'def456'
    (git / 'HEAD').write_text('fedcba\n')
    assert tl.git_commit(tmp_path) == 'fedcba'


def test_config_ref(tmp_path):
    path = tmp_path / 'c.yaml'
    path.write_text('a: 1\n')
    ref = tl.config_ref(path)
    assert ref['path'] == str(path) and len(ref['sha256_12']) == 12


def _scene(**overrides):
    record = {
        'run_id': 'r', 'trial_id': 0, 'git_commit': 'abc', 'seed': 1,
        'config': {'path': 'c.yaml', 'sha256_12': '0123456789ab'}, 'strategy': 's',
        'truth': [{'name': 'cube_0'}], 'detections': [{'class_id': 'cube'}],
        'n_objects': 1, 'n_detections': 1, 'matches': [], 'missed': [], 'false_positives': [],
        'success': True, 'failure_reason': None, 'perception_s': 0.2,
    }
    record.update(overrides)
    return record


def test_perception_record_is_validated_and_appended(tmp_path):
    path = tmp_path / 'scenes.jsonl'
    tl.append(path, _scene(), tl.validate_perception)
    assert tl.read(path) == [_scene()]


@pytest.mark.parametrize('overrides', [
    {'n_objects': 2},
    {'n_detections': 0},
    {'success': False},
    {'perception_s': -0.1},
])
def test_perception_record_rejected(overrides):
    with pytest.raises(ValueError):
        tl.validate_perception(_scene(**overrides))


def test_perception_record_reports_missing_fields():
    record = _scene()
    del record['truth']
    with pytest.raises(ValueError, match='truth'):
        tl.validate_perception(record)
