"""ROS-free logic of the phase 1 reach test: goal sampling, errors, records, summary."""

import math
import random

from trial_log import config_hash, git_commit  # noqa: F401  (shared helpers, re-exported)

# moveit_msgs/MoveItErrorCodes values mapped to the project failure reasons; others are
# reported as moveit_error_<code>.
MOVEIT_SUCCESS = 1
_PLANNING_CODES = {
    -1,   # PLANNING_FAILED
    -2,   # INVALID_MOTION_PLAN
    -10,  # START_STATE_IN_COLLISION
    -11,  # START_STATE_VIOLATES_PATH_CONSTRAINTS
    -12,  # GOAL_IN_COLLISION
    -13,  # GOAL_VIOLATES_PATH_CONSTRAINTS
    -14,  # GOAL_CONSTRAINTS_VIOLATED
    -16,  # INVALID_GOAL_CONSTRAINTS
    -26,  # START_STATE_INVALID
    -27,  # GOAL_STATE_INVALID
    -31,  # NO_IK_SOLUTION
}
_EXECUTION_CODES = {
    -3,   # MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE
    -4,   # CONTROL_FAILED
    -7,   # PREEMPTED
}
_TIMEOUT_CODES = {-6}  # TIMED_OUT


def sample_joint_goal(rng: random.Random, joints, ranges):
    return [rng.uniform(*ranges[name]) for name in joints]


def max_abs_error(goal, final):
    return max(abs(g - f) for g, f in zip(goal, final, strict=True))


def failure_reason(moveit_code, final_error, tolerance, invalid_states):
    """None on success, otherwise the first failing stage."""
    if moveit_code != MOVEIT_SUCCESS:
        if moveit_code in _PLANNING_CODES:
            return 'planning_failed'
        if moveit_code in _EXECUTION_CODES:
            return 'execution_failed'
        if moveit_code in _TIMEOUT_CODES:
            return 'timeout'
        return f'moveit_error_{moveit_code}'
    if invalid_states > 0:
        return 'collision'
    if final_error is None or final_error > tolerance:
        return 'goal_error'
    return None


def summarize(records, goals_required):
    errors = [r['final_max_abs_error'] for r in records if r['final_max_abs_error'] is not None]
    reasons = {}
    for r in records:
        if r['failure_reason']:
            reasons[r['failure_reason']] = reasons.get(r['failure_reason'], 0) + 1
    successes = sum(r['success'] for r in records)
    return {
        'goals': len(records),
        'successes': successes,
        'failures_by_reason': reasons,
        'final_error_max': max(errors) if errors else None,
        'final_error_mean': sum(errors) / len(errors) if errors else None,
        'executed_states_checked': sum(r['executed_states_checked'] for r in records),
        'executed_states_invalid': sum(r['executed_states_invalid'] for r in records),
        'planning_s_mean': _mean(r['planning_s'] for r in records),
        'total_s_mean': _mean(r['total_s'] for r in records),
        'passed': len(records) == goals_required and successes == goals_required,
    }


def _mean(values):
    values = [v for v in values if v is not None and not math.isnan(v)]
    return sum(values) / len(values) if values else None
