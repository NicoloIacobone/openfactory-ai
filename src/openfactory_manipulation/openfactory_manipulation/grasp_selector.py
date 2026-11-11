"""grasp_selector: object hypotheses -> ranked feasible grasps (phase 4, D-016).

Order (docs/agent-guides/manipulation.md): candidates are generated from geometry, hard filters
reject them with a reason, and only the survivors are ranked.
1. core.grasps: top-down candidates per object, then gripper width and the clearance of the
   fingers to the bin walls and to the other objects seen in the bin.
2. MoveIt, which has the last word: a collision-free IK solution above the grasp with the
   fingers at the approach opening, then the straight approach and lift lines, all checked
   against the planning scene (fixtures and every object of the view).
3. Baseline score on the survivors.
The planning scene must already hold the objects of the view (MoveItInterface.set_objects).
Nothing here reads camera data or simulator state: the input is perception's hypotheses.
"""

from . import constants as c
from .core import grasps
from .core import pick_place as core

NOT_EVALUATED = 'not_evaluated'
EXCLUDED = 'excluded'


class GraspSelector:

    def __init__(self, moveit, config, catalog, gripper):
        self.moveit = moveit
        self.cfg = config
        self.catalog = catalog
        self.gripper = gripper

    def _tool0(self, tcp_pose):
        return core.tool0_from_tcp(tcp_pose, self.gripper['tcp_offset'])

    def _moveit_filters(self, grasp, tcp):
        """(rejection reason or None, pre-grasp arm joints) for one candidate."""
        selection, motion = self.cfg['selection'], self.cfg['grasp']
        code, joints = self.moveit.compute_ik(
            self._tool0(core.raised(tcp, motion['approach_distance'])), self.cfg['ready_joints'],
            grasp.approach_position, selection['ik_timeout_s'])
        if code != core.MOVEIT_SUCCESS:
            return 'ik_failed', None
        above = self.moveit.robot_state(joints, grasp.approach_position)
        code, trajectory, _, _ = self.moveit.plan_cartesian(self._tool0(tcp), start_state=above)
        if code != core.MOVEIT_SUCCESS:
            return 'approach_blocked', None
        at_grasp = self.moveit.robot_state(self.moveit.end_joints(trajectory),
                                           grasp.approach_position)
        code, _, _, _ = self.moveit.plan_cartesian(
            self._tool0(core.raised(tcp, motion['lift_height'])), start_state=at_grasp)
        if code != core.MOVEIT_SUCCESS:
            return 'lift_blocked', None
        return None, joints

    def select(self, objects, bin_pose, inner_size, excluded=lambda grasp: False):
        """Rank the feasible grasps of a view.

        `objects`: core.grasps.SceneObject list in the frame of the bin at world pose
        `bin_pose`, of inner size `inner_size`; `excluded(grasp)` is True for candidates the
        caller does not want tried again. Returns {'ranked': [candidate], 'candidates':
        [candidate], 'candidates_total', 'candidates_rejected': {reason: count}}; a candidate
        is a dict with the grasp, its features, its rejection reason or score and, when
        feasible, the validated pre-grasp joints and the gripper_tcp world pose.
        """
        params = {**self.cfg['grasp'], **self.cfg['selection']}
        weights = self.cfg['ranking']
        pickable = [obj for obj in objects
                    if obj.class_id in self.catalog['objects']
                    and obj.score >= self.cfg['perception']['min_score']]
        candidates = []
        for obj in pickable:
            spec = self.catalog['objects'][obj.class_id]
            for grasp in grasps.generate(obj, spec, self.gripper, params):
                obstacles = grasps.obstacles_for(grasp, objects, self.catalog)
                reason, features = grasps.prefilter(grasp, self.gripper, inner_size, obstacles,
                                                    params)
                if reason is None and excluded(grasp):
                    reason = EXCLUDED
                clearance = min(features['wall_clearance'], features['neighbour_clearance'])
                candidates.append({
                    'grasp': grasp, 'object_score': obj.score, 'clearance': clearance,
                    'reason': reason, 'score': None, 'motion': None, 'pre_grasp_joints': None,
                    'tcp': None,
                    # cheapest first guess of the final score: motion is not known before IK
                    'prior': grasps.score(obj.score, clearance, 0.0, weights)})

        feasible = []
        for candidate in sorted((x for x in candidates if x['reason'] is None),
                                key=lambda x: -x['prior']):
            if len(feasible) >= self.cfg['selection']['max_feasible']:
                candidate['reason'] = NOT_EVALUATED
                continue
            grasp = candidate['grasp']
            tcp = grasps.tcp_pose(grasp, bin_pose)
            reason, joints = self._moveit_filters(grasp, tcp)
            candidate['reason'] = reason
            if reason is not None:
                continue
            motion = grasps.joint_distance(self.cfg['ready_joints'], joints)
            candidate.update(
                tcp=tcp, pre_grasp_joints=list(joints), motion=motion,
                score=grasps.score(candidate['object_score'], candidate['clearance'], motion,
                                   weights))
            feasible.append(candidate)

        rejected = {}
        for candidate in candidates:
            if candidate['reason'] not in (None, NOT_EVALUATED):
                rejected[candidate['reason']] = rejected.get(candidate['reason'], 0) + 1
        return {'ranked': sorted(feasible, key=lambda x: -x['score']),
                'candidates': candidates,
                'candidates_total': len(candidates),
                'candidates_rejected': rejected,
                'frame': c.INPUT_BIN_FRAME}


def candidate_record(candidate):
    """JSON-ready summary of a candidate for the trial log."""
    grasp = candidate['grasp']
    clearance = candidate['clearance']
    return {
        'object_index': grasp.object_index, 'class_id': grasp.class_id, 'label': grasp.label,
        'x': grasp.x, 'y': grasp.y, 'z': grasp.z, 'finger_yaw': grasp.finger_yaw,
        'width_m': grasp.width, 'object_score': candidate['object_score'],
        'clearance_m': None if clearance == float('inf') else clearance,
        'motion_rad': candidate['motion'], 'score': candidate['score'],
        'reason': candidate['reason'],
    }
