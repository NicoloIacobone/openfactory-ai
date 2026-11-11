"""Thin rclpy wrapper over move_group, the planning scene and the gripper controller.

Every call blocks by spinning the node it was given, so the owner must not spin that node
elsewhere. Planning and execution are separate calls, so their times are measured separately.
"""

import time

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject, CollisionObject, Constraints, JointConstraint,
    OrientationConstraint, PlanningScene, PositionConstraint, RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

from . import constants as c
from .core.objects import bounding_box
from .core.pick_place import MOVEIT_SUCCESS


class ManipulationError(Exception):
    """A request that could not even be sent (server missing, timeout)."""


def to_pose(pose):
    (x, y, z), (qx, qy, qz, qw) = pose
    return Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))


def collision_object(object_id, spec, pose, operation=CollisionObject.ADD):
    """CollisionObject for a catalog primitive at a world pose (the prism as its bounding box)."""
    obj = CollisionObject(id=object_id, operation=operation)
    obj.header.frame_id = c.WORLD_FRAME
    obj.pose = to_pose(pose)
    if spec['shape'] == 'cylinder':
        shape = SolidPrimitive(type=SolidPrimitive.CYLINDER,
                               dimensions=[spec['length'], spec['radius']])
    else:
        shape = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=bounding_box(spec))
    obj.primitives = [shape]
    obj.primitive_poses = [Pose(orientation=Quaternion(w=1.0))]
    return obj


class MoveItInterface:

    def __init__(self, node, config):
        self.node = node
        self.cfg = config
        self.latest_joints = None
        self.tf_buffer = Buffer()
        # no spin thread: TF arrives while this node spins in the blocking calls
        self.tf_listener = TransformListener(self.tf_buffer, node, spin_thread=False)
        self.joint_sub = node.create_subscription(
            JointState, c.JOINT_STATES_TOPIC, self._on_joint_state, 10)
        self.move_client = ActionClient(node, MoveGroup, c.MOVE_ACTION)
        self.execute_client = ActionClient(node, ExecuteTrajectory, c.EXECUTE_ACTION)
        self.gripper_client = ActionClient(node, GripperCommand, c.GRIPPER_ACTION)
        self.cartesian_client = node.create_client(GetCartesianPath, c.CARTESIAN_SERVICE)
        self.scene_client = node.create_client(ApplyPlanningScene, c.PLANNING_SCENE_SERVICE)
        self.ik_client = node.create_client(GetPositionIK, c.IK_SERVICE)
        self.names = {
            self.move_client: c.MOVE_ACTION, self.execute_client: c.EXECUTE_ACTION,
            self.gripper_client: c.GRIPPER_ACTION, self.cartesian_client: c.CARTESIAN_SERVICE,
            self.scene_client: c.PLANNING_SCENE_SERVICE, self.ik_client: c.IK_SERVICE,
        }
        self.world_ids = set()        # collision objects put in the world by set_objects

    # -- plumbing -------------------------------------------------------------------------

    def _on_joint_state(self, msg):
        self.latest_joints = dict(zip(msg.name, msg.position))

    def _wait(self, future, timeout):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done():
            raise ManipulationError(f'no response within {timeout} s')
        return future.result()

    def _action(self, client, goal):
        """Send an action goal and wait for the result; returns the result message."""
        timeout = self.cfg['timeouts']['action_s']
        handle = self._wait(client.send_goal_async(goal), self.cfg['timeouts']['server_s'])
        if not handle.accepted:
            raise ManipulationError(f'goal rejected by {self.names[client]}')
        return self._wait(handle.get_result_async(), timeout).result

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait_ready(self):
        timeout = self.cfg['timeouts']['server_s']
        for client in (self.move_client, self.execute_client, self.gripper_client):
            if not client.wait_for_server(timeout_sec=timeout):
                raise ManipulationError(f'{self.names[client]} not available: is the cell up?')
        for client in (self.cartesian_client, self.scene_client, self.ik_client):
            if not client.wait_for_service(timeout_sec=timeout):
                raise ManipulationError(f'{self.names[client]} not available')
        end = time.monotonic() + timeout
        while self.latest_joints is None and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.latest_joints is None:
            raise ManipulationError(f'no {c.JOINT_STATES_TOPIC}')

    def frame_pose(self, frame):
        """(xyz, quat) of `frame` in the world frame, from TF."""
        timeout = self.cfg['timeouts']['server_s']
        end = time.monotonic() + timeout
        while not self.tf_buffer.can_transform(c.WORLD_FRAME, frame, rclpy.time.Time()):
            if time.monotonic() > end:
                raise ManipulationError(f'no TF {c.WORLD_FRAME} -> {frame}')
            rclpy.spin_once(self.node, timeout_sec=0.1)
        t = self.tf_buffer.lookup_transform(c.WORLD_FRAME, frame, rclpy.time.Time()).transform
        return ((t.translation.x, t.translation.y, t.translation.z),
                (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))

    def finger_position(self):
        return None if self.latest_joints is None else self.latest_joints.get(c.FINGER_JOINT)

    def jaw_opening(self):
        """Left + right finger position (m): the distance between the pads."""
        if self.latest_joints is None:
            return None
        values = [self.latest_joints.get(name) for name in c.FINGER_JOINTS]
        return None if None in values else sum(values)

    def arm_positions(self):
        return [self.latest_joints[name] for name in c.ARM_JOINTS]

    # -- planning and execution -----------------------------------------------------------

    def _motion_request(self, goal):
        p = self.cfg['planning']
        msg = MoveGroup.Goal()
        r = msg.request
        r.group_name = c.ARM_GROUP
        r.pipeline_id = p['pipeline']
        r.num_planning_attempts = p['attempts']
        r.allowed_planning_time = p['planning_time_s']
        r.max_velocity_scaling_factor = p['velocity_scaling']
        r.max_acceleration_scaling_factor = p['acceleration_scaling']
        r.start_state.is_diff = True
        r.goal_constraints = [goal]
        msg.planning_options.plan_only = True
        msg.planning_options.planning_scene_diff.is_diff = True
        msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        return msg

    def plan_joints(self, positions):
        """Plan to an arm joint configuration: (error code, trajectory, planning s)."""
        tol = self.cfg['planning']['joint_tolerance']
        goal = Constraints(joint_constraints=[
            JointConstraint(joint_name=name, position=float(value), tolerance_above=tol,
                            tolerance_below=tol, weight=1.0)
            for name, value in zip(c.ARM_JOINTS, positions)])
        return self._plan(goal)

    def plan_pose(self, tool0_pose):
        """Plan to a tool0 pose in the world frame: (error code, trajectory, planning s)."""
        p = self.cfg['planning']
        pose = to_pose(tool0_pose)
        position = PositionConstraint(link_name=c.TOOL_LINK, weight=1.0)
        position.header.frame_id = c.WORLD_FRAME
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[p['position_tolerance']])
        position.constraint_region.primitives = [sphere]
        position.constraint_region.primitive_poses = [Pose(position=pose.position,
                                                           orientation=Quaternion(w=1.0))]
        orientation = OrientationConstraint(
            link_name=c.TOOL_LINK, orientation=pose.orientation, weight=1.0,
            absolute_x_axis_tolerance=p['orientation_tolerance'],
            absolute_y_axis_tolerance=p['orientation_tolerance'],
            absolute_z_axis_tolerance=p['orientation_tolerance'])
        orientation.header.frame_id = c.WORLD_FRAME
        return self._plan(Constraints(position_constraints=[position],
                                      orientation_constraints=[orientation]))

    def _plan(self, goal):
        start = time.monotonic()
        result = self._action(self.move_client, self._motion_request(goal))
        return result.error_code.val, result.planned_trajectory, time.monotonic() - start

    @staticmethod
    def robot_state(arm_joints, finger_position=None):
        """The current robot state with the arm at `arm_joints` and, if given, both fingers at
        `finger_position`: for requests about a state the robot is not in."""
        names = list(c.ARM_JOINTS)
        positions = [float(v) for v in arm_joints]
        if finger_position is not None:
            names += list(c.FINGER_JOINTS)
            positions += [float(finger_position)] * len(c.FINGER_JOINTS)
        state = RobotState(is_diff=True)
        state.joint_state.name = names
        state.joint_state.position = positions
        return state

    def compute_ik(self, tool0_pose, seed_joints, finger_position, timeout_s):
        """Collision-free arm configuration for a tool0 pose in the world frame, searched from
        `seed_joints` with the fingers at `finger_position`: (error code, joints or None)."""
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = c.ARM_GROUP
        ik.robot_state = self.robot_state(seed_joints, finger_position)
        ik.avoid_collisions = True
        ik.ik_link_name = c.TOOL_LINK
        ik.pose_stamped.header.frame_id = c.WORLD_FRAME
        ik.pose_stamped.pose = to_pose(tool0_pose)
        ik.timeout = Duration(seconds=timeout_s).to_msg()
        response = self._wait(self.ik_client.call_async(request),
                              self.cfg['timeouts']['server_s'])
        code = response.error_code.val
        if code != MOVEIT_SUCCESS:
            return code, None
        solution = dict(zip(response.solution.joint_state.name,
                            response.solution.joint_state.position))
        return code, [solution[name] for name in c.ARM_JOINTS]

    @staticmethod
    def end_joints(trajectory):
        """Arm configuration at the end of a planned trajectory."""
        joint_trajectory = trajectory.joint_trajectory
        last = dict(zip(joint_trajectory.joint_names, joint_trajectory.points[-1].positions))
        return [last[name] for name in c.ARM_JOINTS]

    def plan_cartesian(self, tool0_pose, start_state=None):
        """Straight tool0 line from the current state, or from `start_state` (robot_state):
        (error code, trajectory, fraction, s)."""
        cart = self.cfg['cartesian']
        request = GetCartesianPath.Request(
            group_name=c.ARM_GROUP, link_name=c.TOOL_LINK, waypoints=[to_pose(tool0_pose)],
            max_step=cart['max_step'], jump_threshold=0.0, avoid_collisions=True,
            max_velocity_scaling_factor=cart['velocity_scaling'],
            max_acceleration_scaling_factor=cart['acceleration_scaling'])
        request.header.frame_id = c.WORLD_FRAME
        if start_state is None:
            request.start_state.is_diff = True
        else:
            request.start_state = start_state
        start = time.monotonic()
        response = self._wait(self.cartesian_client.call_async(request),
                              self.cfg['timeouts']['action_s'])
        code = response.error_code.val
        if code == MOVEIT_SUCCESS and response.fraction < cart['min_fraction']:
            code = -1  # PLANNING_FAILED: only part of the line is feasible
        return code, response.solution, response.fraction, time.monotonic() - start

    def execute(self, trajectory):
        """Execute through move_group and the controllers: (error code, execution s)."""
        start = time.monotonic()
        result = self._action(self.execute_client, ExecuteTrajectory.Goal(trajectory=trajectory))
        return result.error_code.val, time.monotonic() - start

    # -- gripper --------------------------------------------------------------------------

    def gripper(self, position, max_effort):
        """Command the left finger position: (result message, seconds)."""
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        start = time.monotonic()
        result = self._action(self.gripper_client, goal)
        self.spin_for(self.cfg['grasp']['settle_s'])
        return result, time.monotonic() - start

    # -- planning scene -------------------------------------------------------------------

    def _apply(self, scene):
        scene.is_diff = True
        scene.robot_state.is_diff = True
        response = self._wait(self.scene_client.call_async(ApplyPlanningScene.Request(scene=scene)),
                              self.cfg['timeouts']['server_s'])
        return response.success

    def add_object(self, object_id, spec, pose):
        scene = PlanningScene()
        scene.world.collision_objects = [collision_object(object_id, spec, pose)]
        return self._apply(scene)

    def set_objects(self, objects):
        """Make the world hold exactly `objects` among those this method manages.

        `objects`: {id: (spec, world pose)}, spec as in the catalog (`shape` and its sizes).
        Objects added before and missing now are removed; add_object and remove_object keep
        working on single ids. Returns True if every new object was accepted.
        """
        for object_id in self.world_ids - set(objects):
            remove = PlanningScene()
            remove.world.collision_objects = [CollisionObject(id=object_id,
                                                              operation=CollisionObject.REMOVE)]
            self._apply(remove)       # fails only if the object is already gone
        self.world_ids = set(objects)
        if not objects:
            return True
        scene = PlanningScene()
        scene.world.collision_objects = [collision_object(object_id, spec, pose)
                                         for object_id, (spec, pose) in objects.items()]
        return self._apply(scene)

    def clear_objects(self):
        """Empty the planning-scene world and the gripper, whoever filled them.

        The world of this cell holds objects only (fixtures are links of the robot model,
        D-008), and move_group keeps them when the process that added them ends: a new scene
        must not inherit the obstacles of an earlier run. An empty id means every object.
        """
        detach = PlanningScene()
        detach.robot_state.attached_collision_objects = [
            AttachedCollisionObject(object=CollisionObject(operation=CollisionObject.REMOVE))]
        remove = PlanningScene()
        remove.world.collision_objects = [CollisionObject(operation=CollisionObject.REMOVE)]
        detached = self._apply(detach)
        removed = self._apply(remove)
        self.world_ids = set()
        return detached and removed

    def attach_object(self, object_id):
        """Move a world object onto the gripper; the gripper links may touch it.

        Attaching by id an object that exists in the world moves it out of the world, so the
        diff must not also remove it (that second step would fail the whole request).
        """
        attached = AttachedCollisionObject(link_name=c.TCP_LINK,
                                           touch_links=list(c.GRIPPER_TOUCH_LINKS))
        attached.object.id = object_id
        attached.object.operation = CollisionObject.ADD
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects = [attached]
        return self._apply(scene)

    def remove_object(self, object_id):
        """Detach the object if attached, then remove it from the world.

        Returns True if either step succeeded: MoveIt reports a failure for whichever of the two
        finds nothing to remove (detaching puts the object back into the world first).
        """
        attached = AttachedCollisionObject(link_name=c.TCP_LINK)
        attached.object.id = object_id
        attached.object.operation = CollisionObject.REMOVE
        detach = PlanningScene()
        detach.robot_state.attached_collision_objects = [attached]
        remove = PlanningScene()
        remove.world.collision_objects = [CollisionObject(id=object_id,
                                                          operation=CollisionObject.REMOVE)]
        detached = self._apply(detach)
        removed = self._apply(remove)
        self.world_ids.discard(object_id)
        return detached or removed
