# Manipulation

Applies when a task touches grasp generation or ranking, task planning, MoveIt 2, the planning scene, gripper control, execution, verification, or recovery.

## Components

| Component | Input | Output |
|---|---|---|
| `grasp_selector` | object hypotheses, point cloud, robot state | ranked grasp candidates (only feasible ones) |
| `task_planner` | best grasp, destination bin, task state | MoveIt planning requests, next action |
| `robot_executor` | trajectories, gripper commands | execution result and state |
| `scene_manager` | prior state, perception update, action result | updated scene/task state, recovery request |

Interfaces are recorded in `docs/architecture.md`; changing one needs the user's approval.

## Planning

- MoveIt 2 when the audited environment supports it (planning groups, kinematics, controllers in `openfactory_moveit_config`). Start with the standard APIs (`moveit_py` or MoveGroup interface); add MoveIt Task Constructor only when multi-stage tasks need it.
- The planning scene contains the table, both bins and the camera mount as collision objects (fixed infrastructure may be known), plus perceived objects from perception — not from ground truth.
- Grasped objects are **attached** to the end effector in the planning scene on grasp and **detached** on release; the gripper's allowed collisions with the attached object are set explicitly.
- Trajectories run through controllers (`FollowJointTrajectory`); the gripper through its controller/action. Never write joint states directly.

## Grasp selection

Order is fixed:

1. Generate candidates from geometry (top-down and side grasps on primitives; antipodal pairs on point clouds).
2. **Hard filters:** gripper width/geometry, IK solution, reachability, collision-free pre-grasp/grasp/lift/retreat, trajectory feasibility. Rejected candidates are logged with the reason.
3. Rank the survivors. Baseline: hand-designed score. Later objective, terms normalized:
   `J(g) = λ1·success(g) − λ2·collision(g) − λ3·motion_cost(g) − λ4·occlusion(g)`.
   Collision and safety stay hard filters, never only penalties.

## Pick and place sequence

pre-grasp → approach → close gripper → verify contact → lift → (attach) → move to destination → release → (detach) → retreat → verify → update scene.

## Verification and recovery

- Success needs evidence from more than one source: gripper state (width/effort), execution result, and the scene after the action (perception sees the object gone from the input bin / in the destination bin).
- Failures get a reason code (`no_detection`, `no_feasible_grasp`, `ik_failed`, `planning_failed`, `execution_failed`, `grasp_slipped`, `place_failed`, `timeout`, ...) logged per attempt.
- Recovery: re-perceive and replan, try the next candidate, back off and retry; give up after a bounded number of attempts and record it. Failed attempts never count as successes.

## Phase 2 note

Deterministic pick/place uses a known object pose as fixed infrastructure input, documented as such; it is the baseline that camera-driven picking is compared against later.
