# Manipulation

Applies when a task touches grasp generation or ranking, task planning, MoveIt 2, the planning scene, gripper control, execution, verification, or recovery.

## Components

| Component | In code (`openfactory_manipulation`) | Input | Output |
|---|---|---|---|
| `grasp_selector` | `grasp_selector.GraspSelector`, `core/grasps.py`, `core/ranking.py` | object hypotheses, robot state | ranked grasp candidates (only feasible ones) |
| `task_planner` + `robot_executor` | `pick_place.PickPlace`, `core/pick_place.py` | chosen grasp, place point | MoveIt requests, trajectories, gripper commands, cycle result |
| `scene_manager` | `bin_picking.BinPicking`, `core/task.py` | prior state, new view, cycle result | task state, next action, recovery |

Interfaces are recorded in `docs/architecture.md`; changing one needs the user's approval.

## Planning

- MoveIt 2 through the actions and services of `move_group` (`moveit_interface.py`): free-space moves are `/move_action` plans with `plan_only`, executed through `/execute_trajectory`; every move near or with an object is a straight `/compute_cartesian_path` line; `/compute_ik` for grasp feasibility; `/apply_planning_scene` for objects (D-010, D-016). MoveIt Task Constructor only if multi-stage tasks need it.
- Fixed infrastructure (table, bins, camera body, floor) is made of links of the cell URDF, not planning-scene objects (D-006, D-008, D-011). The planning-scene world holds objects only: those of the current view and those the cell placed, never from ground truth. It belongs to the loop, which empties it at every new scene: `move_group` keeps the objects of a process that ended (D-017).
- Grasped objects are **attached** to `gripper_tcp` on grasp and **detached** on release, with the gripper links as touch links.
- Trajectories run through controllers (`FollowJointTrajectory`); the gripper through `GripperCommand`. Never write joint states directly.

## Grasp selection

Order is fixed:

1. Generate candidates from geometry: top-down grasps on catalog shapes (D-016) and on `unknown` clusters, across the long and the short side of the footprint (D-018). Side grasps and antipodal pairs on point clouds do not exist yet.
2. **Hard filters:** gripper width, fingers and palm against walls and neighbours, then MoveIt: collision-free IK above the grasp, approach line, lift line. Rejected candidates are logged with the reason.
3. Rank the survivors with `ranking.strategy` (`core/ranking.py`): `learned`, the default, is a logistic P(success) minus a motion term (D-019); `baseline` is the hand-designed score of D-016; `uniform_random` is for data collection. A strategy only orders feasible candidates and cannot refuse one: never put a feasibility check in a score. `ranking.features` is the single source of the features for logs and inference: change it only together with a new training run.

## Pick and place sequence

pre-grasp → approach → close gripper → verify jaw opening → lift → (attach) → move to destination → release → (detach) → retreat → verify with the next view → update scene.

## Verification and recovery

- Success needs evidence from more than one source: jaw opening after closing and after lift and transfer, execution result, and the next view (the object is gone from the input bin). The output bin is not observed (one camera).
- Failures get a reason code logged per attempt. Cycle (`core/pick_place.py`): `scene_failed`, `ik_failed`, `planning_failed`, `execution_failed`, `gripper_failed`, `grasp_failed`, `grasp_slipped`, `place_blocked`, `timeout`. Loop (`core/task.py`): `object_not_removed`, `no_view`. Harness, from ground truth: `place_failed`, `collision`. Candidates are rejected with `gripper_width`, `wall_collision`, `neighbour_collision`, `ik_failed`, `approach_blocked`, `lift_blocked` (`core/grasps.py`), and a scene ends with one of the reasons in `core/task.py` (`bin_empty`, `no_feasible_grasp`, ...).
- Recovery: open, straight rise, back to ready, look again and select again; a failed grasp is not repeated; bounded attempts per object and per scene (D-017). Failed attempts never count as successes.

## Bin-picking loop

- `ros2 run openfactory_manipulation bin_picking` empties the input bin; `BinPicking.step()` is one attempt, `reset()` starts a new scene and empties the planning-scene world.
- Views are taken with the arm at ready only: perception sees whatever is in the bin volume. The wait for a view (`perception.timeout_s`) runs on the node's clock, not on wall time: the simulation may run slower than real time (E-006).
- A held prism can swing between the pads (D-015); do not raise its grasp.
- `unknown` clusters are tried only with `selection.unknown_grasps` (on by default since E-005); their width is measured, so the jaw checks use `selection.unknown_width_tolerance`. A standing prism held across the long side of its footprint is a wedge and slips out; across the short side it holds (D-018).
- `pick_place_once` (object at a known pose, D-009) and `pick_perceived_once` (most confident hypothesis, D-013) run single cycles for debugging.

## Relay loop of the ring (phase 6, D-021)

- `ros2 run openfactory_manipulation relay --arms N [--handovers K]` with `ring.launch.py n:=N` running. `RelayLoop.step()` is one turn, `reset()` parks every arm and empties the planning-scene world.
- An arm is driven through its `ArmNames` (`names.py`: `CELL`, `ring_arm(i)`): never write `c.ARM_GROUP`, `c.TOOL_LINK` or a joint name in code that an arm of the ring may run; use `moveit.names`. `MoveItInterface.for_arm` shares the node, TF, joint states and world ids between arms.
- One arm moves at a time; the others are at `relay.park_joints`. Do not send two arms to `ready_joints`: with N = 2 they meet at the centre of the ring (MoveIt refuses the second plan, and the simulator stops them).
- Place points and obstacles of the destination bin come from its view (`core/relay.free_slots`, `RelayLoop._destination`), not from memory. A handover needs the pick verified and the object seen at its place point (`object_not_arrived`).
- The object that waited longest in the source bin goes first (`core/relay.waiting_since`, passed to `grasp_selector.select` as `priority`): an order between objects set by the task, before the ranking. It must stay an order, never a filter.
- Loop end reasons: `handovers_done`, `no_work`, `failure_budget`, `park_failed`, `no_view` (`core/relay.py`).

## Rubik's cube cell (phase 7, D-022)

- Cube logic lives in `core/rubik/` (ROS-free): `cube.py` (54 facelets, moves, `check_solvable`), `cubies.py` (26 cubies with rotations), `orientation.py` (the cube's orientation in the nest; `turn_move`, `after_turn`, `after_flip`), `solver.py` (`kociemba`, imported inside `solve` only).
- The nest holds the bottom layer and the gripper the two layers above it, the core among them: a gripper turn of k quarter turns about the vertical makes the face that is down turn k quarter turns anticlockwise seen from outside, and turns the cube frame with the gripper (`orientation.physical_turn` is the geometry behind it, checked by pytest for the 24 orientations).
- The turn, the flip, the perception of the cube and the solve loop are not written yet.
