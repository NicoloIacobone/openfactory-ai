# Ideas

Candidate goals for the phases after 5, written down before any plan. An idea becomes a phase only with a plan, a measurable closing test and the user's go-ahead; until then nothing here is a commitment. The rules of `CLAUDE.md` apply to all of it.

## 1. N-arm relay loop (first, the simple one)

N arms (N ≥ 2), each with its own conveyor. Arm *i* picks the objects that arrive on its conveyor and places them on the conveyor of arm *(i + 1) mod N*, forever: with N = 2, arm 0 feeds arm 1 and arm 1 feeds arm 0.

- Reuses the phase 5 cell for each arm: camera-driven perception, grasp selection with hard filters, pick and place, verification and recovery.
- New: conveyors that carry the objects physically (rule 1: nothing is teleported from one end to the other), several arms in one simulation and one planning problem, a pick area on a belt instead of a bin.
- Possible measures, to be fixed in the plan: objects moved per hour; handovers or hours until the first failure the cells do not recover from; objects lost off a belt; executed states in collision, arm against arm included; all from seeded runs.
- Open questions:
  - One MoveIt instance with every arm in one URDF (one planning group per arm), or one instance per arm in its own namespace.
  - Conveyor model in Gazebo Harmonic (for example the track controller system of its conveyor example; to be checked on this VM) and whether the belt stops for a pick.
  - One camera per pick area, or one camera over the whole cell.
  - Memory: each arm adds controllers, MoveIt state and possibly a camera to a 3.8 GB VM; measure with N = 2 before going higher.
  - A first step could be two arms with two bins and no conveyor, to keep the multi-arm problem apart from the conveyor problem.

## 2. Rubik's cube: one arm scrambles, one solves (later, the hard one)

The same relay idea with one object: an arm scrambles a Rubik's cube with seeded random turns and hands it over; the other arm reads the cube's state from the camera, solves it and hands it back; repeat.

Needs more thought before any plan. Hard parts known so far:

- A cube that can be turned in the simulator: a mechanism whose small cubes change face after every turn, not a rigid primitive. How to model it in Gazebo is the first question.
- Layer turns with parallel-jaw grippers: something holds the rest of the cube while a gripper turns one layer (a second arm, or a fixture), plus regrasps to reach every face.
- State from the camera: sticker colours on six faces, of which one fixed view sees at most three. Ogre 1 renders flat-shaded colour (D-005), which may be enough to tell six colours apart.
- Solver: the `doppelbot` project already scans a real cube and solves it with `kociemba` (a pip package, not apt); a new dependency needs a decision entry.
- The simulator's cube state is ground truth: evaluation only (rule 5).
