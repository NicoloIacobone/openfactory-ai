# Ideas

Candidate goals for later phases, written down before any plan. An idea becomes a phase only with a plan, a measurable closing test and the user's go-ahead; until then nothing here is a commitment. The rules of `CLAUDE.md` apply to all of it.

## 1. N-arm relay loop (first, the simple one)

**Update 2026-11-13:** the first step listed below (arms and bins, no conveyor) is built as the ring of N arms and closed as phase 6: `docs/architecture.md`, "Ring of N arms", D-020, D-021, E-006. One model and one MoveIt instance for every arm, one camera per bin, one arm moving at a time. What is left of this idea:

- Conveyors that carry the objects between the arms, instead of a bin shared by two neighbours.
- Arms moving at the same time. In the ring only arms that share no bin could (never with N = 2 or 3, whose arms all share one); it needs execution outside `move_group`'s single `ExecuteTrajectory`, and either plans checked against the other arm's trajectory or work zones that an arm must own before it enters them.
- A loop that keeps its objects. In E-006 objects drop out of the loop one by one: a prism set down at a corner place point with its length towards the end wall cannot be picked again (top-down fingers need room on that side), and two objects that touch after a slip are seen as one cluster too wide for the gripper. Ways out, none tried: choose the yaw (or the place point) at which an object is set down so that the next arm can grasp it; place points farther from the walls for the ring, fewer of them; a push or a regrasp that separates two touching objects or moves one off a wall. Then a run long enough to measure how long a ring lasts (hours or handovers until `no_work`).
- A lighter ring for large N. What limits N on this VM is not the arms but the Python nodes: each one handles the 800 Hz `/clock` and takes about a third of a core doing nothing else, and each bin adds two of them and 270 MB (`docs/environment.md`, "Phase 6 ring"): the simulation ran at 0.7–1.0 of real time with N = 2, 0.2–0.5 with N = 3, 0.3–0.4 with N = 4. Two ways out, neither tried: one perception process and one depth-model process serving every camera, or a `/clock` bridged at a lower rate (Gazebo Transport can throttle a subscription, `ros_gz_bridge` 1.0.24 cannot). The second touches every node of the cell too and needs a decision entry.

The idea as first written:

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

**Update 2026-11-14:** a first, simpler form of this idea is phase 7, closed on E-007 (plan: D-022 in `docs/decisions.md`): one arm, a fixed nest that holds the bottom layer and one overhead camera solve a cube that starts scrambled. The cube is a mechanism of 26 cubies driven by a Gazebo system plugin, the solver is `kociemba`. What is left of this idea after phase 7: the second arm that scrambles and the handover between the two, and turning a layer with two grippers instead of a nest.

Ideas that came up while building it (D-023 to D-027, E-007), none tried:

- A test of the recoveries the closing test never needed (E-007: nothing failed in 16 cubes): a development harness that disturbs the cube between two operations (a layer left out of square, a face turned behind the cell's back) and checks that the cell realigns, scans again and still ends right. Until then `realign` and the rescan of D-027 are untested code.
- The cube starting anywhere on the table or in a bin: the cell of phase 5 picks it (a 57 mm cube is a catalog shape away) and seats it in the nest.
- A faster scan: a second camera looking at the cube from the side would show three faces at once; today six views cost five flips, about 80 s.
- Fewer flips: a second nest that holds two layers would let the arm turn the top face as well, so two opposite faces per orientation; or choosing among equivalent solutions the one with the fewest changes of face.
- A cube that is less ideal inside: pieces that can pop out or twist, as contacts instead of welds (the alternative D-022 set aside).
- Reporting upstream the memory the Gazebo server loses every time a detachable joint is created and removed (`docs/environment.md`), with the probe that shows it.

The idea as first written:

The same relay idea with one object: an arm scrambles a Rubik's cube with seeded random turns and hands it over; the other arm reads the cube's state from the camera, solves it and hands it back; repeat.

Needs more thought before any plan. Hard parts known so far:

- A cube that can be turned in the simulator: a mechanism whose small cubes change face after every turn, not a rigid primitive. How to model it in Gazebo is the first question.
- Layer turns with parallel-jaw grippers: something holds the rest of the cube while a gripper turns one layer (a second arm, or a fixture), plus regrasps to reach every face.
- State from the camera: sticker colours on six faces, of which one fixed view sees at most three. Ogre 1 renders flat-shaded colour (D-005), which may be enough to tell six colours apart.
- Solver: the `doppelbot` project already scans a real cube and solves it with `kociemba` (a pip package, not apt); a new dependency needs a decision entry.
- The simulator's cube state is ground truth: evaluation only (rule 5).

## 3. Robot catalog (when an idea needs a different robot)

A workshop where robot types are tried and the ones that work are kept in an inventory, so that later ideas can pick from what is available, including odd mixes.

- A robot (arm + gripper + MoveIt configuration) enters the catalog only after a standard qualification test, the same for all: a reach test like E-001 and a pick-and-place test like E-002. Its results are recorded in `docs/experiments.md`, like any other result.
- What is available: on Jazzy aarch64 only the UR family has a full apt stack. The installed `ur_description` has 14 models (ur3 to ur30, e-series included), and the cell xacro already takes a `ur_type` argument (default `ur5e`). Franka has no apt package (`docs/environment.md`); other brands need checking one by one. So the catalog would start with UR variants and different grippers.
- Limits: the qualification runs one robot at a time, because the 3.8 GB VM holds one cell with a camera. Several agents could write configurations in parallel, but their tests queue on the same VM; truly parallel tests need another machine. A UR variant changes parameters (reach, ready joints, bin layout); another brand is a whole new description, MoveIt configuration and controller set.
- When: after the relay loop, which needs N identical UR5e arms and no catalog. A light first step is a cell parametric on `ur_type` with ur3e and ur10e qualified; a full workshop only when an idea needs a robot the catalog does not have (for example the Rubik's cube pair).
