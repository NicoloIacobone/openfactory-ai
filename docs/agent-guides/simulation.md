# Simulation

Applies when a task touches the simulator, worlds, object models, sensors, the ROS ↔ simulator bridge, `ros2_control` in simulation, or scene randomization.

## Choice

Simulator and version are those recorded in `docs/environment.md` and `docs/decisions.md`. The candidate is Gazebo Harmonic via `ros-jazzy-ros-gz` (binary, aarch64); it is not installed until approved. Do not install a different Gazebo release or build it from source without a decision entry.

## Layout (`openfactory_simulation`)

- `worlds/`: SDF worlds. Keep them small: ground plane, cell table, input bin, destination bin, camera mount, light.
- `models/`: primitive objects (cube, cylinder, prism) with correct mass and inertia; simple collision shapes, visual meshes only if cheap.
- `config/bridge.yaml`: `ros_gz_bridge` topic mappings. Always bridge `/clock`.
- `launch/`: simulation launch with arguments `world`, `seed`, `headless`.

## Integration

- Robot control through `gz_ros2_control` and standard controllers (joint trajectory controller, gripper controller); never set joint positions directly.
- Robot description comes from `openfactory_description` (or the vendor package); the same URDF feeds the simulator, `robot_state_publisher` and MoveIt.
- RGB-D camera as a simulator sensor publishing image, depth, `camera_info` (and optionally points) through the bridge; its pose is defined once (URDF/SDF) and appears in TF.
- `use_sim_time: true` everywhere.

## Ground truth

- Object poses from the simulator are published only under `/ground_truth/...` and used for evaluation and debug labels.
- Perception and manipulation nodes must not subscribe to `/ground_truth/...`; keep a test or launch check that enforces it when those nodes exist.

## Randomization

- `scripts/randomize_scene.py` (when created) takes an explicit seed and config file, and writes the generated object list (type, pose) next to the trial logs.
- Same seed + same config + same code version → same scene. Record the git commit with each trial.
- Spawning happens before a trial starts; objects are not moved by the harness during a trial.

## Resources (3.8 GB VM)

- Run headless (`gz sim -s` or the launch `headless:=true`) for trials; GUI only for debugging.
- Few objects, coarse physics step tuned only as needed, camera at modest resolution and rate.
- Measure RAM (`free -h`) with simulator + MoveIt + RViz the first time they run together and record it in `docs/environment.md`.
- Rendering uses the VM's virtio GPU (Mesa); if camera sensors fail to render, record the error before trying workarounds.
