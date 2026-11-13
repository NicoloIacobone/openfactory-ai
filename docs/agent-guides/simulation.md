# Simulation

Applies when a task touches the simulator, worlds, object models, sensors, the ROS ↔ simulator bridge, `ros2_control` in simulation, or scene randomization.

## Choice

Gazebo Harmonic (Sim 8.15) through `ros-jazzy-ros-gz`, with the Ogre 1 render engine because the VM's GPU has no OpenGL 3.3 (D-005, `docs/environment.md`). Do not install a different Gazebo release or build it from source without a decision entry.

## Layout

- `openfactory_simulation/worlds/cell.sdf`: the world only (physics, user commands, scene broadcaster, sensors system with `ogre`, light, ground plane). Keep it small.
- Table, bins and camera body are one fixtures model spawned from `openfactory_description` (`fixtures.urdf.xacro`, geometry in `cell.yaml`, D-008); the robot is spawned from the same description that feeds `robot_state_publisher` and MoveIt.
- Objects: Gazebo models generated from the catalog `openfactory_description/config/objects.yaml` by `scripts/scene_objects.py` (D-009); simple collision shapes, correct mass and inertia.
- `openfactory_simulation/config/`: `bridge.yaml` (`ros_gz_bridge` mappings; `/clock` always bridged) and `ros2_controllers.yaml`.
- `openfactory_simulation/launch/simulation.launch.py`: arguments `world`, `headless`, `camera`, `depth_noise`, `seed` (seed of the depth noise).
- Sensor models between the bridge and perception, pure logic with pytest: `depth_noise.py`, node `scripts/depth_sensor_model.py` (D-011).

## Ring of N arms (phase 6, D-020)

- `ring_simulation.launch.py n:=N` (and `openfactory_bringup ring.launch.py`) starts N arms as one Gazebo model with `self_collide` next to the fixtures model (round table, one bin and one camera per arm). The world file is the cell's.
- Layout, controllers and bridge mappings of one N are generated at launch into `/tmp/openfactory_ring/n<N>/` (`openfactory_description/ring.py`, `openfactory_simulation/ring_config.py`) from `cell.yaml`, `ring.yaml` and the cell's `ros2_controllers.yaml` and `bridge.yaml`. Change the sources, never the generated files; write generated YAML with `ring.dump_yaml` (no anchors: `rcl` refuses them).
- Names: `arm_<i>_` prefixes links, joints and controllers; `/bin_<k>/camera/...` are the camera topics; one `depth_sensor_model` per camera in `/bin_<k>`.
- Objects for the ring are spawned by `scripts/relay_benchmark.py` (`relay_eval.spawn_pose`: bins are rotated about z); `gz_scene.py spawn` and `randomize_scene.py --spawn` know only the cell's two bins.
- Start it detached with `scripts/vm/launch.sh`, stop it with `scripts/vm/stop_cell.sh` (`docs/environment.md`).

## Integration

- Robot control through `gz_ros2_control` and standard controllers (joint trajectory controller, gripper controller); never set joint positions directly.
- RGB-D camera as a simulator sensor publishing image, depth and `camera_info` through the bridge; its pose is defined once (URDF) and appears in TF.
- `use_sim_time: true` everywhere.

## Ground truth

- Object poses are not bridged to ROS (D-009): only the harness reads them, from Gazebo Transport (`scripts/gz_scene.py`), for evaluation and debug labels. If a ROS ground-truth topic is ever needed, it lives under `/ground_truth/`.
- Pytests in `openfactory_perception` and `openfactory_manipulation` fail if a file of the package names ground truth, Gazebo Transport, the bridge or (perception) the ideal depth topic. Keep them passing; extend them to any new package that perceives or acts.

## Randomization

- `scripts/randomize_scene.py` draws a scene from (seed, scene index) and the scene parameters (command line, or the `scene` block of a benchmark config). The benchmark writes every generated object list to `scenes.jsonl` next to the attempt log.
- Same seed + same config + same code version → same scene. Record the git commit with each trial.
- Spawning happens before a trial starts; objects are not moved by the harness during a trial.
- Rest states: catalog orientation, or `tumbled` (cylinder on its side, prism on an end face) from `scripts/scene_objects.py` (`rest_rpy`, `resting_size`, `rest_error`). `tumbled_share` turns them on; without it the sampler draws exactly what it drew in phases 3 and 4.

## Resources (3.8 GB VM)

- Run headless (`headless:=true`) for trials; GUI only for debugging. With the camera sensor the server still needs the VM display (D-005); `camera:=false` removes the sensor for physics-only runs.
- Few objects, coarse physics step tuned only as needed, camera at modest resolution and rate.
- Measure RAM (`free -h`) whenever the simulation grows (another arm, camera or many objects) and record it in `docs/environment.md`.
- If a camera sensor fails to render, record the error before trying workarounds.
