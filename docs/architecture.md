# Architecture

Components, interfaces and frames. Everything here is **proposed** until the phase that implements it closes; changing an implemented interface needs explicit approval.

## System loop

```
simulated RGB-D camera + robot state
  → scene_perception     RGB/depth → objects, segmentation, 3D poses, confidence
  → grasp_selector       geometry + robot state → ranked feasible grasps
  → task_planner         MoveIt 2: IK, collisions, trajectory
  → robot_executor       controllers + gripper: pick and place
  → scene_manager        verify, update scene/task state, recover or continue
  → back to perception
```

Perception never commands joints. Simulator ground truth (`/ground_truth/...`) is consumed only by evaluation and debugging tools.

## Components

| Component | Package | Input | Output |
|---|---|---|---|
| `scene_perception` | `openfactory_perception` | RGB, depth, `camera_info`, TF | objects, segmentation, 3D poses, confidence |
| `grasp_selector` | `openfactory_manipulation` | object poses/geometry, point cloud, robot state | ranked feasible grasp candidates |
| `task_planner` | `openfactory_manipulation` | best grasp, destination bin, task state | MoveIt requests, next action |
| `robot_executor` | `openfactory_manipulation` | trajectories, gripper commands | execution result, state |
| `scene_manager` | `openfactory_manipulation` | prior state, perception update, action result | updated scene/task state, recovery request |
| `factory_manager` | later | production goals, station states | task assignments (not in the MVP) |

## Interfaces

To be defined in the phase that introduces each component. Standard types first (`sensor_msgs`, `geometry_msgs`, `vision_msgs`, `moveit_msgs`, `control_msgs`).

| Name | Type | Producer → consumer | Notes |
|---|---|---|---|
| — | — | — | — |

## Frames

To be fixed in phases 1 and 3. Conventions: REP 103 / REP 105, SI units, radians; camera data in `*_optical_frame`.

| Frame | Parent | Defined by | Notes |
|---|---|---|---|
| `world` | — | simulation | cell floor, z up |
