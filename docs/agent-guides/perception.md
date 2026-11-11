# Perception

Applies when a task touches the camera, calibration, depth or point-cloud processing, segmentation, object pose estimation, or perception outputs. Not before phase 3.

## Contract

- **Inputs:** RGB image, depth image, `camera_info`, TF. Nothing else; in particular no `/ground_truth/...` topics and no simulator APIs.
- **Outputs:** object hypotheses with 3D pose, size/shape class, confidence, and segmentation (standard types first, e.g. `vision_msgs/Detection3DArray`, `sensor_msgs/PointCloud2`); never joint commands or trajectories.
- Outputs are stamped with the image stamp and expressed in a documented frame (usually `world`, transformed via TF).

## Calibration and frames

- Intrinsics come from `camera_info`, never hard-coded.
- Extrinsics come from TF (camera pose defined in the description/world). Verify them: project a known point into the image and check it lands where expected.
- Depth units and invalid values (NaN, 0, max range) are handled explicitly.
- Perception subscribes to the measured depth (`/camera/depth/image_raw`), never to the simulator's ideal depth (`/camera/depth/image_ideal`); `test_no_ground_truth.py` enforces it.
- Verify with `scripts/phase3_camera_check.py` after any change to the camera, its mount or the bridge.
- Record camera frames and conventions in `docs/architecture.md`, "Frames".

## Geometric baseline (phase 3–4)

1. Depth → point cloud (organized when useful), crop to the input-bin volume.
2. Downsample (voxel grid) to keep CPU/RAM low.
3. Remove the bin floor and walls (plane fitting / known bin geometry from fixed infrastructure).
4. Segment objects (Euclidean clustering or similar).
5. Estimate pose and shape per cluster (PCA / oriented bounding box / primitive fitting) and a confidence.

Each step is a pure function with pytest tests on synthetic clouds.

## Learned perception (phase 5 only)

- Only after the geometric baseline is measured in `docs/experiments.md`.
- Thesis-informed 3D/instance segmentation or multi-view representations are candidates; the research question is whether they improve pick success over the baseline, measured on the same held-out seeds.
- Inference must fit the VM or run as a separate batch step; no training inside the simulation loop.

## Libraries

- OpenCV and NumPy first: the phase 3 baseline uses nothing else (D-012). Open3D has no apt package on this VM; pip availability for aarch64/Python 3.12 is unverified — check before depending on it and record the result in `docs/environment.md`.
- New libraries go in a pinned requirements file and a decision entry.

## Evaluation

- Perception quality (instance/pose error, confidence calibration) is computed offline against ground truth logged by the trial harness, never fed back into the perception node.
