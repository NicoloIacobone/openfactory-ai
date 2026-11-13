# Perception

Applies when a task touches the camera, calibration, depth or point-cloud processing, segmentation, object pose estimation, or perception outputs.

## Contract

- **Inputs:** RGB image, depth image, `camera_info`, TF. Nothing else; in particular no ground truth and no simulator APIs.
- **Outputs:** object hypotheses with 3D pose, shape class, confidence (standard types first: `vision_msgs/Detection3DArray`, debug `sensor_msgs/PointCloud2`); never joint commands or trajectories. A cluster that matches no catalog shape is published as `unknown`, never dropped silently.
- Outputs are stamped with the image stamp and expressed in a documented frame (`world`, transformed via TF).

## Calibration and frames

- Intrinsics come from `camera_info`, never hard-coded. The measured principal-point correction lives in `perception.yaml` (D-011).
- Extrinsics come from TF (camera pose defined once in `cell.yaml`). Verify with `scripts/phase3_camera_check.py` after any change to the camera, its mount or the bridge.
- Depth units and invalid values (NaN, 0, out of range) are handled explicitly.
- Perception subscribes to the measured depth (`/camera/depth/image_raw`), never to the simulator's ideal depth (`/camera/depth/image_ideal`); `test_no_ground_truth.py` enforces it.
- Record camera frames and conventions in `docs/architecture.md`, "Frames".

## Geometric baseline (D-012)

1. Depth → points with K from `camera_info` → `input_bin` frame through TF at the image stamp.
2. Crop to the inner bin volume and remove the floor as the known plane z = 0.
3. Mean-height map on a 2 mm top-view grid (this is also the downsampling).
4. Morphological closing and opening, connected components, small blobs dropped.
5. Per blob: top height, minimum-area footprint rectangle, fill ratios; model-based recognition against each catalog shape, confidence exp(−residual / 2).

Each step is a pure function in `openfactory_perception/core` with pytest tests on synthetic views. The pipeline sees whatever is in the bin volume, the arm included: consumers take views with the arm at ready.

## Learned perception

- Only after the geometric baseline is measured on the same task (for the bin: E-003, E-004).
- Thesis-informed 3D/instance segmentation or multi-view representations are candidates; the question is whether they improve pick success over the baseline, measured on the same held-out seeds.
- Inference must fit the VM or run as a separate batch step; no training inside the simulation loop.

## Libraries

- NumPy and OpenCV only so far (D-012). Open3D has no apt package on this VM and its pip wheel for aarch64 / Python 3.12 is unverified: check before depending on it and record the result in `docs/environment.md`.
- New libraries go in a pinned requirements file and a decision entry.

## Evaluation

- Perception quality (instance/pose error, confidence calibration) is computed offline against ground truth logged by the trial harness, never fed back into the perception node.
