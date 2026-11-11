#!/usr/bin/env python3
"""Phase 3 closing test, part A: camera calibration check.

Verifies that what the camera publishes agrees with what the cell description says about it:
1. intrinsics: camera_info K against the pinhole model of camera.yaml, image size and frame id;
2. extrinsics, rotation and height: the floor of the empty input bin, seen in the ideal depth
   image and transformed with TF, must be the plane z = 0 of the input_bin frame;
3. extrinsics, position: the top-face centre of a cube at a known pose, projected into the
   image with TF and K, must land on the centre of the top face seen in the depth image. The
   offset is reported with the K of camera_info as it is (this is where a principal-point
   convention shows up) and with the calibration correction perception applies;
4. depth model: the noise of the measured depth on the bin floor against the model.

This is a harness tool: it reads the simulator's ideal depth and ground-truth poses, which the
perception node never sees. Logs: ~/openfactory_data/trials/<run_id>/camera_check.json.

Run in the VM with the cell launched (camera on):
    python3 scripts/phase3_camera_check.py [--config FILE]
"""

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from openfactory_manipulation.core.geometry import quat_rotate
from openfactory_perception.core import camera
from openfactory_simulation.depth_noise import depth_sigma
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

import phase3_common as common
import scene_objects as so
import trial_log as tl
from gz_scene import GzScene, load_config

REPO = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO / 'scripts' / 'config' / 'phase3_perception.yaml'
OPTICAL_FRAME = 'camera_optical_frame'
TOPICS = {'info': ('/camera/camera_info', CameraInfo),
          'ideal': ('/camera/depth/image_ideal', Image),
          'measured': ('/camera/depth/image_raw', Image)}


def depth_array(msg):
    return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, -1)[:, :msg.width]


def lookup(node, buffer, target, source, timeout_s=10.0):
    end = time.monotonic() + timeout_s
    while not buffer.can_transform(target, source, Time()):
        if time.monotonic() > end:
            raise RuntimeError(f'no TF {target} <- {source}')
        rclpy.spin_once(node, timeout_sec=0.1)
    t = buffer.lookup_transform(target, source, Time()).transform
    return camera.pose_matrix((t.translation.x, t.translation.y, t.translation.z),
                              (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))


def bin_floor(depth, k, bin_from_optical, inner, margin):
    """(points in the bin frame, their optical z) of the pixels over the inner bin floor."""
    optical, _ = camera.depth_to_points(depth, k, 0.05, 5.0)
    points = camera.transform_points(bin_from_optical, optical)
    mask = ((np.abs(points[:, 0]) < inner[0] / 2 - margin)
            & (np.abs(points[:, 1]) < inner[1] / 2 - margin) & (np.abs(points[:, 2]) < 0.02))
    return points[mask], optical[mask, 2]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--out', type=Path, default=Path.home() / 'openfactory_data' / 'trials')
    args = parser.parse_args()

    cfg = yaml.safe_load(args.config.read_text())['calibration']
    limits = cfg['thresholds']
    catalog, cell = load_config()
    camera_cfg = yaml.safe_load(
        (Path(get_package_share_directory('openfactory_description')) / 'config' / 'camera.yaml')
        .read_text())
    image, depth_cfg = camera_cfg['image'], camera_cfg['depth']
    correction = yaml.safe_load(
        (Path(get_package_share_directory('openfactory_perception')) / 'config'
         / 'perception.yaml').read_text())['principal_point_offset']
    inner = cell['bins']['input']['inner_size']
    run_id = f"phase3_camera_check_{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}"
    out_dir = args.out / run_id
    out_dir.mkdir(parents=True)

    rclpy.init()
    node = Node('phase3_camera_check',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    latest = {}
    subs = [node.create_subscription(
        kind, topic, lambda msg, key=key: latest.__setitem__(key, msg), qos_profile_sensor_data)
        for key, (topic, kind) in TOPICS.items()]
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    scene = GzScene()
    names = common.object_names(catalog)
    report = {'run_id': run_id, 'git_commit': tl.git_commit(REPO),
              'config': tl.config_ref(args.config)}

    def fresh(key):
        return common.wait_message(node, latest, key, node.get_clock().now(), 10.0)

    try:
        common.clear_objects(scene, names)
        common.spin_for(node, cfg['settle_s'])

        # 1. intrinsics
        info, ideal = fresh('info'), fresh('ideal')
        k = camera.intrinsics_from_k(info.k)
        k_used = camera.intrinsics_from_k(info.k, correction)
        expected = camera.pinhole_intrinsics(image['width'], image['height'],
                                             math.radians(image['hfov_deg']))
        report['intrinsics'] = {
            'camera_info': k._asdict(), 'expected': expected._asdict(),
            'focal_error': max(abs(k.fx / expected.fx - 1), abs(k.fy / expected.fy - 1)),
            'principal_point_error_px': max(abs(k.cx - expected.cx), abs(k.cy - expected.cy)),
            'size': [info.width, info.height],
            'frame_ids': {'camera_info': info.header.frame_id, 'depth': ideal.header.frame_id},
            'distortion': list(info.d),
            'principal_point_offset_used_by_perception': list(correction),
        }

        # 2. bin floor plane, through TF
        bin_from_optical = lookup(node, buffer, 'input_bin', ideal.header.frame_id)
        world_from_optical = lookup(node, buffer, 'world', ideal.header.frame_id)
        floor, _ = bin_floor(depth_array(ideal), k, bin_from_optical, inner, cfg['bin_margin'])
        normal, centroid, rms = camera.fit_plane(floor)
        report['floor_plane'] = {
            'points': len(floor), 'offset_m': float(centroid[2]),
            'tilt_rad': math.acos(min(1.0, float(normal[2]))), 'rms_m': rms,
            'camera_height_above_floor_m': float(bin_from_optical[2, 3]),
        }

        # 4. depth model on the same floor (several frames)
        ratios, valid, sigma_measured, sigma_model = [], [], [], []
        for _ in range(5):
            measured = depth_array(fresh('measured'))
            points, z = bin_floor(measured, k, bin_from_optical, inner, cfg['bin_margin'])
            model = math.hypot(float(depth_sigma(z.mean(), k.fx, depth_cfg['baseline'],
                                                 depth_cfg['disparity_noise_px'])),
                               depth_cfg['quantization'] / math.sqrt(12))
            sigma_measured.append(float(points[:, 2].std()))
            sigma_model.append(model)
            ratios.append(sigma_measured[-1] / model)
            valid.append(len(points) / max(len(floor), 1))
        report['depth_noise'] = {
            'sigma_measured_m': float(np.mean(sigma_measured)),
            'sigma_model_m': float(np.mean(sigma_model)),
            'ratio': float(np.mean(ratios)), 'valid_fraction': float(np.mean(valid)),
        }

        # 3. known points: cube top-face centre, ground truth -> TF + K -> pixel
        spec = catalog['objects']['cube']
        height = spec['size'][2]
        optical_from_world = np.linalg.inv(world_from_optical)
        points_report = []
        for x, y, yaw in cfg['cube_poses']:
            xyz, _ = so.spawn_pose(cell, 'input', spec, x, y, yaw)
            scene.spawn('cube', so.model_sdf('cube', spec, catalog['friction']), xyz,
                        (0.0, 0.0, yaw))
            common.spin_for(node, cfg['settle_s'])
            truth_xyz, truth_quat = scene.model_poses(['cube'])['cube']
            top = np.add(truth_xyz, quat_rotate(truth_quat, (0.0, 0.0, height / 2)))
            in_optical = camera.transform_points(optical_from_world, top)[0]
            u_raw, v_raw = camera.project(in_optical, k)[0]
            u, v = camera.project(in_optical, k_used)[0]

            depth = depth_array(fresh('ideal'))
            optical, pixels = camera.depth_to_points(depth, k_used, 0.05, 5.0)
            in_bin = camera.transform_points(bin_from_optical, optical)
            # top face only: the inner faces of the bin walls reach the same height
            face = ((np.abs(in_bin[:, 2] - height) < cfg['top_tolerance'])
                    & (np.abs(in_bin[:, 0]) < inner[0] / 2 - cfg['bin_margin'])
                    & (np.abs(in_bin[:, 1]) < inner[1] / 2 - cfg['bin_margin']))
            seen_v, seen_u = pixels[face].mean(axis=0)
            points_report.append({
                'pose': [x, y, yaw], 'top_face_pixels': int(face.sum()),
                'expected_px': [float(u), float(v)], 'seen_px': [float(seen_u), float(seen_v)],
                'du_px': float(seen_u - u), 'dv_px': float(seen_v - v),
                'error_px': float(math.hypot(seen_u - u, seen_v - v)),
                'uncorrected_du_px': float(seen_u - u_raw),
                'uncorrected_dv_px': float(seen_v - v_raw),
                'depth_error_m': float(np.median(optical[face, 2]) - in_optical[2]),
            })
            scene.remove('cube')
            common.spin_for(node, 0.3)
        report['known_points'] = points_report
    finally:
        common.clear_objects(scene, names)
        del subs, listener
        node.destroy_node()
        rclpy.shutdown()

    intr, plane, noise = report['intrinsics'], report['floor_plane'], report['depth_noise']
    checks = {
        'focal': intr['focal_error'] <= limits['max_focal_error'],
        'principal_point': intr['principal_point_error_px'] <= limits['max_principal_point_px'],
        'image_size': intr['size'] == [image['width'], image['height']],
        'frame_id': set(intr['frame_ids'].values()) == {OPTICAL_FRAME},
        'plane_offset': abs(plane['offset_m']) <= limits['max_plane_offset_m'],
        'plane_tilt': plane['tilt_rad'] <= limits['max_plane_tilt_rad'],
        'reprojection': all(p['error_px'] <= limits['max_reprojection_px']
                            for p in report['known_points']),
        'depth_at_known_point': all(abs(p['depth_error_m']) <= limits['max_depth_error_m']
                                    for p in report['known_points']),
        'depth_noise': abs(noise['ratio'] - 1) <= limits['max_noise_ratio_error'],
    }
    report['checks'] = checks
    report['passed'] = all(checks.values())
    (out_dir / 'camera_check.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
