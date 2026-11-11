"""Helpers shared by the phase 3 harness scripts (they run in the VM with the cell launched)."""

import time

import rclpy
from rclpy.time import Time

import perception_eval as pe
import scene_objects as so


def object_names(catalog, per_type=4):
    """Every model name the harness may have left in the world."""
    names = list(catalog['objects'])
    return names + [f'{name}_{i}' for name in catalog['objects'] for i in range(per_type)]


def clear_objects(scene, names):
    for name in names:
        try:
            scene.remove(name)
        except RuntimeError:
            pass                               # not in the world


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def spawn_scene(scene, objects, catalog, cell):
    """Spawn sampled objects [{'name', 'type', 'x', 'y', 'yaw'}] resting in the input bin."""
    for item in objects:
        spec = catalog['objects'][item['type']]
        xyz, yaw = so.spawn_pose(cell, 'input', spec, item['x'], item['y'], item['yaw'])
        scene.spawn(item['name'], so.model_sdf(item['name'], spec, catalog['friction']), xyz,
                    (0.0, 0.0, yaw))


def read_truth(scene, objects, cell, tolerances):
    """Ground truth of the spawned objects in the input_bin frame, and whether each one rests
    upright where it was spawned: ([{'name', 'type', 'x', 'y', 'yaw'}], all_resting)."""
    origin = so.bin_origin(cell, 'input')
    poses = scene.model_poses([item['name'] for item in objects])
    truth, resting = [], True
    for item in objects:
        pose = poses.get(item['name'])
        if pose is None:
            resting = False
            continue
        in_bin = pe.pose_in_bin(pose, origin)
        moved = max(abs(in_bin['x'] - item['x']), abs(in_bin['y'] - item['y']))
        if moved > tolerances['spawn_tolerance_m'] or in_bin['tilt'] > tolerances['max_tilt_rad']:
            resting = False
        truth.append({'name': item['name'], 'type': item['type'], 'x': in_bin['x'],
                      'y': in_bin['y'], 'yaw': in_bin['yaw']})
    return truth, resting


def hypotheses_in_bin(hypotheses, cell):
    """Perception hypotheses (world frame) as [{'class_id', 'score', 'x', 'y', 'yaw'}] in the
    input_bin frame, for the evaluation."""
    origin = so.bin_origin(cell, 'input')
    out = []
    for h in hypotheses:
        in_bin = pe.pose_in_bin((h['xyz'], h['quat']), origin)
        out.append({'class_id': h['class_id'], 'score': h['score'], 'x': in_bin['x'],
                    'y': in_bin['y'], 'yaw': in_bin['yaw']})
    return out


def wait_message(node, holder, key, after, timeout_s):
    """Spin until holder[key] is a message stamped later than `after` (rclpy Time)."""
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        msg = holder.get(key)
        if msg is not None and Time.from_msg(msg.header.stamp) > after:
            return msg
        rclpy.spin_once(node, timeout_sec=0.05)
    raise RuntimeError(f'no {key} message newer than the request within {timeout_s} s')
