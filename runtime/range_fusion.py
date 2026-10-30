"""Acquisition-pose range filtering for recorded world-cell observations."""
import shlex
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


def base_in_camera(launch_path, camera_frame='cam_left', base_frame='base_link'):
    edges = {}
    for node in ET.parse(launch_path).getroot().findall('node'):
        if node.get('pkg') == 'tf' and node.get('type') == 'static_transform_publisher':
            args = shlex.split(node.get('args'))
            xyz = np.asarray(args[:3], float)
            yaw, pitch, roll = map(float, args[3:6])
            edges[args[7]] = (args[6], Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix(), xyz)
    camera_parent, rotation, camera_xyz = edges[camera_frame]
    base_parent, _, base_xyz = edges[base_frame]
    if camera_parent != base_parent:
        raise ValueError('Range replay requires camera and base calibration in the same parent frame')
    return rotation.T @ (base_xyz-camera_xyz)


def observation_range(data, meta, base_camera):
    """Use the same image-time TF as projection, never the final vehicle pose.

    Distance is horizontal, from the exported world-cell center to base_link.
    Return a distance for every support-accepted observation in this event.
    """
    rotation = np.asarray(data['map_to_camera_rotation'], np.float64)
    translation = np.asarray(data['map_to_camera_translation'], np.float64).reshape(3)
    if rotation.shape != (3, 3) or not np.isfinite(rotation).all() or not np.isfinite(translation).all():
        raise ValueError('Missing or invalid acquisition-time camera transform')
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError('Camera rotation must be orthonormal')
    base_world = rotation.T @ (np.asarray(base_camera)-translation)
    x = meta['origin'][0]+(data['x']+.5)*meta['resolution']
    y = meta['origin'][1]+(data['y']+.5)*meta['resolution']
    distance = np.hypot(x-base_world[0], y-base_world[1])
    return distance, base_world


def within_range(distance, radius):
    if not np.isfinite(radius) or radius <= 0:
        raise ValueError('Range radius must be finite and positive')
    distance = np.asarray(distance)
    return np.isfinite(distance) & (distance <= radius)
