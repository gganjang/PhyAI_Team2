"""Translation between the measured arm frame and the tabletop map.

The tabletop origin is the junction of the four plates. Axes retain the
installation's arm-frame directions; all distances are metres.
"""
import numpy as np


def map_origin(config):
    coordinates = config.get('coordinate_system', {})
    origin = np.asarray(coordinates.get('origin_in_source', [0., 0., 0.]), dtype=float)
    if origin.shape != (3,) or not np.isfinite(origin).all():
        raise ValueError('coordinate_system.origin_in_source requires finite XYZ metres.')
    if origin[2] != 0:
        raise ValueError('The installation support plane must remain at z=0.')
    return origin


def source_frame(config):
    map_origin(config)  # Validate even when only the TF target is requested.
    return config.get('coordinate_system', {}).get('source_frame', config['map_frame'])


def camera_to_map(config, camera_to_source):
    result = np.array(camera_to_source, dtype=float, copy=True)
    result[:3, 3] -= map_origin(config)
    return result


def camera_to_source(config, camera_to_map_matrix):
    result = np.array(camera_to_map_matrix, dtype=float, copy=True)
    result[:3, 3] += map_origin(config)
    return result


def target_to_source(config, target):
    return {**target, 'position': (np.asarray(target['position']) + map_origin(config)).tolist()}
