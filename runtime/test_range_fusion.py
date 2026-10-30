import importlib.util
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from range_fusion import observation_range, within_range

spec = importlib.util.spec_from_file_location('risk', Path(__file__).resolve().parents[2]/
    'emcp/src/elevation_mapping_cupy/elevation_mapping_cupy/script/elevation_mapping_cupy/risk_mapping.py')
risk = importlib.util.module_from_spec(spec)
spec.loader.exec_module(risk)


class RangeFusionTest(unittest.TestCase):
    def test_rotated_camera_offset_and_inclusive_boundary(self):
        rotation = Rotation.from_euler('xyz', [.2, -.1, 1.3]).as_matrix()
        base_world, base_camera = np.array([3., 4., 2.]), np.array([.4, -.1, -.8])
        data = dict(x=np.array([9, 3]), y=np.array([4, 12]), map_to_camera_rotation=rotation,
                    map_to_camera_translation=base_camera-rotation@base_world)
        distance, restored = observation_range(data, dict(origin=[-.5, -.5], resolution=1.), base_camera)
        np.testing.assert_allclose(restored, base_world, atol=1e-12)
        np.testing.assert_allclose(distance, [6., 8.], atol=1e-12)
        np.testing.assert_array_equal(within_range([6., 6.001, 8., np.nan], 6), [True, False, False, False])

    def test_far_history_cannot_contaminate_near_state(self):
        # One cell is first seen at 7 m with q=.2, then at 5 m with q=.8.
        # A final crop would retain both; per-observation 6 m fusion must not.
        results = {}
        for radius in [6., 8.]:
            mean, var, count = (np.zeros(1) for _ in range(3))
            for distance, q in [(7., .2), (5., .8)]:
                if within_range([distance], radius)[0]:
                    mean, var, count = risk.ema_update(mean, var, count, np.array([q]), .2)
            results[radius] = (mean[0], var[0], count[0])
        np.testing.assert_allclose(results[6.], [.8, 0., 1.])
        np.testing.assert_allclose(results[8.], [.32, .04608, 2.])


if __name__ == '__main__':
    unittest.main()
