"""Regression checks for temporal gating, timestamps and cloud geometry."""
import importlib.util
from pathlib import Path
import unittest
from types import SimpleNamespace

import numpy as np
from geometry_msgs.msg import Transform
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2

spec = importlib.util.spec_from_file_location(
    'temporal_pointcloud_filter',
    Path(__file__).resolve().parents[1] / 'scripts/temporal_pointcloud_filter.py',
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class TemporalFilterTests(unittest.TestCase):
    def test_requires_elapsed_observation_and_resets_after_gap(self):
        gate = module.TemporalVoxelGate(0.03, 1.0, 0.2)
        points = np.array([[0.01, 0.01, 0.31]])
        for i in range(10):
            self.assertFalse(gate.select(points, i * 100_000_000)[0])
        self.assertTrue(gate.select(points, 1_000_000_000)[0])
        self.assertFalse(gate.select(points, 1_200_000_000)[0])

    def test_transient_points_do_not_pass_and_missing_tracks_expire(self):
        gate = module.TemporalVoxelGate(0.03, 0.3, 0.2)
        for i in range(5):
            # The first voxel is stable, the second appears only once.
            points = np.array([[0.01, 0, 1], [i + 1., 0, 1]])
            mask = gate.select(points, i * 100_000_000)
            self.assertEqual(mask.tolist(), [i >= 3, False])
        gate.select(np.empty((0, 3)), 1_000_000_000)
        self.assertEqual(gate.tracks, {})

    def test_multiple_points_in_one_frame_cannot_confirm(self):
        gate = module.TemporalVoxelGate(0.03, 0.1, 0.2)
        points = np.repeat([[0.01, 0, 1]], 1000, axis=0)
        self.assertFalse(gate.select(points, 0).any())
        self.assertFalse(gate.select(points, 0).any())
        self.assertTrue(gate.select(points, 100_000_000).all())
        self.assertFalse(gate.select(points, 0).any())  # Time rewind resets.

    def test_zero_duration_and_invalid_parameters(self):
        gate = module.TemporalVoxelGate(0.03, 0, 0.2)
        self.assertTrue(gate.select(np.array([[0, 0, 1]]), 0)[0])
        for values in [(0, 1, .2), (.03, -1, .2), (.03, 1, 0),
                       (float('nan'), 1, .2), (.03, float('inf'), .2)]:
            with self.assertRaises(ValueError):
                module.TemporalVoxelGate(*values)

    def test_map_coordinates_and_original_sensor_origin(self):
        transform = Transform()
        transform.rotation.z = np.sqrt(0.5)
        transform.rotation.w = np.sqrt(0.5)
        transform.translation.x = 1.0
        points = np.array([[1., 0., 0.]])
        np.testing.assert_allclose(
            module.transform_xyz(points, transform), [[1., 1., 0.]], atol=1e-7
        )
        np.testing.assert_array_equal(points, [[1., 0., 0.]])

    def test_padded_big_endian_cloud(self):
        cloud = PointCloud2(height=2, width=1, point_step=12, row_step=16,
                            is_bigendian=True)
        cloud.fields = [PointField(name=name, offset=i*4,
                                  datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z'))]
        payload = bytearray(32)
        points = np.ndarray((2, 3), dtype='>f4', buffer=payload, strides=(16, 4))
        points[:] = [[1, 2, 3], [4, 5, 6]]
        cloud.data = bytes(payload)
        np.testing.assert_array_equal(module.read_xyz(cloud), [[1, 2, 3], [4, 5, 6]])

    def test_callback_publishes_only_confirmed_finite_sensor_points(self):
        published = []
        node = SimpleNamespace(
            gate=module.TemporalVoxelGate(.03, .3, .2),
            map_frame='camera', sensor_frame=None,
            publisher=SimpleNamespace(publish=published.append),
        )
        for i in range(4):
            header = Header(frame_id='camera')
            header.stamp.nanosec = i * 100_000_000
            cloud = point_cloud2.create_cloud_xyz32(header, np.array([
                [.01, .01, 1.], [i + 1., 0., 1.], [np.nan, 0., 1.],
            ], dtype=np.float32))
            module.TemporalPointcloudFilter.cloud_callback(node, cloud)
            self.assertEqual(published[-1].width, 1 if i == 3 else 0)
            self.assertEqual(published[-1].header, cloud.header)
        np.testing.assert_allclose(module.read_xyz(published[-1]), [[.01, .01, 1.]])


if __name__ == '__main__':
    unittest.main()
