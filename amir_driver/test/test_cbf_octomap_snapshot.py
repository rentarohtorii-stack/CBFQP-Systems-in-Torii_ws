"""Verify one-shot capture and robot-frame map geometry without hardware."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np
from geometry_msgs.msg import Point, Transform
from visualization_msgs.msg import Marker, MarkerArray

spec = importlib.util.spec_from_file_location(
    'cbf_octomap_snapshot',
    Path(__file__).resolve().parents[1] / 'scripts/cbf_octomap_snapshot.py',
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def map_message():
    marker = Marker()
    marker.header.frame_id = 'camera_depth_optical_frame'
    marker.header.stamp.sec = 123
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.
    marker.scale.x = marker.scale.y = marker.scale.z = .1
    marker.points = [Point(x=1., y=0., z=0.)]
    return MarkerArray(markers=[marker])


class SnapshotTests(unittest.TestCase):
    def test_delay_freshness_and_only_once(self):
        state = module.SnapshotState(10)
        self.assertFalse(state.ready(100, 1))
        state.update('first', 100)
        self.assertFalse(state.ready(109, 1))
        self.assertFalse(state.ready(110, 1))  # No recent data.
        state.update('latest', 110)
        self.assertTrue(state.ready(110, 1))
        self.assertEqual(state.started, 100)
        state.done = True
        self.assertFalse(state.update('ignored', 111))
        self.assertFalse(state.ready(111, 1))
        self.assertEqual(state.message, 'latest')

    def test_marker_pose_then_capture_time_tf_and_unchanged_radius(self):
        message = map_message()
        marker = message.markers[0]
        marker.pose.position.y = 1.
        transform = Transform()
        transform.rotation.z = np.sqrt(.5)
        transform.rotation.w = np.sqrt(.5)
        transform.translation.x = 2.
        calls = []

        def lookup(target, source, stamp):
            calls.append((target, source, stamp.sec))
            return transform

        frozen = module.transform_snapshot(message, 'link0_1', lookup)
        p = frozen.markers[0].points[0]
        np.testing.assert_allclose([p.x, p.y, p.z], [1., 1., 0.], atol=1e-7)
        self.assertEqual(calls, [('link0_1', 'camera_depth_optical_frame', 123)])
        self.assertEqual(frozen.markers[0].scale, marker.scale)
        self.assertEqual(frozen.markers[0].header.frame_id, 'link0_1')
        self.assertEqual(frozen.markers[0].header.stamp.sec, 0)
        self.assertEqual(frozen.markers[0].pose.orientation.w, 1.)
        self.assertEqual(marker.points[0].x, 1.)  # Input preserved.

    def test_empty_and_invalid_maps_are_rejected(self):
        with self.assertRaises(ValueError):
            module.transform_snapshot(MarkerArray(), 'link0_1', None)
        for value in (0., float('nan')):
            message = map_message()
            message.markers[0].scale.x = value
            with self.assertRaises(ValueError):
                module.transform_snapshot(message, 'link0_1', None)

    def test_receive_accepts_new_snapshot_while_waiting_for_tf(self):
        destroyed = []
        logger = SimpleNamespace(info=lambda *_: None)
        node = SimpleNamespace(
            mode='receive', state=module.SnapshotState(0), subscription='sub',
            destroy_subscription=destroyed.append, get_logger=lambda: logger,
        )
        first, second = map_message(), map_message()
        second.markers[0].points[0].x = 9.
        module.CbfOctomapSnapshot.receive(node, first)
        module.CbfOctomapSnapshot.receive(node, second)
        self.assertIs(node.state.message, second)
        self.assertEqual(destroyed, [])

    def test_sender_does_not_reuse_old_map_after_empty_update(self):
        node = SimpleNamespace(
            mode='send', state=module.SnapshotState(10),
            get_logger=lambda: SimpleNamespace(info=lambda *_: None),
        )
        module.CbfOctomapSnapshot.receive(node, map_message())
        module.CbfOctomapSnapshot.receive(node, MarkerArray())
        self.assertIsNone(node.state.message)
        self.assertIsNotNone(node.state.started)

    def test_tick_publishes_once_and_retains_publisher(self):
        output, cancelled, destroyed = [], [], []
        node = SimpleNamespace(
            mode='send', state=module.SnapshotState(0), max_age=1.,
            publisher=SimpleNamespace(publish=output.append),
            timer=SimpleNamespace(cancel=lambda: cancelled.append(True)),
            subscription='sub', destroy_subscription=destroyed.append,
            get_logger=lambda: SimpleNamespace(info=lambda *_: None),
        )
        module.CbfOctomapSnapshot.receive(node, map_message())
        module.CbfOctomapSnapshot.tick(node)
        module.CbfOctomapSnapshot.tick(node)
        self.assertEqual(len(output), 1)
        self.assertEqual(destroyed, ['sub'])
        self.assertEqual(cancelled, [True])
        self.assertTrue(node.state.done)

    def test_republish_preserves_frozen_geometry(self):
        output = []
        message = map_message()
        node = SimpleNamespace(retained_output=None,
                               publisher=SimpleNamespace(publish=output.append))
        module.CbfOctomapSnapshot.republish(node)
        self.assertEqual(output, [])
        node.retained_output = message
        module.CbfOctomapSnapshot.republish(node)
        module.CbfOctomapSnapshot.republish(node)
        self.assertEqual(len(output), 2)
        self.assertTrue(all(item is message for item in output))

    def test_receiver_does_not_publish_without_tf(self):
        output = []

        def missing_tf(*args):
            raise module.TransformException('TF not available')

        node = SimpleNamespace(
            mode='receive', state=module.SnapshotState(0), target_frame='link0_1',
            tf_buffer=SimpleNamespace(lookup_transform=missing_tf),
            publisher=SimpleNamespace(publish=output.append),
            get_logger=lambda: SimpleNamespace(warning=lambda *args, **kwargs: None),
        )
        node.state.update(map_message(), 0)
        module.CbfOctomapSnapshot.tick(node)
        self.assertEqual(output, [])
        self.assertFalse(node.state.done)
        self.assertIsNotNone(node.state.message)


if __name__ == '__main__':
    unittest.main()
