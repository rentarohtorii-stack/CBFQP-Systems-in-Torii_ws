#!/usr/bin/env python3
"""Only forward points whose map-frame voxels persist for a minimum duration.

Tracks use acquisition timestamps. Output stays in the original sensor frame so
OctoMap ray casting still uses the real sensor origin. This gates new hits; it
neither expires existing OctoMap cells nor invents free-space measurements.
"""

import math
from copy import copy

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


class TemporalVoxelGate:
    """Track first/last observations of each voxel without retaining old clouds."""

    def __init__(self, voxel_size, duration, max_gap):
        if not math.isfinite(voxel_size) or voxel_size <= 0:
            raise ValueError("voxel_size must be finite and positive")
        if not math.isfinite(duration) or duration < 0:
            raise ValueError("confirmation_time must be finite and nonnegative")
        if not math.isfinite(max_gap) or max_gap <= 0:
            raise ValueError("max_observation_gap must be finite and positive")
        self.voxel_size = voxel_size
        self.duration_ns = round(duration * 1e9)
        self.max_gap_ns = round(max_gap * 1e9)
        self.tracks = {}
        self.last_stamp = None

    def select(self, points, stamp_ns):
        if self.last_stamp is not None:
            if stamp_ns == self.last_stamp:
                return np.zeros(len(points), dtype=bool)
            if stamp_ns < self.last_stamp:
                self.tracks.clear()
        self.last_stamp = stamp_ns
        if self.duration_ns == 0:
            self.tracks.clear()
            return np.ones(len(points), dtype=bool)

        # Prune missing voxels, including when an empty cloud arrives.
        self.tracks = {
            key: value for key, value in self.tracks.items()
            if stamp_ns - value[1] < self.max_gap_ns
        }
        if len(points) == 0:
            return np.zeros(0, dtype=bool)
        voxel_keys = np.floor(points / self.voxel_size).astype(np.int64)
        # Sort fixed-size records rather than Python tuples or NumPy axis rows.
        records = voxel_keys.view(np.dtype((np.void, voxel_keys.dtype.itemsize * 3))).ravel()
        _, indices, inverse = np.unique(
            records, return_index=True, return_inverse=True,
        )
        keys = voxel_keys[indices]
        confirmed = np.zeros(len(keys), dtype=bool)
        for index, key in enumerate(map(tuple, keys)):
            first, _ = self.tracks.get(key, (stamp_ns, stamp_ns))
            self.tracks[key] = (first, stamp_ns)
            confirmed[index] = stamp_ns - first >= self.duration_ns
        return confirmed[inverse]


def read_xyz(message):
    """Read organized XYZ clouds, respecting padding and endianness."""
    dtype = point_cloud2.dtype_from_fields(
        message.fields, point_step=message.point_step
    ).newbyteorder('>' if message.is_bigendian else '<')
    data = np.ndarray(
        (message.height, message.width), dtype=dtype, buffer=message.data,
        strides=(message.row_step, message.point_step),
    )
    return np.column_stack([data[name].ravel() for name in ('x', 'y', 'z')])


def transform_xyz(points, transform):
    q = transform.rotation
    quaternion = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm == 0:
        raise ValueError("Invalid TF quaternion")
    x, y, z, w = quaternion / norm
    rotation = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])
    t = transform.translation
    return points @ rotation.T + [t.x, t.y, t.z]


class TemporalPointcloudFilter(Node):
    def __init__(self):
        super().__init__('temporal_pointcloud_filter')
        defaults = {
            'input_topic': '/amir_pc/depth/points',
            'output_topic': '/amir_pc/depth/confirmed_points',
            'map_frame': 'camera_depth_optical_frame',
            'voxel_size': 0.03,
            'confirmation_time': 1.0,
            'max_observation_gap': 0.2,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        values = {name: self.get_parameter(name).value for name in defaults}
        self.gate = TemporalVoxelGate(
            values['voxel_size'], values['confirmation_time'],
            values['max_observation_gap'],
        )
        self.map_frame = values['map_frame']
        self.sensor_frame = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            PointCloud2, values['output_topic'], qos_profile_sensor_data
        )
        # Avoid processing a backlog of stale camera frames.
        qos = copy(qos_profile_sensor_data)
        qos.depth = 1
        self.subscription = self.create_subscription(
            PointCloud2, values['input_topic'], self.cloud_callback, qos
        )
        self.get_logger().info(
            f"Temporal cloud filter: {values['confirmation_time']}s confirmation, "
            f"{values['max_observation_gap']}s gap reset, "
            f"{values['voxel_size']}m voxels in {self.map_frame}"
        )

    def cloud_callback(self, message):
        if not message.header.frame_id:
            self.get_logger().warning('Dropping cloud without frame_id', throttle_duration_sec=5.0)
            return
        try:
            points = read_xyz(message)
            points = points[np.isfinite(points).all(axis=1)]
            if message.header.frame_id == self.map_frame:
                map_points = points
            else:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, message.header.frame_id,
                    Time.from_msg(message.header.stamp),
                )
                map_points = transform_xyz(points, transform.transform)
            if self.sensor_frame != message.header.frame_id:
                self.gate.tracks.clear()
                self.gate.last_stamp = None
                self.sensor_frame = message.header.frame_id
            stamp = Time.from_msg(message.header.stamp).nanoseconds
            mask = self.gate.select(map_points, stamp)
            output = point_cloud2.create_cloud_xyz32(
                message.header, np.ascontiguousarray(points[mask], dtype=np.float32)
            )
            output.is_dense = True
            self.publisher.publish(output)
        except (TransformException, ValueError, KeyError) as error:
            self.get_logger().warning(
                f'Dropping cloud: {error}', throttle_duration_sec=5.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TemporalPointcloudFilter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
