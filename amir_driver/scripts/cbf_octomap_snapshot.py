#!/usr/bin/env python3
"""Send one delayed CBF map or receive and freeze one map in robot coordinates."""

from copy import deepcopy
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def rotation_matrix(q):
    values = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(values)
    if not np.isfinite(norm) or norm == 0:
        raise ValueError('Invalid quaternion')
    x, y, z, w = values / norm
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def sphere_markers(message):
    return [marker for marker in message.markers
            if marker.action == Marker.ADD
            and marker.type == Marker.SPHERE_LIST and marker.points]


def transform_snapshot(message, target_frame, lookup):
    """Freeze marker poses and capture-time TF into target-frame sphere centers."""
    output = MarkerArray()
    transforms = {}
    for marker in sphere_markers(message):
        source = marker.header.frame_id
        if not source:
            raise ValueError('Sphere marker has no frame_id')
        scales = np.array([marker.scale.x, marker.scale.y, marker.scale.z])
        if not np.isfinite(scales).all() or np.any(scales <= 0):
            raise ValueError('Invalid sphere diameter')
        centers = np.array([[p.x, p.y, p.z] for p in marker.points], dtype=float)
        position = marker.pose.position
        centers = centers @ rotation_matrix(marker.pose.orientation).T
        centers += [position.x, position.y, position.z]
        if source != target_frame:
            key = (source, marker.header.stamp.sec, marker.header.stamp.nanosec)
            if key not in transforms:
                transforms[key] = lookup(target_frame, source, marker.header.stamp)
            transform = transforms[key]
            centers = centers @ rotation_matrix(transform.rotation).T
            t = transform.translation
            centers += [t.x, t.y, t.z]
        if not np.isfinite(centers).all():
            raise ValueError('Nonfinite sphere coordinates')
        frozen = deepcopy(marker)
        frozen.header.frame_id = target_frame
        # A frozen map has no live TF dependency or expiration.
        frozen.header.stamp.sec = 0
        frozen.header.stamp.nanosec = 0
        frozen.pose.position.x = 0.0
        frozen.pose.position.y = 0.0
        frozen.pose.position.z = 0.0
        frozen.pose.orientation.x = 0.0
        frozen.pose.orientation.y = 0.0
        frozen.pose.orientation.z = 0.0
        frozen.pose.orientation.w = 1.0
        frozen.lifetime.sec = 0
        frozen.lifetime.nanosec = 0
        frozen.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in centers]
        output.markers.append(frozen)
    if not output.markers:
        raise ValueError('Snapshot contains no obstacle spheres')
    return output


class SnapshotState:
    """One-shot state, independent of ROS clocks and transport."""

    def __init__(self, delay):
        if not math.isfinite(delay) or delay < 0:
            raise ValueError('capture_delay must be finite and nonnegative')
        self.delay = delay
        self.started = None
        self.updated = None
        self.message = None
        self.done = False

    def update(self, message, now):
        if self.done:
            return False
        if self.started is None:
            self.started = now
        self.message = message
        self.updated = now
        return True

    def ready(self, now, max_age):
        return (not self.done and self.message is not None
                and now - self.started >= self.delay
                and now - self.updated <= max_age)


class CbfOctomapSnapshot(Node):
    def __init__(self):
        super().__init__('cbf_octomap_snapshot')
        for name, default in {
            'mode': 'send', 'input_topic': '/cbf/obstacle_spheres',
            'output_topic': '/cbf/octomap_snapshot', 'capture_delay': 10.0,
            'max_source_age': 1.0, 'target_frame': 'link0_1',
        }.items():
            self.declare_parameter(name, default)
        self.mode = self.get_parameter('mode').value
        if self.mode not in ('send', 'receive'):
            raise ValueError('mode must be send or receive')
        self.state = SnapshotState(float(self.get_parameter('capture_delay').value))
        self.max_age = float(self.get_parameter('max_source_age').value)
        if not math.isfinite(self.max_age) or self.max_age <= 0:
            raise ValueError('max_source_age must be positive')
        self.target_frame = self.get_parameter('target_frame').value
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(
            MarkerArray, self.get_parameter('output_topic').value, qos
        )
        self.retained_output = None
        self.republish_timer = self.create_timer(1.0, self.republish)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(
            MarkerArray, self.get_parameter('input_topic').value, self.receive, qos
        )
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f'{self.mode}: {self.get_parameter("input_topic").value} -> '
            f'{self.get_parameter("output_topic").value}'
        )

    def receive(self, message):
        if self.state.done:
            return
        if not sphere_markers(message):
            # Do not freeze the previous occupied map if the latest map is empty.
            if self.mode == 'send':
                self.state.message = None
            return
        first = self.state.started is None
        self.state.update(message, time.monotonic())
        if self.mode == 'receive':
            if first:
                self.get_logger().info('First snapshot received; waiting for capture-time TF')
        elif first:
            self.get_logger().info(f'First occupied map received; capturing in {self.state.delay}s')

    def republish(self):
        """Keep the same frozen geometry available to restarted subscribers."""
        if self.retained_output is not None:
            self.publisher.publish(self.retained_output)

    def tick(self):
        if self.state.done or self.state.message is None:
            return
        if self.mode == 'send':
            if not self.state.ready(time.monotonic(), self.max_age):
                return
            output = self.state.message
        else:
            try:
                output = transform_snapshot(
                    self.state.message, self.target_frame,
                    lambda target, source, stamp: self.tf_buffer.lookup_transform(
                        target, source, Time.from_msg(stamp)
                    ).transform,
                )
            except (TransformException, ValueError) as error:
                self.get_logger().warning(
                    f'Snapshot not ready: {error}', throttle_duration_sec=5.0
                )
                return
        self.retained_output = output
        self.publisher.publish(output)
        self.state.done = True
        self.state.message = None
        self.timer.cancel()
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None
        self.get_logger().info('Snapshot published once; retaining it for late subscribers')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CbfOctomapSnapshot()
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
