#!/usr/bin/env python3
"""Publish one timestamp-matched RGB-D/point-cloud snapshot after YOLO sees a bottle."""

from collections import deque
import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class BottleOctomapSnapshot(Node):
    def __init__(self):
        super().__init__("bottle_octomap_snapshot")

        defaults = {
            "cloud_topic": "/d435/depth/color/points",
            "depth_topic": "/d435/depth/image_rect_raw",
            "detections_topic": "/yolo26/detections",
            "bottle_rgb_topic": "/yolo26/bottle_rgb_image",
            "bottle_annotated_topic": "/yolo26/bottle_annotated_image",
            "snapshot_cloud_topic": "/bottle_snapshot/points",
            "snapshot_depth_topic": "/bottle_snapshot/depth",
            "snapshot_rgb_topic": "/bottle_snapshot/rgb",
            "snapshot_annotated_topic": "/bottle_snapshot/annotated_image",
            "completion_topic": "/bottle_snapshot/complete",
            "octomap_markers_topic": "/occupied_cells_vis_array",
            "require_octomap_confirmation": True,
            "minimum_confidence": 0.35,
            "buffer_seconds": 4.0,
            "max_stamp_difference_seconds": 0.15,
            "sample_period_seconds": 0.08,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def param(name):
            return self.get_parameter(name).value

        self.buffer_ns = int(float(param("buffer_seconds")) * 1e9)
        self.max_difference_ns = int(
            float(param("max_stamp_difference_seconds")) * 1e9
        )
        self.sample_period_ns = int(float(param("sample_period_seconds")) * 1e9)
        self.minimum_confidence = float(param("minimum_confidence"))
        self.require_octomap = bool(param("require_octomap_confirmation"))
        if self.buffer_ns <= 0 or self.max_difference_ns <= 0:
            raise ValueError("buffer_seconds and max_stamp_difference_seconds must be positive")
        if self.sample_period_ns < 0:
            raise ValueError("sample_period_seconds must not be negative")

        self.clouds = deque(maxlen=64)
        self.depths = deque(maxlen=64)
        self.bottle_rgbs = deque(maxlen=24)
        self.bottle_annotated = deque(maxlen=24)
        self.pending_detections = deque(maxlen=24)
        self.capture = None
        self.published = False
        self.complete = False
        self.map_ready = False
        self.last_status_log = 0.0

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.cloud_topic = str(param("snapshot_cloud_topic"))
        self.cloud_pub = self.create_publisher(PointCloud2, self.cloud_topic, latched_qos)
        self.depth_pub = self.create_publisher(
            Image, str(param("snapshot_depth_topic")), latched_qos
        )
        self.rgb_pub = self.create_publisher(
            Image, str(param("snapshot_rgb_topic")), latched_qos
        )
        self.annotated_pub = self.create_publisher(
            Image, str(param("snapshot_annotated_topic")), latched_qos
        )
        self.complete_pub = self.create_publisher(
            Bool, str(param("completion_topic")), latched_qos
        )

        self.input_subscriptions = [
            self.create_subscription(
                PointCloud2, str(param("cloud_topic")), self.cloud_callback,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Image, str(param("depth_topic")), self.depth_callback,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Image, str(param("bottle_rgb_topic")), self.rgb_callback,
                reliable_qos,
            ),
            self.create_subscription(
                Image, str(param("bottle_annotated_topic")),
                self.annotated_callback, reliable_qos,
            ),
            self.create_subscription(
                String, str(param("detections_topic")),
                self.detection_callback, reliable_qos,
            ),
        ]
        self.create_subscription(
            MarkerArray, str(param("octomap_markers_topic")),
            self.markers_callback, latched_qos,
        )
        self.create_timer(0.1, self.poll)
        self.get_logger().info(
            "Waiting for YOLO bottle detection and timestamp-matched "
            f"RGB/depth/point cloud (tolerance={self.max_difference_ns / 1e9:.2f}s)"
        )

    @staticmethod
    def closest(buffer, target_ns, tolerance_ns):
        if not buffer:
            return None
        item = min(buffer, key=lambda pair: abs(pair[0] - target_ns))
        return item if abs(item[0] - target_ns) <= tolerance_ns else None

    def store_sensor(self, buffer, message):
        if self.capture is not None:
            return
        current_ns = stamp_ns(message.header.stamp)
        if current_ns <= 0:
            return
        if buffer and current_ns < buffer[-1][0]:
            buffer.clear()
        if buffer and current_ns - buffer[-1][0] < self.sample_period_ns:
            return
        buffer.append((current_ns, message))
        cutoff_ns = current_ns - self.buffer_ns
        while buffer and buffer[0][0] < cutoff_ns:
            buffer.popleft()
        self.try_capture()

    def store_bottle_image(self, buffer, message):
        if self.capture is not None:
            return
        current_ns = stamp_ns(message.header.stamp)
        if current_ns <= 0:
            return
        buffer.append((current_ns, message))
        self.try_capture()

    def cloud_callback(self, message):
        self.store_sensor(self.clouds, message)

    def depth_callback(self, message):
        self.store_sensor(self.depths, message)

    def rgb_callback(self, message):
        self.store_bottle_image(self.bottle_rgbs, message)

    def annotated_callback(self, message):
        self.store_bottle_image(self.bottle_annotated, message)

    def detection_callback(self, message):
        if self.capture is not None:
            return
        try:
            result = json.loads(message.data)
            bottle_found = any(
                str(item.get("class_name", "")).casefold() == "bottle"
                and float(item.get("confidence", 0.0)) >= self.minimum_confidence
                for item in result.get("detections", [])
            )
            stamp = result["stamp"]
            detected_ns = (
                int(stamp["sec"]) * 1_000_000_000 + int(stamp["nanosec"])
            )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(f"Invalid YOLO result: {error}")
            return
        if not bottle_found or detected_ns <= 0:
            return
        self.pending_detections.append((detected_ns, time.monotonic()))
        self.try_capture()

    def try_capture(self):
        if self.capture is not None:
            return
        for detected_ns, _arrival in self.pending_detections:
            rgb = self.closest(self.bottle_rgbs, detected_ns, 0)
            annotated = self.closest(self.bottle_annotated, detected_ns, 0)
            cloud = self.closest(self.clouds, detected_ns, self.max_difference_ns)
            depth = self.closest(self.depths, detected_ns, self.max_difference_ns)
            if not all((rgb, annotated, cloud, depth)):
                continue
            self.capture = (detected_ns, rgb[1], annotated[1], depth[1], cloud[1])
            self.get_logger().info(
                "Bottle snapshot captured: "
                f"cloud Δ={abs(cloud[0] - detected_ns) / 1e3:.0f}µs, "
                f"depth Δ={abs(depth[0] - detected_ns) / 1e3:.0f}µs; "
                "freezing all input subscriptions"
            )
            return

    def markers_callback(self, message):
        if self.published and any(marker.points for marker in message.markers):
            self.map_ready = True

    def octomap_subscriber_ready(self):
        return any(
            endpoint.node_name == "octomap_server"
            for endpoint in self.get_subscriptions_info_by_topic(self.cloud_topic)
        )

    def poll(self):
        now = time.monotonic()
        if self.capture is None:
            while self.pending_detections and now - self.pending_detections[0][1] > (
                self.buffer_ns / 1e9 + 1.0
            ):
                self.pending_detections.popleft()
            if now - self.last_status_log >= 10.0:
                self.last_status_log = now
                self.get_logger().info(
                    "Snapshot status: waiting for bottle/matched frames; "
                    f"clouds={len(self.clouds)}, depths={len(self.depths)}, "
                    f"bottle_results={len(self.pending_detections)}"
                )
            return

        if not self.published:
            if self.require_octomap and not self.octomap_subscriber_ready():
                if now - self.last_status_log >= 5.0:
                    self.last_status_log = now
                    self.get_logger().warning("Waiting for OctoMap cloud subscriber")
                return
            for subscription in self.input_subscriptions:
                self.destroy_subscription(subscription)
            self.input_subscriptions.clear()
            detected_ns, rgb, annotated, depth, cloud = self.capture
            self.rgb_pub.publish(rgb)
            self.annotated_pub.publish(annotated)
            self.depth_pub.publish(depth)
            self.cloud_pub.publish(cloud)
            self.clouds.clear()
            self.depths.clear()
            self.bottle_rgbs.clear()
            self.bottle_annotated.clear()
            self.pending_detections.clear()
            self.published = True
            self.get_logger().info(
                f"Snapshot published at RGB stamp {detected_ns}; "
                "no further point-cloud updates will be sent to OctoMap"
            )

        if not self.complete and (not self.require_octomap or self.map_ready):
            self.complete = True
            self.complete_pub.publish(Bool(data=True))
            if self.require_octomap:
                self.get_logger().info("OctoMap occupied cells confirmed; YOLO stop requested")
            else:
                self.get_logger().info("Snapshot complete; YOLO stop requested")
        elif not self.complete and now - self.last_status_log >= 5.0:
            self.last_status_log = now
            self.get_logger().info("Snapshot sent; waiting for OctoMap occupied cells before stopping YOLO")


def main(args=None):
    rclpy.init(args=args)
    node = BottleOctomapSnapshot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
