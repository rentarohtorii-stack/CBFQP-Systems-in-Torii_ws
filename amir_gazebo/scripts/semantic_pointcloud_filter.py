#!/usr/bin/env python3
"""Remove YOLO-detected object depths from timestamp-matched point clouds."""

from collections import deque
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


class SemanticPointCloudFilter(Node):
    def __init__(self):
        super().__init__("semantic_pointcloud_filter")

        self.declare_parameter("cloud_topic", "/amir_camera/points")
        self.declare_parameter("detections_topic", "/yolo26/detections")
        self.declare_parameter("output_topic", "/amir_camera/points_filtered")
        self.declare_parameter("excluded_classes", ["bottle"])
        self.declare_parameter("minimum_confidence", 0.35)
        self.declare_parameter("cloud_buffer_duration", 3.0)
        self.declare_parameter("maximum_cloud_stamp_difference", 0.15)
        self.declare_parameter("maximum_buffered_clouds", 60)
        self.declare_parameter("bbox_padding", 3)
        self.declare_parameter("depth_band", 0.12)
        self.declare_parameter("input_reliability", "best_effort")

        cloud_topic = self.get_parameter("cloud_topic").value
        detections_topic = self.get_parameter("detections_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.cloud_topic = str(cloud_topic)
        self.excluded_classes = {
            str(name).lower()
            for name in self.get_parameter("excluded_classes").value
        }
        self.minimum_confidence = float(
            self.get_parameter("minimum_confidence").value
        )
        self.cloud_buffer_duration_ns = int(
            float(self.get_parameter("cloud_buffer_duration").value) * 1e9
        )
        self.maximum_cloud_stamp_difference_ns = int(
            float(self.get_parameter("maximum_cloud_stamp_difference").value)
            * 1e9
        )
        self.maximum_buffered_clouds = max(
            1, int(self.get_parameter("maximum_buffered_clouds").value)
        )
        self.bbox_padding = int(self.get_parameter("bbox_padding").value)
        self.depth_band = float(self.get_parameter("depth_band").value)
        input_reliability = str(
            self.get_parameter("input_reliability").value
        ).strip().lower()
        if input_reliability not in {"best_effort", "reliable"}:
            raise ValueError(
                "input_reliability must be 'best_effort' or 'reliable'"
            )
        self.input_reliability = input_reliability

        self.cloud_buffer = deque()
        self.last_cloud_stamp_ns = None
        self.received_clouds = 0
        self.published_clouds = 0
        self.unmatched_detections = 0
        self.last_unmatched_log_time = 0.0
        self.warned_unorganized = False

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=(
                ReliabilityPolicy.RELIABLE
                if self.input_reliability == "reliable"
                else ReliabilityPolicy.BEST_EFFORT
            ),
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.cloud_subscription = self.create_subscription(
            PointCloud2, cloud_topic, self.cloud_callback, sensor_qos
        )
        self.detection_subscription = self.create_subscription(
            String, detections_topic, self.detection_callback, 10
        )
        self.filtered_cloud_publisher = self.create_publisher(
            PointCloud2, output_topic, output_qos
        )
        self.status_timer = self.create_timer(5.0, self.log_status)

        self.get_logger().info(
            f"Semantic point-cloud filter: {cloud_topic} -> {output_topic}; "
            f"excluded={sorted(self.excluded_classes)}, "
            f"buffer={self.cloud_buffer_duration_ns / 1e9:.1f}s, "
            f"input_reliability={self.input_reliability}"
        )

    @staticmethod
    def stamp_to_nanoseconds(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def cloud_callback(self, cloud):
        """Keep recent clouds until the matching YOLO result arrives."""

        cloud_stamp_ns = self.stamp_to_nanoseconds(cloud.header.stamp)
        if (
            self.last_cloud_stamp_ns is not None
            and cloud_stamp_ns < self.last_cloud_stamp_ns
        ):
            self.cloud_buffer.clear()
        self.last_cloud_stamp_ns = cloud_stamp_ns
        self.received_clouds += 1
        self.cloud_buffer.append((cloud_stamp_ns, cloud))
        self.trim_cloud_buffer(cloud_stamp_ns)

    def detection_callback(self, message):
        """Process the exact point cloud captured with YOLO's RGB image."""

        try:
            result = json.loads(message.data)
            stamp = result["stamp"]
            detection_stamp_ns = (
                int(stamp["sec"]) * 1_000_000_000 + int(stamp["nanosec"])
            )
            image_width = int(result["image_width"])
            image_height = int(result["image_height"])
            excluded_boxes = [
                detection["bbox"]
                for detection in result.get("detections", [])
                if str(detection.get("class_name", "")).lower()
                in self.excluded_classes
                and float(detection.get("confidence", 0.0))
                >= self.minimum_confidence
                and "bbox" in detection
            ]
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(f"Invalid YOLO detection message: {error}")
            return

        match = self.find_matching_cloud(detection_stamp_ns)
        if match is None:
            self.unmatched_detections += 1
            self.log_unmatched_detection(detection_stamp_ns)
            return

        cloud_stamp_ns, cloud = match
        self.publish_filtered_cloud(
            cloud,
            excluded_boxes,
            image_width,
            image_height,
        )
        self.discard_consumed_clouds(cloud_stamp_ns)

    def trim_cloud_buffer(self, newest_stamp_ns):
        minimum_stamp_ns = newest_stamp_ns - self.cloud_buffer_duration_ns
        while self.cloud_buffer and (
            self.cloud_buffer[0][0] < minimum_stamp_ns
            or len(self.cloud_buffer) > self.maximum_buffered_clouds
        ):
            self.cloud_buffer.popleft()

    def find_matching_cloud(self, detection_stamp_ns):
        if not self.cloud_buffer:
            return None
        cloud_stamp_ns, cloud = min(
            self.cloud_buffer,
            key=lambda item: abs(item[0] - detection_stamp_ns),
        )
        if (
            abs(cloud_stamp_ns - detection_stamp_ns)
            > self.maximum_cloud_stamp_difference_ns
        ):
            return None
        return cloud_stamp_ns, cloud

    def discard_consumed_clouds(self, matched_stamp_ns):
        while self.cloud_buffer and self.cloud_buffer[0][0] <= matched_stamp_ns:
            self.cloud_buffer.popleft()

    def publish_filtered_cloud(
        self,
        cloud,
        excluded_boxes,
        image_width,
        image_height,
    ):
        if cloud.height <= 1 or cloud.width <= 1:
            if not self.warned_unorganized:
                self.get_logger().error(
                    "Input PointCloud2 is not organized; image bounding boxes "
                    "cannot be mapped to pixels. Cloud is being dropped."
                )
                self.warned_unorganized = True
            return

        field_offsets = {field.name: field.offset for field in cloud.fields}
        if not {"x", "y", "z"}.issubset(field_offsets):
            self.get_logger().error("PointCloud2 has no x/y/z fields")
            return
        if image_width <= 0 or image_height <= 0:
            return

        filtered = PointCloud2()
        filtered.header = cloud.header
        filtered.height = cloud.height
        filtered.width = cloud.width
        filtered.fields = cloud.fields
        filtered.is_bigendian = cloud.is_bigendian
        filtered.point_step = cloud.point_step
        filtered.row_step = cloud.row_step
        filtered.is_dense = False

        mutable_data = bytearray(cloud.data)
        float_type = np.dtype(">f4" if cloud.is_bigendian else "<f4")
        shape = (int(cloud.height), int(cloud.width))
        strides = (int(cloud.row_step), int(cloud.point_step))
        coordinates = {
            axis: np.ndarray(
                shape=shape,
                dtype=float_type,
                buffer=mutable_data,
                offset=int(field_offsets[axis]),
                strides=strides,
            )
            for axis in ("x", "y", "z")
        }

        scale_x = cloud.width / image_width
        scale_y = cloud.height / image_height
        removed_pixels = 0
        for box in excluded_boxes:
            removed_pixels += self.remove_box_depth_band(
                coordinates,
                box,
                scale_x,
                scale_y,
                int(cloud.width),
                int(cloud.height),
            )

        filtered.data = bytes(mutable_data)
        self.filtered_cloud_publisher.publish(filtered)
        self.published_clouds += 1
        if removed_pixels:
            self.get_logger().debug(
                f"Removed {removed_pixels} point pixels in "
                f"{len(excluded_boxes)} bottle region(s)"
            )

    def remove_box_depth_band(
        self,
        coordinates,
        box,
        scale_x,
        scale_y,
        cloud_width,
        cloud_height,
    ):
        x_min = math.floor(float(box["x_min"]) * scale_x) - self.bbox_padding
        y_min = math.floor(float(box["y_min"]) * scale_y) - self.bbox_padding
        x_max = math.ceil(float(box["x_max"]) * scale_x) + self.bbox_padding
        y_max = math.ceil(float(box["y_max"]) * scale_y) + self.bbox_padding
        x_min = max(0, min(cloud_width, x_min))
        x_max = max(0, min(cloud_width, x_max))
        y_min = max(0, min(cloud_height, y_min))
        y_max = max(0, min(cloud_height, y_max))
        if x_min >= x_max or y_min >= y_max:
            return 0

        depth_region = coordinates["z"][y_min:y_max, x_min:x_max]
        valid_depth = np.isfinite(depth_region) & (depth_region > 0.0)
        if not np.any(valid_depth):
            return 0
        median_depth = float(np.median(depth_region[valid_depth]))
        bottle_mask = valid_depth & (
            np.abs(depth_region - median_depth) <= self.depth_band
        )
        for axis in coordinates.values():
            region = axis[y_min:y_max, x_min:x_max]
            region[bottle_mask] = np.nan
        return int(np.count_nonzero(bottle_mask))

    def log_unmatched_detection(self, detection_stamp_ns):
        now = time.monotonic()
        if now - self.last_unmatched_log_time < 5.0:
            return
        self.last_unmatched_log_time = now
        if self.received_clouds == 0:
            self.get_logger().warning(
                "YOLO detections are arriving, but no input PointCloud2 has "
                "been received; check the Gazebo/RealSense point-cloud bridge"
            )
            return
        self.get_logger().warning(
            "No buffered point cloud matches YOLO stamp "
            f"{detection_stamp_ns}; check the RGB/point-cloud timestamps and "
            "cloud_buffer_duration"
        )

    def log_status(self):
        self.get_logger().info(
            "Semantic filter status: "
            f"received={self.received_clouds}, published={self.published_clouds}, "
            f"buffered={len(self.cloud_buffer)}, "
            f"unmatched_yolo={self.unmatched_detections}, "
            f"raw_publishers={self.count_publishers(self.cloud_topic)}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SemanticPointCloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
