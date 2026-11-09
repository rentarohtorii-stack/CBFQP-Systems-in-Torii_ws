#!/usr/bin/env python3
"""Save AMIR RealSense RGB images for manually labelled YOLO training data."""

from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class YoloDatasetCapture(Node):
    """Capture spaced RGB frames without adding labels automatically."""

    def __init__(self):
        super().__init__("yolo_dataset_capture")

        self.declare_parameter("image_topic", "/amir_camera/image")
        self.declare_parameter(
            "output_dir", "~/ros2_ws/datasets/amir_bottle/images/raw"
        )
        self.declare_parameter("capture_fps", 1.0)
        self.declare_parameter("max_images", 300)
        self.declare_parameter("jpeg_quality", 95)

        image_topic = str(self.get_parameter("image_topic").value)
        self.output_dir = Path(
            str(self.get_parameter("output_dir").value)
        ).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        capture_fps = float(self.get_parameter("capture_fps").value)
        self.capture_period_ns = int(1e9 / capture_fps) if capture_fps > 0 else 0
        self.max_images = max(0, int(self.get_parameter("max_images").value))
        self.jpeg_quality = max(
            1, min(100, int(self.get_parameter("jpeg_quality").value))
        )

        self.bridge = CvBridge()
        self.captured_images = 0
        self.last_stamp_ns = None
        self.capture_enabled = True

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, image_qos
        )
        self.get_logger().info(
            f"Capturing {image_topic} to {self.output_dir} "
            f"at {capture_fps:g} fps (max_images={self.max_images or 'unlimited'})."
        )
        self.get_logger().info(
            "Move the robot/camera and vary the bottle pose, distance, lighting, "
            "and background while collecting images."
        )

    @staticmethod
    def stamp_to_nanoseconds(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def image_callback(self, message):
        if not self.capture_enabled:
            return

        stamp_ns = self.stamp_to_nanoseconds(message.header.stamp)
        if (
            self.last_stamp_ns is not None
            and stamp_ns >= self.last_stamp_ns
            and stamp_ns - self.last_stamp_ns < self.capture_period_ns
        ):
            return
        self.last_stamp_ns = stamp_ns

        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            filename = (
                f"bottle_{message.header.stamp.sec:010d}_"
                f"{message.header.stamp.nanosec:09d}.jpg"
            )
            output_path = self.output_dir / filename
            saved = cv2.imwrite(
                str(output_path),
                image,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
            )
            if not saved:
                self.get_logger().error(f"Failed to save {output_path}")
                return
        except Exception as error:
            self.get_logger().error(f"Image capture failed: {error}")
            return

        self.captured_images += 1
        if self.captured_images % 25 == 0:
            self.get_logger().info(
                f"Captured {self.captured_images} images in {self.output_dir}"
            )
        if self.max_images and self.captured_images >= self.max_images:
            self.capture_enabled = False
            self.get_logger().info(
                f"Reached max_images={self.max_images}; capture is stopped."
            )


def main(args=None):
    rclpy.init(args=args)
    node = YoloDatasetCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
