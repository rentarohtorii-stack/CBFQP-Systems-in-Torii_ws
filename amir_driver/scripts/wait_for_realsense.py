#!/usr/bin/env python3
"""Exit successfully once live RGB images and point clouds are available."""

import os
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2


class WaitForRealSense(Node):
    def __init__(self):
        super().__init__("wait_for_realsense")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("pointcloud_topic", "/camera/camera/depth/color/points")
        self.declare_parameter("max_sample_age", 5.0)
        self.declare_parameter("discovery_retry_seconds", 15.0)

        image_topic = self.get_parameter("image_topic").value
        pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self.max_sample_age = float(self.get_parameter("max_sample_age").value)
        if self.max_sample_age <= 0.0:
            raise ValueError("max_sample_age must be greater than zero")
        self.discovery_retry_seconds = float(
            self.get_parameter("discovery_retry_seconds").value
        )
        if self.discovery_retry_seconds <= 0.0:
            raise ValueError("discovery_retry_seconds must be greater than zero")

        self.last_image_time = None
        self.last_pointcloud_time = None
        self.ready = False
        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2,
            pointcloud_topic,
            self.pointcloud_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(5.0, self.log_status)
        self.get_logger().info(
            "Waiting for live RealSense data: "
            f"image={image_topic}, pointcloud={pointcloud_topic}"
        )

    def image_callback(self, message):
        if message.width == 0 or message.height == 0 or not message.data:
            return
        if self.last_image_time is None:
            self.get_logger().info("Received RealSense RGB image")
        self.last_image_time = time.monotonic()
        self.check_ready()

    def pointcloud_callback(self, message):
        if message.width == 0 or message.height == 0 or not message.data:
            return
        if self.last_pointcloud_time is None:
            self.get_logger().info("Received RealSense point cloud")
        self.last_pointcloud_time = time.monotonic()
        self.check_ready()

    def check_ready(self):
        now = time.monotonic()
        self.ready = (
            self.last_image_time is not None
            and self.last_pointcloud_time is not None
            and now - self.last_image_time <= self.max_sample_age
            and now - self.last_pointcloud_time <= self.max_sample_age
        )
        if self.ready:
            self.get_logger().info("RealSense RGB and point cloud are ready")

    def log_status(self):
        if not self.ready:
            now = time.monotonic()

            def status(last_received):
                if last_received is None:
                    return "no"
                if now - last_received > self.max_sample_age:
                    return "stale"
                return "yes"

            self.get_logger().info(
                "Waiting for RealSense: "
                f"RGB={status(self.last_image_time)}, "
                "pointcloud="
                f"{status(self.last_pointcloud_time)}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = WaitForRealSense()
    retry_deadline = time.monotonic() + node.discovery_retry_seconds
    try:
        while rclpy.ok() and not node.ready:
            rclpy.spin_once(node, timeout_sec=0.5)
            if time.monotonic() >= retry_deadline and not node.ready:
                node.get_logger().warning(
                    "No matching RGB image and point cloud received after "
                    f"{node.discovery_retry_seconds:g}s; restarting the DDS reader"
                )
                # A discovered Cyclone DDS reader can remain stuck without
                # delivering samples. A fresh process creates a fresh DDS
                # participant while keeping this launch action alive.
                os.execv(sys.executable, [sys.executable, *sys.argv])
        return 0 if node.ready else 1
    except (KeyboardInterrupt, ExternalShutdownException):
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
