#!/usr/bin/env python3
"""Forward raw and reconstructed PointCloud2 streams to one OctoMap input."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class PointCloudMux(Node):
    """Make octomap_server integrate every cloud published by either source."""

    def __init__(self):
        super().__init__("pointcloud_mux")
        self.declare_parameter("raw_topic", "/amir_camera/points")
        self.declare_parameter("reconstructed_topic", "/gcasp/bottle_points")
        self.declare_parameter("output_topic", "/amir_camera/points_fused")

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_topic = str(self.get_parameter("output_topic").value)
        self.publisher = self.create_publisher(PointCloud2, output_topic, output_qos)
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("raw_topic").value),
            self.forward_raw,
            sensor_qos,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("reconstructed_topic").value),
            self.forward_reconstructed,
            sensor_qos,
        )
        self.raw_count = 0
        self.reconstructed_count = 0
        self.status_timer = self.create_timer(5.0, self.log_status)
        self.get_logger().info(
            f"PointCloud fusion input is published reliably on {output_topic}"
        )

    def forward_raw(self, message):
        self.raw_count += 1
        self.publisher.publish(message)

    def forward_reconstructed(self, message):
        self.reconstructed_count += 1
        self.publisher.publish(message)

    def log_status(self):
        self.get_logger().info(
            "PointCloud mux status: "
            f"raw={self.raw_count}, gcasp={self.reconstructed_count}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMux()
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
