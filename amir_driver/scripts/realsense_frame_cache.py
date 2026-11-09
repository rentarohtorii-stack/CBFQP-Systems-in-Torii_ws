#!/usr/bin/env python3
"""Keep the latest valid RealSense image and point cloud for late subscribers."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2


class RealSenseFrameCache(Node):
    def __init__(self):
        super().__init__("realsense_frame_cache")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("pointcloud_topic", "/camera/camera/depth/color/points")
        self.declare_parameter("cached_image_topic", "/amir/realsense/color/image_cached")
        self.declare_parameter("cached_pointcloud_topic", "/amir/realsense/depth/points_cached")

        image_topic = self.get_parameter("image_topic").value
        pointcloud_topic = self.get_parameter("pointcloud_topic").value
        cached_image_topic = self.get_parameter("cached_image_topic").value
        cached_pointcloud_topic = self.get_parameter("cached_pointcloud_topic").value
        if image_topic == cached_image_topic or pointcloud_topic == cached_pointcloud_topic:
            raise ValueError("A cache input and output topic must not be the same")

        cache_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.image_publisher = self.create_publisher(Image, cached_image_topic, cache_qos)
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2, cached_pointcloud_topic, cache_qos
        )
        self.latest_image = None
        self.latest_pointcloud = None
        self.pair_ready = False
        self.image_subscriber_count = 0
        self.pointcloud_subscriber_count = 0

        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, pointcloud_topic, self.pointcloud_callback,
            qos_profile_sensor_data,
        )
        # Sensor-data consumers generally request VOLATILE durability. Resend
        # once when a new consumer joins, without repeatedly inserting a stale
        # cloud into OctoMap while the camera is offline.
        self.create_timer(0.25, self.republish_for_new_subscribers)
        self.get_logger().info(
            f"Caching latest RGB {image_topic} -> {cached_image_topic}; "
            f"point cloud {pointcloud_topic} -> {cached_pointcloud_topic}"
        )

    def image_callback(self, message):
        if message.width == 0 or message.height == 0 or not message.data:
            return
        self.latest_image = message
        self.image_publisher.publish(message)
        self.publish_first_complete_pair()

    def pointcloud_callback(self, message):
        if message.width == 0 or message.height == 0 or not message.data:
            return
        self.latest_pointcloud = message
        self.pointcloud_publisher.publish(message)
        self.publish_first_complete_pair()

    def publish_first_complete_pair(self):
        if self.pair_ready or self.latest_image is None or self.latest_pointcloud is None:
            return
        self.pair_ready = True
        # Both cached topics become fresh together, even when one camera stream
        # started significantly earlier than the other.
        self.image_publisher.publish(self.latest_image)
        self.pointcloud_publisher.publish(self.latest_pointcloud)
        self.get_logger().info("RGB and point cloud are cached and ready")

    def republish_for_new_subscribers(self):
        image_count = self.image_publisher.get_subscription_count()
        if image_count > self.image_subscriber_count and self.latest_image is not None:
            self.image_publisher.publish(self.latest_image)
        self.image_subscriber_count = image_count

        pointcloud_count = self.pointcloud_publisher.get_subscription_count()
        if (
            pointcloud_count > self.pointcloud_subscriber_count
            and self.latest_pointcloud is not None
        ):
            self.pointcloud_publisher.publish(self.latest_pointcloud)
        self.pointcloud_subscriber_count = pointcloud_count


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseFrameCache()
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
