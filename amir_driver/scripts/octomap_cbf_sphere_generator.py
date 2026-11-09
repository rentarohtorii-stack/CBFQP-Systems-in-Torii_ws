#!/usr/bin/env python3
"""Convert occupied OctoMap voxels into conservative CBF obstacle spheres."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class OctomapCbfSphereGenerator(Node):
    """Represent every occupied voxel by its circumscribed sphere."""

    def __init__(self):
        super().__init__("octomap_cbf_sphere_generator")

        self.declare_parameter("input_topic", "/occupied_cells_vis_array")
        self.declare_parameter("output_topic", "/cbf/obstacle_spheres")
        self.declare_parameter("sphere_padding", 0.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.sphere_padding = max(
            0.0, float(self.get_parameter("sphere_padding").value)
        )

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(
            MarkerArray, output_topic, latched_qos
        )
        self.subscription = self.create_subscription(
            MarkerArray, input_topic, self.marker_callback, latched_qos
        )
        self.previous_marker_count = 0

        self.get_logger().info(
            f"OctoMap CBF spheres: {input_topic} -> {output_topic}"
        )

    def marker_callback(self, message):
        output = MarkerArray()
        marker_id = 0
        last_header = None

        for occupied_cells in message.markers:
            if occupied_cells.type != Marker.CUBE_LIST:
                continue
            if not occupied_cells.points:
                continue

            voxel_diagonal = math.sqrt(
                occupied_cells.scale.x ** 2
                + occupied_cells.scale.y ** 2
                + occupied_cells.scale.z ** 2
            )
            sphere_diameter = voxel_diagonal + 2.0 * self.sphere_padding
            if sphere_diameter <= 0.0:
                continue

            spheres = Marker()
            spheres.header = occupied_cells.header
            spheres.ns = "cbf_octomap_spheres"
            spheres.id = marker_id
            spheres.type = Marker.SPHERE_LIST
            spheres.action = Marker.ADD
            spheres.pose = occupied_cells.pose
            spheres.scale.x = sphere_diameter
            spheres.scale.y = sphere_diameter
            spheres.scale.z = sphere_diameter
            spheres.color.r = 1.0
            spheres.color.g = 0.15
            spheres.color.b = 0.05
            spheres.color.a = 0.7
            spheres.points = occupied_cells.points
            output.markers.append(spheres)

            marker_id += 1
            last_header = occupied_cells.header

        for stale_id in range(marker_id, self.previous_marker_count):
            stale = Marker()
            if last_header is not None:
                stale.header = last_header
            stale.ns = "cbf_octomap_spheres"
            stale.id = stale_id
            stale.action = Marker.DELETE
            output.markers.append(stale)

        self.publisher.publish(output)
        self.previous_marker_count = marker_id


def main(args=None):
    rclpy.init(args=args)
    node = OctomapCbfSphereGenerator()
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
