#!/usr/bin/env python3
"""MR development copy of rviz_ee_target.py; currently uses RViz manual input.

Add MR reception and coordinate conversion here, then update the marker pose
with server.setPose()/applyChanges() and publish the same target to QP.
No MR subscription or coordinate conversion is implemented in this copy yet."""

import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.node import Node
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)


class MrEeTarget(Node):
    """Expose a movable RViz marker and publish its position as a QP target."""

    def __init__(self):
        super().__init__("mr_ee_target")

        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("target_topic", "/ee_target_pose")
        self.declare_parameter("clicked_point_topic", "/mr/clicked_point")
        self.declare_parameter("interactive_marker_namespace", "mr_ee_target_marker")
        self.declare_parameter("initial_x", 0.35)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_z", 0.65)
        self.declare_parameter("marker_scale", 0.12)
        self.declare_parameter("publish_while_dragging", False)
        self.declare_parameter("drag_update_rate", 30.0)

        self.frame_id = str(self.get_parameter("frame_id").value).strip("/")
        target_topic = str(self.get_parameter("target_topic").value)
        clicked_point_topic = str(
            self.get_parameter("clicked_point_topic").value
        )
        marker_namespace = str(
            self.get_parameter("interactive_marker_namespace").value
        ).strip("/")
        marker_scale = max(
            0.03, float(self.get_parameter("marker_scale").value)
        )
        self.publish_while_dragging = bool(
            self.get_parameter("publish_while_dragging").value
        )
        drag_update_rate = float(
            self.get_parameter("drag_update_rate").value
        )
        self.minimum_drag_update_period = (
            1.0 / drag_update_rate if drag_update_rate > 0.0 else 0.0
        )
        self.last_drag_publish_time = 0.0

        if not self.frame_id:
            raise ValueError("frame_id must not be empty")
        if not marker_namespace:
            raise ValueError("interactive_marker_namespace must not be empty")

        self.target_publisher = self.create_publisher(PoseStamped, target_topic, 10)
        self.clicked_point_subscription = self.create_subscription(
            PointStamped,
            clicked_point_topic,
            self.clicked_point_callback,
            10,
        )

        self.server = InteractiveMarkerServer(self, marker_namespace)
        interactive_marker = self.make_interactive_marker(marker_scale)
        self.server.insert(
            interactive_marker,
            feedback_callback=self.feedback_callback,
        )
        self.server.applyChanges()

        publish_mode = (
            f"streamed while dragging at up to {drag_update_rate:g} Hz"
            if self.publish_while_dragging
            else "published when the drag finishes"
        )
        self.get_logger().info(
            "MR development EE target ready (RViz manual input only). Use the Interact tool to move the red marker; "
            f"the target is {publish_mode}. "
            f"Fallback: Publish Point on {clicked_point_topic}."
        )

    def make_interactive_marker(self, scale):
        marker = InteractiveMarker()
        marker.header.frame_id = self.frame_id
        marker.name = "ee_target"
        marker.description = "EE target"
        marker.scale = scale
        marker.pose.position.x = float(self.get_parameter("initial_x").value)
        marker.pose.position.y = float(self.get_parameter("initial_y").value)
        marker.pose.position.z = float(self.get_parameter("initial_z").value)
        marker.pose.orientation.w = 1.0

        visual = Marker()
        visual.type = Marker.SPHERE
        visual.action = Marker.ADD
        visual.scale.x = 0.45 * scale
        visual.scale.y = 0.45 * scale
        visual.scale.z = 0.45 * scale
        visual.color.r = 1.0
        visual.color.g = 0.05
        visual.color.b = 0.05
        visual.color.a = 0.9

        visual_control = InteractiveMarkerControl()
        visual_control.name = "target_visual"
        visual_control.always_visible = True
        visual_control.interaction_mode = InteractiveMarkerControl.NONE
        visual_control.markers.append(visual)
        marker.controls.append(visual_control)

        self.add_axis_control(marker, "move_x", 1.0, 1.0, 0.0, 0.0)
        self.add_axis_control(marker, "move_y", 1.0, 0.0, 0.0, 1.0)
        self.add_axis_control(marker, "move_z", 1.0, 0.0, 1.0, 0.0)
        return marker

    @staticmethod
    def add_axis_control(marker, name, w, x, y, z):
        control = InteractiveMarkerControl()
        control.name = name
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        control.orientation.w = w / norm
        control.orientation.x = x / norm
        control.orientation.y = y / norm
        control.orientation.z = z / norm
        control.orientation_mode = InteractiveMarkerControl.FIXED
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)

    def feedback_callback(self, feedback):
        if (
            self.publish_while_dragging
            and feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE
        ):
            now = time.monotonic()
            if (
                now - self.last_drag_publish_time
                < self.minimum_drag_update_period
            ):
                return
            self.last_drag_publish_time = now
            self.publish_target(
                feedback.header.frame_id,
                feedback.pose.position,
                log_result=False,
            )
            return

        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return
        self.last_drag_publish_time = time.monotonic()
        self.publish_target(
            feedback.header.frame_id,
            feedback.pose.position,
        )

    def clicked_point_callback(self, message):
        self.publish_target(message.header.frame_id, message.point)

    def publish_target(self, frame_id, position, log_result=True):
        values = (float(position.x), float(position.y), float(position.z))
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning("Rejected a target containing NaN or Inf")
            return

        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = frame_id.strip("/") or self.frame_id
        target.pose.position.x = values[0]
        target.pose.position.y = values[1]
        target.pose.position.z = values[2]
        target.pose.orientation.w = 1.0
        self.target_publisher.publish(target)

        if log_result:
            self.get_logger().info(
                f"Published EE target: frame={target.header.frame_id}, "
                f"xyz=({values[0]:.3f}, {values[1]:.3f}, {values[2]:.3f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = MrEeTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
