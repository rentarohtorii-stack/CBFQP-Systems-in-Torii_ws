#!/usr/bin/env python3
"""Expose simple open/close services for the ros2_control gripper action."""

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger


class GripperCommandServer(Node):
    """Translate Trigger service calls into GripperCommand action goals."""

    def __init__(self):
        super().__init__("gripper_command_server")

        self.declare_parameter("action_name", "gripper_controller/gripper_cmd")
        self.declare_parameter("open_position", -1.0471)
        self.declare_parameter("closed_position", 0.261799)
        self.declare_parameter("max_effort", 0.4)
        self.declare_parameter("server_timeout", 2.0)

        self.action_name = str(self.get_parameter("action_name").value)
        self.open_position = float(self.get_parameter("open_position").value)
        self.closed_position = float(self.get_parameter("closed_position").value)
        self.max_effort = float(self.get_parameter("max_effort").value)
        self.server_timeout = float(self.get_parameter("server_timeout").value)

        self.action_client = ActionClient(
            self, GripperCommand, self.action_name
        )
        self.open_service = self.create_service(
            Trigger, "gripper/open", self.open_callback
        )
        self.close_service = self.create_service(
            Trigger, "gripper/close", self.close_callback
        )

        self.get_logger().info(
            "Gripper services ready: gripper/open, gripper/close "
            f"-> {self.action_name} "
            f"(open={self.open_position:.6f}, close={self.closed_position:.6f})"
        )

    def open_callback(self, request, response):
        del request
        return self.send_command("open", self.open_position, response)

    def close_callback(self, request, response):
        del request
        return self.send_command("close", self.closed_position, response)

    def send_command(self, command_name, position, response):
        if not self.action_client.wait_for_server(timeout_sec=self.server_timeout):
            response.success = False
            response.message = (
                f"Gripper action server is not ready: {self.action_name}"
            )
            self.get_logger().warning(response.message)
            return response

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = self.max_effort

        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed_future: self.goal_response_callback(
                command_name, position, completed_future
            )
        )

        response.success = True
        response.message = (
            f"Sent gripper {command_name} command: position={position:.6f} rad"
        )
        self.get_logger().info(response.message)
        return response

    def goal_response_callback(self, command_name, position, future):
        try:
            goal_handle = future.result()
        except Exception as exception:  # noqa: BLE001 - log ROS future errors.
            self.get_logger().error(
                f"Failed to send gripper {command_name} goal: {exception}"
            )
            return

        if not goal_handle.accepted:
            self.get_logger().error(
                f"Gripper {command_name} goal was rejected "
                f"(position={position:.6f})"
            )
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed_future: self.result_callback(
                command_name, completed_future
            )
        )

    def result_callback(self, command_name, future):
        try:
            result = future.result().result
        except Exception as exception:  # noqa: BLE001 - log ROS future errors.
            self.get_logger().error(
                f"Gripper {command_name} action failed: {exception}"
            )
            return

        if result.reached_goal:
            self.get_logger().info(
                f"Gripper {command_name} completed: "
                f"position={result.position:.6f}, effort={result.effort:.3f}"
            )
        elif result.stalled:
            # A grasped object can stop the fingers before the closed limit.
            self.get_logger().info(
                f"Gripper {command_name} stalled (object may be grasped): "
                f"position={result.position:.6f}, effort={result.effort:.3f}"
            )
        else:
            self.get_logger().warning(
                f"Gripper {command_name} finished without reaching its goal: "
                f"position={result.position:.6f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = GripperCommandServer()
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
