#!/usr/bin/env python3

from copy import deepcopy
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.task import Future
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from tf2_msgs.msg import TFMessage


class TargetPosePublisher(Node):
    """
    Gazebo Sim内のee_targetモデルの位置・姿勢を取得し、
    geometry_msgs/msg/PoseStampedとして配信するノード。

    入力トピック:
        /world/default/pose/info

    入力型:
        tf2_msgs/msg/TFMessage

    出力トピック:
        /ee_target_pose

    出力型:
        geometry_msgs/msg/PoseStamped
    """

    def __init__(self) -> None:
        super().__init__("target_pose_publisher")

        # Gazebo内で探すモデル名
        self.declare_parameter(
            "target_name",
            "ee_target",
        )

        # ros_gz_bridgeによってROS 2側へ変換された
        # Gazeboのモデル姿勢トピック
        self.declare_parameter(
            "gazebo_pose_topic",
            "/world/default/pose/info",
        )

        # QP制御ノードへ渡す目標姿勢トピック
        self.declare_parameter(
            "output_topic",
            "/ee_target_pose",
        )

        # RViz Interactive Markerからの連続目標。
        self.declare_parameter(
            "command_topic",
            "/ee_target_pose_command",
        )
        self.declare_parameter(
            "set_pose_service",
            "/world/default/set_pose",
        )
        self.declare_parameter(
            "command_override_duration",
            0.2,
        )

        # Gazeboから姿勢更新が来ない停止中でも、最新目標を
        # QPコントローラへ送り続ける周期。
        self.declare_parameter(
            "publish_rate",
            50.0,
        )

        self.target_name = str(
            self.get_parameter("target_name").value
        )

        self.gazebo_pose_topic = str(
            self.get_parameter("gazebo_pose_topic").value
        )

        self.output_topic = str(
            self.get_parameter("output_topic").value
        )

        self.command_topic = str(
            self.get_parameter("command_topic").value
        )
        self.set_pose_service = str(
            self.get_parameter("set_pose_service").value
        )
        self.command_override_duration = max(
            0.0,
            float(
                self.get_parameter("command_override_duration").value
            ),
        )

        self.publish_rate = float(
            self.get_parameter("publish_rate").value
        )

        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be greater than zero")

        # ee_targetの位置姿勢をPublish
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            self.output_topic,
            10,
        )

        # Gazebo内の全モデルの位置姿勢をSubscribe
        self.pose_subscription = self.create_subscription(
            TFMessage,
            self.gazebo_pose_topic,
            self.pose_info_callback,
            10,
        )
        self.command_subscription = self.create_subscription(
            PoseStamped,
            self.command_topic,
            self.target_command_callback,
            10,
        )
        self.set_pose_client = self.create_client(
            SetEntityPose,
            self.set_pose_service,
        )

        # 前回のコールバックでターゲットが見つかったか
        self.last_found = False

        # Gazeboから最後に受信したee_target姿勢。
        # ターゲットが停止してもこの姿勢を定期配信する。
        self.latest_target_pose: Optional[PoseStamped] = None
        self.latest_command_pose: Optional[PoseStamped] = None
        self.command_override_until = 0.0
        self.command_sequence = 0
        self.sent_command_sequence = 0
        self.pending_set_pose: Optional[Future] = None
        self.service_wait_logged = False
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.update_target,
        )

        self.get_logger().info(
            "Target pose publisher started."
        )
        self.get_logger().info(
            f"Target model name: {self.target_name}"
        )
        self.get_logger().info(
            f"Input pose topic: {self.gazebo_pose_topic}"
        )
        self.get_logger().info(
            f"Output target topic: {self.output_topic} "
            f"(continuous {self.publish_rate:g} Hz)"
        )
        self.get_logger().info(
            f"Live target command: {self.command_topic} -> "
            f"Gazebo model '{self.target_name}'"
        )

    def pose_info_callback(
        self,
        message: TFMessage,
    ) -> None:
        """
        Gazeboから受信したTFMessageの中から、
        ee_targetに対応する姿勢を探してPublishする。
        """

        target_transform = self.find_target_transform(
            message
        )

        # ターゲットが見つからなかった場合
        if target_transform is None:
            if self.last_found:
                self.get_logger().warning(
                    f"Target '{self.target_name}' "
                    "disappeared from Gazebo pose information."
                )

            self.last_found = False
            return

        # 初めて見つかったときだけログを表示
        if not self.last_found:
            self.get_logger().info(
                f"Found Gazebo target "
                f"'{self.target_name}'."
            )

        self.last_found = True

        # Interactive Markerの連続指令中は、set_poseがGazeboの
        # pose/infoへ反映されるまでの遅れた姿勢で上書きしない。
        if time.monotonic() < self.command_override_until:
            return

        pose_message = PoseStamped()

        # Gazebo world座標系を基準にする
        pose_message.header.frame_id = "world"

        pose_message.pose.position.x = (
            target_transform.transform.translation.x
        )
        pose_message.pose.position.y = (
            target_transform.transform.translation.y
        )
        pose_message.pose.position.z = (
            target_transform.transform.translation.z
        )

        pose_message.pose.orientation.x = (
            target_transform.transform.rotation.x
        )
        pose_message.pose.orientation.y = (
            target_transform.transform.rotation.y
        )
        pose_message.pose.orientation.z = (
            target_transform.transform.rotation.z
        )
        pose_message.pose.orientation.w = (
            target_transform.transform.rotation.w
        )

        self.latest_target_pose = pose_message

        # 移動中の遅延を減らすため、Gazeboから新しい姿勢を
        # 受け取った瞬間にも送信する。
        self.publish_latest_target()

    def target_command_callback(self, message: PoseStamped) -> None:
        """Accept a live RViz target and forward it to QP and Gazebo."""

        frame_id = message.header.frame_id.strip("/")
        if frame_id != "world":
            self.get_logger().warning(
                "Live target command must use frame_id='world'; "
                f"received '{message.header.frame_id}'."
            )
            return

        values = (
            float(message.pose.position.x),
            float(message.pose.position.y),
            float(message.pose.position.z),
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning(
                "Rejected a live target containing NaN or Inf."
            )
            return

        command_pose = deepcopy(message)
        command_pose.header.frame_id = "world"
        command_pose.header.stamp.sec = 0
        command_pose.header.stamp.nanosec = 0

        self.latest_command_pose = command_pose
        self.latest_target_pose = command_pose
        self.command_sequence += 1
        self.command_override_until = (
            time.monotonic() + self.command_override_duration
        )

        # QPへはドラッグ中の各姿勢を即時送信する。
        self.publish_latest_target()
        self.update_gazebo_target()

    def update_target(self) -> None:
        """Continuously publish the target and apply pending Gazebo commands."""

        self.publish_latest_target()
        self.update_gazebo_target()

    def update_gazebo_target(self) -> None:
        """Apply the newest unsent interactive target to the Gazebo model."""

        if self.latest_command_pose is None:
            return
        if self.sent_command_sequence >= self.command_sequence:
            return
        if self.pending_set_pose is not None and not self.pending_set_pose.done():
            return

        if not self.set_pose_client.service_is_ready():
            if not self.service_wait_logged:
                self.get_logger().info(
                    f"Waiting for Gazebo service {self.set_pose_service}."
                )
                self.service_wait_logged = True
            return

        self.service_wait_logged = False

        request = SetEntityPose.Request()
        request.entity.name = self.target_name
        request.entity.type = Entity.MODEL
        request.pose = deepcopy(self.latest_command_pose.pose)

        sent_sequence = self.command_sequence
        self.sent_command_sequence = sent_sequence
        self.pending_set_pose = self.set_pose_client.call_async(request)
        self.pending_set_pose.add_done_callback(
            lambda future: self.set_pose_done(future, sent_sequence)
        )

    def set_pose_done(self, future: Future, sent_sequence: int) -> None:
        """Check the Gazebo pose request and retain newer queued commands."""

        self.pending_set_pose = None
        try:
            response = future.result()
        except Exception as error:
            self.sent_command_sequence = min(
                self.sent_command_sequence,
                sent_sequence - 1,
            )
            self.get_logger().warning(
                f"Failed to move Gazebo target: {error}"
            )
            return

        if response is None or not response.success:
            self.sent_command_sequence = min(
                self.sent_command_sequence,
                sent_sequence - 1,
            )
            self.get_logger().warning(
                f"Gazebo target '{self.target_name}' was not found."
            )
            return

        # サービス応答待ちの間に次の姿勢が来ていれば即送る。
        self.update_gazebo_target()

    def publish_latest_target(self) -> None:
        """Publish the latest target using the newest available TF."""

        if self.latest_target_pose is None:
            return

        pose_message = deepcopy(self.latest_target_pose)

        # Pose送信時の /clock は robot_state_publisher の最新TFより
        # 数ms先になることがある。その時刻を指定するとtf2が
        # "extrapolation into the future" で world -> link0_1 変換を
        # 拒否する。ゼロ時刻はtf2の「最新利用可能TF」を表す。
        pose_message.header.stamp.sec = 0
        pose_message.header.stamp.nanosec = 0
        self.pose_publisher.publish(pose_message)

    def find_target_transform(
        self,
        message: TFMessage,
    ) -> Optional[TransformStamped]:
        """
        TFMessage内からee_targetに対応するTransformStampedを探す。

        Gazeboやros_gz_bridgeの形式によって、
        child_frame_idは次のようになる可能性がある。

        ee_target
        ee_target::target_link
        ee_target/target_link
        world/ee_target
        world/ee_target/target_link
        """

        for transform in message.transforms:
            child_name = transform.child_frame_id

            if self.is_target_name(child_name):
                return transform

        return None

    def is_target_name(
        self,
        child_name: str,
    ) -> bool:
        """
        child_frame_idが対象モデルまたは対象リンクか判定する。
        """

        normalized_name = child_name.strip("/")

        candidates = {
            self.target_name,
            f"{self.target_name}::target_link",
            f"{self.target_name}/target_link",
            f"world/{self.target_name}",
            f"world/{self.target_name}/target_link",
            f"world/{self.target_name}::target_link",
        }

        if normalized_name in candidates:
            return True

        # 名前空間などが前に付いている場合
        if normalized_name.endswith(
            f"/{self.target_name}"
        ):
            return True

        if normalized_name.endswith(
            f"/{self.target_name}/target_link"
        ):
            return True

        if normalized_name.endswith(
            f"/{self.target_name}::target_link"
        ):
            return True

        if normalized_name.endswith(
            f"{self.target_name}::target_link"
        ):
            return True

        return False


def main(args=None) -> None:
    rclpy.init(args=args)

    node = TargetPosePublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            "Target pose publisher stopped."
        )

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
