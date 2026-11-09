#!/usr/bin/env python3

"""順運動学で計算したEE位置をGazebo Simの青球で可視化する。"""

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.task import Future
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

# PoseStampedのTF変換型を登録するために必要
import tf2_geometry_msgs  # noqa: F401


class ArmVisualPublisher(Node):
    """FK位置をworldへ変換し、Gazebo可視化モデルの姿勢を更新する。"""

    def __init__(self) -> None:
        super().__init__("arm_visual_publisher")

        self.declare_parameter("fk_pose_topic", "/fk_ee_pose")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("model_name", "viz_sphere")
        self.declare_parameter(
            "set_pose_service",
            "/world/default/set_pose",
        )
        self.declare_parameter("update_rate", 30.0)

        fk_pose_topic = str(
            self.get_parameter("fk_pose_topic").value
        )
        self.world_frame = str(
            self.get_parameter("world_frame").value
        ).strip("/")
        self.model_name = str(
            self.get_parameter("model_name").value
        )
        set_pose_service = str(
            self.get_parameter("set_pose_service").value
        )
        update_rate = float(
            self.get_parameter("update_rate").value
        )

        if update_rate <= 0.0:
            raise ValueError("update_rateは0より大きい値にしてください。")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.latest_fk_pose: Optional[PoseStamped] = None
        self.pending_request: Optional[Future] = None
        self.service_wait_logged = False
        self.transform_warning_logged = False
        self.first_update_logged = False

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            fk_pose_topic,
            self.fk_pose_callback,
            10,
        )

        self.set_pose_client = self.create_client(
            SetEntityPose,
            set_pose_service,
        )

        self.timer = self.create_timer(
            1.0 / update_rate,
            self.update_visual,
        )

        self.get_logger().info(
            "FK EE visualizer started: "
            f"{fk_pose_topic} -> Gazebo model '{self.model_name}'"
        )

    def fk_pose_callback(self, message: PoseStamped) -> None:
        """QPノードから最新のFK位置を保持する。"""

        self.latest_fk_pose = message

    def update_visual(self) -> None:
        """最新FK位置をworldへ変換してGazeboモデルへ反映する。"""

        if self.latest_fk_pose is None:
            return

        if not self.set_pose_client.service_is_ready():
            if not self.service_wait_logged:
                self.get_logger().info(
                    "Gazebo set_pose serviceを待っています。"
                )
                self.service_wait_logged = True
            return

        self.service_wait_logged = False

        if (
            self.pending_request is not None
            and not self.pending_request.done()
        ):
            return

        # 最新TFを使う。FK位置はlink0_1に固定された点なので、可視化では
        # メッセージ生成時刻より最新のベース姿勢を優先する。
        local_pose = PoseStamped()
        local_pose.header.frame_id = (
            self.latest_fk_pose.header.frame_id
        )
        local_pose.pose = self.latest_fk_pose.pose

        try:
            world_pose = self.tf_buffer.transform(
                local_pose,
                self.world_frame,
                timeout=Duration(seconds=0.02),
            )
            self.transform_warning_logged = False
        except TransformException as error:
            if not self.transform_warning_logged:
                self.get_logger().warning(
                    "FK位置をworldへ変換できません: "
                    f"{error}"
                )
                self.transform_warning_logged = True
            return

        request = SetEntityPose.Request()
        request.entity.name = self.model_name
        request.entity.type = Entity.MODEL
        request.pose = world_pose.pose

        self.pending_request = self.set_pose_client.call_async(
            request
        )
        self.pending_request.add_done_callback(
            self.pose_update_done
        )

    def pose_update_done(self, future: Future) -> None:
        """Gazebo set_pose応答を確認する。"""

        try:
            response = future.result()
        except Exception as error:  # ROS service通信失敗もログへ残す
            self.get_logger().warning(
                f"Gazebo可視化球の更新に失敗しました: {error}"
            )
            return

        if response is None or not response.success:
            self.get_logger().warning(
                "Gazebo内に可視化モデル "
                f"'{self.model_name}' が見つかりません。"
            )
            return

        if not self.first_update_logged:
            self.get_logger().info(
                "順運動学EE位置の青球表示を開始しました。"
            )
            self.first_update_logged = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmVisualPublisher()

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
