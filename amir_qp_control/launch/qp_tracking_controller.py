#!/usr/bin/env python3

import math

import cvxpy as cp
import numpy as np
import rclpy

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
# PoseStampedのTF変換型を登録するために必要
import tf2_geometry_msgs  # noqa: F401

from kinematics import AmirKinematics
from joint_speed_profile import project_joint_speeds, validate_joint_speeds
from position_reference import PositionReference, cbf_command_compatible


class QPTrackingController(Node):

    def __init__(self) -> None:
        super().__init__("qp_tracking_controller")
        self.declare_parameter('qp_enabled', True)
        self.declare_parameter('stop_topic', '/amir/qp_stop')
        self.qp_enabled = bool(self.get_parameter('qp_enabled').value)
        self.stop_pub = self.create_publisher(
            Bool, str(self.get_parameter('stop_topic').value), 1)
        self.enable_service = self.create_service(
            SetBool, '~/set_enabled', self.set_enabled_callback)

        # ======================================================
        # ROS 2パラメータ
        # ======================================================

        self.declare_parameter(
            "joint_names",
            [
                "Joint_1",
                "Joint_2",
                "Joint_3",
                "Joint_4",
            ],
        )

        self.declare_parameter(
            "joint_state_topic",
            "/joint_states",
        )

        self.declare_parameter(
            "target_topic",
            "/ee_target_pose",
        )

        self.declare_parameter(
            "command_topic",
            "/forward_position_controller/commands",
        )
        self.declare_parameter("velocity_command_topic", "/forward_velocity_controller/commands")
        self.declare_parameter("joint_speed_mode", "qp")
        self.declare_parameter("joint_speeds", [0.2, 0.2, 0.2, 0.2])
        self.joint_speed_mode = str(self.get_parameter("joint_speed_mode").value)
        if self.joint_speed_mode not in {"qp", "fixed"}:
            raise ValueError("joint_speed_modeはqpまたはfixedにしてください。")
        self.joint_speeds = validate_joint_speeds(self.get_parameter("joint_speeds").value)
        self.nominal_qdot = np.zeros(4)
        self.declare_parameter("position_command_mode", "integrated")
        self.declare_parameter("max_position_lead", 0.02)
        self.position_command_mode = str(self.get_parameter("position_command_mode").value)
        if self.position_command_mode not in {"integrated", "measured"}:
            raise ValueError("position_command_modeはintegratedまたはmeasuredにしてください。")
        self.max_position_lead = float(self.get_parameter("max_position_lead").value)
        self.position_reference = PositionReference(self.max_position_lead)

        # 順運動学で計算した現在のEE位置の可視化用Topic
        self.declare_parameter(
            "fk_pose_topic",
            "/fk_ee_pose",
        )

        # AmirKinematicsが位置を計算する基準座標系
        self.declare_parameter(
            "kinematics_frame",
            "link0_1",
        )

        self.declare_parameter(
            "control_frequency",
            50.0,
        )

        self.declare_parameter(
            "kp",
            10.0,
        )

        self.declare_parameter(
            "regularization",
            1.0e-3,
        )
        # Cost weights for Cartesian tracking in kinematics_frame, not joint axes.
        for axis in "xyz":
            self.declare_parameter(f"tracking_weight_{axis}", 1.0)

        self.declare_parameter(
            "max_cartesian_velocity",
            2.0,
        )

        self.declare_parameter(
            "max_joint_velocity",
            10.0,
        )

        self.declare_parameter(
            "position_tolerance",
            0.005,
        )
        self.declare_parameter("stop_after_reaching_target", False)
        self.declare_parameter("target_hold_time", 0.2)
        self.declare_parameter("enable_cbf_after_reaching_target", False)

        # ======================================================
        # CBF障害物回避パラメータ
        # ======================================================

        self.declare_parameter("cbf_enabled", True)
        self.declare_parameter(
            "cbf_obstacle_topic",
            "/cbf/obstacle_spheres",
        )
        self.declare_parameter(
            "arm_cbf_marker_topic",
            "/cbf/arm_spheres",
        )
        self.declare_parameter(
            "active_cbf_marker_topic",
            "/cbf/active_constraints",
        )
        self.declare_parameter(
            "arm_cbf_radii",
            [0.05] * 8,
        )
        self.declare_parameter("cbf_safety_margin", 0.02)
        self.declare_parameter("cbf_activation_distance", 0.05)
        self.declare_parameter("cbf_alpha", 5.0)
        self.declare_parameter("max_active_cbf_constraints", 128)
        self.declare_parameter("cbf_require_obstacle_map", True)

        # ======================================================
        # パラメータ取得
        # ======================================================

        self.joint_names = list(
            self.get_parameter("joint_names").value
        )

        joint_state_topic = str(
            self.get_parameter(
                "joint_state_topic"
            ).value
        )

        target_topic = str(
            self.get_parameter(
                "target_topic"
            ).value
        )

        command_topic = str(
            self.get_parameter(
                "command_topic"
            ).value
        )

        fk_pose_topic = str(
            self.get_parameter(
                "fk_pose_topic"
            ).value
        )

        self.kinematics_frame = str(
            self.get_parameter(
                "kinematics_frame"
            ).value
        )

        self.frequency = float(
            self.get_parameter(
                "control_frequency"
            ).value
        )

        self.kp = float(
            self.get_parameter("kp").value
        )
        self.tracking_weights = np.array([
            float(self.get_parameter(f"tracking_weight_{axis}").value)
            for axis in "xyz"
        ])
        if (
            not np.all(np.isfinite(self.tracking_weights))
            or np.any(self.tracking_weights < 0.0)
            or not np.any(self.tracking_weights > 0.0)
        ):
            raise ValueError(
                "tracking_weight_x/y/zは0以上の有限値で、少なくとも1軸は正にしてください。"
            )

        self.regularization = float(
            self.get_parameter(
                "regularization"
            ).value
        )

        self.max_cartesian_velocity = float(
            self.get_parameter(
                "max_cartesian_velocity"
            ).value
        )

        max_joint_velocity = float(
            self.get_parameter(
                "max_joint_velocity"
            ).value
        )

        self.position_tolerance = float(
            self.get_parameter(
                "position_tolerance"
            ).value
        )
        self.stop_after_reaching_target = bool(
            self.get_parameter("stop_after_reaching_target").value
        )
        self.target_hold_time = float(
            self.get_parameter("target_hold_time").value
        )
        self.enable_cbf_after_reaching_target = bool(
            self.get_parameter("enable_cbf_after_reaching_target").value
        )

        self.cbf_enabled = bool(
            self.get_parameter("cbf_enabled").value
        )
        cbf_obstacle_topic = str(
            self.get_parameter("cbf_obstacle_topic").value
        )
        arm_cbf_marker_topic = str(
            self.get_parameter("arm_cbf_marker_topic").value
        )
        active_cbf_marker_topic = str(
            self.get_parameter("active_cbf_marker_topic").value
        )
        self.arm_cbf_radii = np.asarray(
            self.get_parameter("arm_cbf_radii").value,
            dtype=float,
        )
        self.cbf_safety_margin = float(
            self.get_parameter("cbf_safety_margin").value
        )
        self.cbf_activation_distance = float(
            self.get_parameter("cbf_activation_distance").value
        )
        self.cbf_alpha = float(
            self.get_parameter("cbf_alpha").value
        )
        self.max_active_cbf_constraints = int(
            self.get_parameter("max_active_cbf_constraints").value
        )
        self.cbf_require_obstacle_map = bool(
            self.get_parameter("cbf_require_obstacle_map").value
        )

        if self.frequency <= 0.0:
            raise ValueError(
                "control_frequencyは"
                "0より大きい値にしてください。"
            )

        if self.target_hold_time < 0.0:
            raise ValueError("target_hold_timeは0以上にしてください。")

        if len(self.joint_names) != 4:
            raise ValueError(
                "joint_namesはJoint_1～Joint_4の"
                "4要素にしてください。"
            )

        if self.arm_cbf_radii.shape != (8,):
            raise ValueError("arm_cbf_radiiはR1～R8の8要素にしてください。")

        if not np.all(np.isfinite(self.arm_cbf_radii)) or np.any(
            self.arm_cbf_radii <= 0.0
        ):
            raise ValueError("arm_cbf_radiiは正の有限値にしてください。")

        if self.cbf_safety_margin < 0.0:
            raise ValueError("cbf_safety_marginは0以上にしてください。")

        if self.cbf_activation_distance < 0.0:
            raise ValueError("cbf_activation_distanceは0以上にしてください。")

        if self.cbf_alpha <= 0.0:
            raise ValueError("cbf_alphaは0より大きい値にしてください。")

        if self.max_active_cbf_constraints < 0:
            raise ValueError("max_active_cbf_constraintsは0以上にしてください。")

        self.dt = 1.0 / self.frequency

        # ======================================================
        # 関節角度・速度制限
        # ======================================================

        self.q_min = np.array(
            [
                -3.14,
                -2.13526034,
                -2.18362788,
                -1.78010,
            ],
            dtype=float,
        )

        self.q_max = np.array(
            [
                3.14,
                2.37079633,
                2.35619449,
                1.78010,
            ],
            dtype=float,
        )

        self.qdot_max = np.full(
            4,
            max_joint_velocity,
            dtype=float,
        )

        # ======================================================
        # 内部状態
        # ======================================================

        self.current_q = None
        # Joint_5はQPの4軸運動学には含めず、起動時の角度を保持する。
        self.joint5_hold_position = None
        self.controller_stopped = False
        self.target_reached_cycle_count = 0
        self.target_hold_cycles = max(
            1,
            int(np.ceil(self.target_hold_time * self.frequency)),
        )

        # 元のPoseStampedを保持する
        # target_callback内ではまだworld座標の可能性がある
        self.target_pose = None

        self.kinematics = AmirKinematics()

        self.received_joint_state_log = False
        self.received_target_log = False
        self.tf_warning_logged = False
        self.cbf_tf_warning_logged = False

        # frame_idごとに、OctoMap球中心と半径を保持する。
        self.obstacle_groups = {}
        self.received_obstacle_map = False
        self.active_cbf_constraint_count = 0
        self.minimum_cbf_clearance = float("inf")
        self.active_arm_indices = np.empty(0, dtype=int)
        self.active_obstacle_indices = np.empty(0, dtype=int)

        # ======================================================
        # TF2
        # ======================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # ======================================================
        # Subscriber
        # ======================================================

        self.joint_state_sub = self.create_subscription(
            JointState,
            joint_state_topic,
            self.joint_state_callback,
            10,
        )

        self.target_sub = self.create_subscription(
            PoseStamped,
            target_topic,
            self.target_callback,
            10,
        )

        cbf_qos = QoSProfile(depth=1)
        cbf_qos.reliability = ReliabilityPolicy.RELIABLE
        cbf_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.cbf_obstacle_sub = self.create_subscription(
            MarkerArray,
            cbf_obstacle_topic,
            self.cbf_obstacle_callback,
            cbf_qos,
        )

        # ======================================================
        # Publisher
        # ======================================================

        self.command_pub = self.create_publisher(
            Float64MultiArray,
            command_topic,
            10,
        )
        self.velocity_command_pub = self.create_publisher(
            Float64MultiArray, str(self.get_parameter("velocity_command_topic").value), 10)

        self.fk_pose_pub = self.create_publisher(
            PoseStamped,
            fk_pose_topic,
            10,
        )

        self.arm_cbf_marker_pub = self.create_publisher(
            MarkerArray,
            arm_cbf_marker_topic,
            cbf_qos,
        )

        self.active_cbf_marker_pub = self.create_publisher(
            MarkerArray,
            active_cbf_marker_topic,
            cbf_qos,
        )

        # ======================================================
        # 制御タイマー
        # ======================================================

        self.timer = self.create_timer(
            self.dt,
            self.control_callback,
        )

        self.get_logger().info(
            "QP tracking controller started."
        )

        self.get_logger().info(
            f"Joint state topic: {joint_state_topic}"
        )

        self.get_logger().info(
            f"Target topic: {target_topic}"
        )

        self.get_logger().info(
            f"Command topic: {command_topic}"
        )

        self.get_logger().info(
            f"FK pose topic: {fk_pose_topic}"
        )

        self.get_logger().info(
            f"Kinematics frame: {self.kinematics_frame}"
        )
        self.get_logger().info(
            f"Cartesian tracking weights [x,y,z]: {self.tracking_weights.tolist()}"
        )
        self.get_logger().info(
            f"Joint speed mode={self.joint_speed_mode}, magnitudes/caps={self.joint_speeds.tolist()} rad/s"
        )
        self.get_logger().info(
            f"Position command mode={self.position_command_mode}, "
            f"max_position_lead={self.max_position_lead:g} rad"
        )

        self.get_logger().info(
            "One-shot positioning: "
            f"enabled={self.stop_after_reaching_target}, "
            f"hold_time={self.target_hold_time:.3f}s, "
            "start_cbf_after_reaching="
            f"{self.enable_cbf_after_reaching_target}"
        )

        self.get_logger().info(
            "CBF-QP: "
            f"enabled={self.cbf_enabled}, "
            f"obstacles={cbf_obstacle_topic}, "
            f"activation={self.cbf_activation_distance:.3f}m, "
            f"margin={self.cbf_safety_margin:.3f}m, "
            f"alpha={self.cbf_alpha:g}"
        )

    def joint_state_callback(
        self,
        msg: JointState,
    ) -> None:
        """
        /joint_statesからJoint_1～Joint_4を、
        joint_namesで指定した順番に並べて取得する。
        """

        joint_position_map = {
            name: position
            for name, position in zip(
                msg.name,
                msg.position,
            )
        }

        missing_joints = [
            name
            for name in self.joint_names
            if name not in joint_position_map
        ]

        if missing_joints:
            return

        if "Joint_5" not in joint_position_map:
            return

        current_q = np.array(
            [
                joint_position_map[name]
                for name in self.joint_names
            ],
            dtype=float,
        )

        if not np.all(np.isfinite(current_q)):
            self.get_logger().warning(
                "joint_statesにNaNまたはInfが含まれています。"
            )
            return

        joint5_position = float(joint_position_map["Joint_5"])
        if not np.isfinite(joint5_position):
            self.get_logger().warning(
                "Joint_5の関節角度にNaNまたはInfが含まれています。"
            )
            return

        # 最初に受信したJoint_5の角度を固定目標として保持する。
        if self.joint5_hold_position is None:
            self.joint5_hold_position = joint5_position

        self.current_q = current_q

        if not self.received_joint_state_log:
            self.get_logger().info(
                "Joint_1～Joint_4の関節状態を受信しました。"
            )
            self.received_joint_state_log = True

    def target_callback(
        self,
        msg: PoseStamped,
    ) -> None:
        """
        /ee_target_poseから目標PoseStampedを受信する。

        座標変換は制御周期内で行う。
        """

        if not msg.header.frame_id:
            self.get_logger().warning(
                "ee_target_poseのframe_idが空です。"
            )
            return

        position = np.array(
            [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ],
            dtype=float,
        )

        if not np.all(np.isfinite(position)):
            self.get_logger().warning(
                "ee_target_poseにNaNまたはInfが含まれています。"
            )
            return

        self.target_pose = msg

        if not self.received_target_log:
            self.get_logger().info(
                "追従ターゲットを受信しました: "
                f"frame={msg.header.frame_id}"
            )
            self.received_target_log = True

    @staticmethod
    def quaternion_to_rotation_matrix(quaternion) -> np.ndarray:
        """geometry_msgs/Quaternionを3×3回転行列へ変換する。"""

        x = float(quaternion.x)
        y = float(quaternion.y)
        z = float(quaternion.z)
        w = float(quaternion.w)
        norm = math.sqrt(x * x + y * y + z * z + w * w)

        if norm <= 1.0e-12:
            return np.eye(3, dtype=float)

        x /= norm
        y /= norm
        z /= norm
        w /= norm

        return np.array(
            [
                [
                    1.0 - 2.0 * (y * y + z * z),
                    2.0 * (x * y - z * w),
                    2.0 * (x * z + y * w),
                ],
                [
                    2.0 * (x * y + z * w),
                    1.0 - 2.0 * (x * x + z * z),
                    2.0 * (y * z - x * w),
                ],
                [
                    2.0 * (x * z - y * w),
                    2.0 * (y * z + x * w),
                    1.0 - 2.0 * (x * x + y * y),
                ],
            ],
            dtype=float,
        )

    def cbf_obstacle_callback(self, message: MarkerArray) -> None:
        """OctoMap近似球をMarkerの座標系ごとに保持する。"""

        grouped_centers = {}
        grouped_radii = {}

        for marker in message.markers:
            if marker.action != Marker.ADD:
                continue
            if marker.type != Marker.SPHERE_LIST:
                continue
            if not marker.points:
                continue

            frame_id = marker.header.frame_id.strip("/")
            if not frame_id:
                continue

            radius = 0.5 * max(
                abs(float(marker.scale.x)),
                abs(float(marker.scale.y)),
                abs(float(marker.scale.z)),
            )
            if not np.isfinite(radius) or radius <= 0.0:
                continue

            centers = np.array(
                [[point.x, point.y, point.z] for point in marker.points],
                dtype=float,
            )
            valid = np.all(np.isfinite(centers), axis=1)
            centers = centers[valid]
            if centers.size == 0:
                continue

            marker_rotation = self.quaternion_to_rotation_matrix(
                marker.pose.orientation
            )
            marker_translation = np.array(
                [
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z,
                ],
                dtype=float,
            )
            centers = centers @ marker_rotation.T + marker_translation

            grouped_centers.setdefault(frame_id, []).append(centers)
            grouped_radii.setdefault(frame_id, []).append(
                np.full(centers.shape[0], radius, dtype=float)
            )

        self.obstacle_groups = {
            frame_id: (
                np.concatenate(grouped_centers[frame_id], axis=0),
                np.concatenate(grouped_radii[frame_id], axis=0),
            )
            for frame_id in grouped_centers
        }
        self.received_obstacle_map = True

    def get_obstacles_in_kinematics_frame(
        self,
    ) -> tuple[np.ndarray, np.ndarray] | None:
        """全障害物球をTFで運動学座標系へ変換する。"""

        empty = (
            np.empty((0, 3), dtype=float),
            np.empty(0, dtype=float),
        )
        if not self.cbf_enabled:
            return empty

        if not self.received_obstacle_map:
            if self.cbf_require_obstacle_map:
                return None
            return empty

        if not self.obstacle_groups:
            return empty

        target_frame = self.kinematics_frame.strip("/")
        transformed_centers = []
        transformed_radii = []

        for source_frame, (centers, radii) in self.obstacle_groups.items():
            if source_frame == target_frame:
                centers_in_target = centers
            else:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        Time(),
                        timeout=RclpyDuration(seconds=0.01),
                    )
                except TransformException as error:
                    if not self.cbf_tf_warning_logged:
                        self.get_logger().warning(
                            "CBF障害物を運動学座標系へ変換できません。 "
                            f"{source_frame} -> {target_frame}: {error}"
                        )
                        self.cbf_tf_warning_logged = True
                    return None

                rotation = self.quaternion_to_rotation_matrix(
                    transform.transform.rotation
                )
                translation = np.array(
                    [
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z,
                    ],
                    dtype=float,
                )
                centers_in_target = centers @ rotation.T + translation

            transformed_centers.append(centers_in_target)
            transformed_radii.append(radii)

        self.cbf_tf_warning_logged = False
        return (
            np.concatenate(transformed_centers, axis=0),
            np.concatenate(transformed_radii, axis=0),
        )

    @staticmethod
    def build_cbf_constraint_data(
        arm_positions: np.ndarray,
        arm_jacobians: np.ndarray,
        arm_radii: np.ndarray,
        obstacle_centers: np.ndarray,
        obstacle_radii: np.ndarray,
        safety_margin: float,
        activation_distance: float,
        alpha: float,
        max_constraints: int,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
        """近距離にある球ペアの速度レベルCBF制約を作る。"""

        if obstacle_centers.shape[0] == 0:
            return (
                np.empty((0, 4), dtype=float),
                np.empty(0, dtype=float),
                np.empty(0, dtype=int),
                np.empty(0, dtype=int),
                float("inf"),
            )

        candidate_arm_indices = []
        candidate_obstacle_indices = []
        candidate_deltas = []
        candidate_h = []
        candidate_clearances = []
        minimum_clearance = float("inf")

        for arm_index in range(arm_positions.shape[0]):
            deltas = arm_positions[arm_index] - obstacle_centers
            distance_squared = np.einsum("ij,ij->i", deltas, deltas)
            distances = np.sqrt(np.maximum(distance_squared, 0.0))
            combined_radii = (
                arm_radii[arm_index]
                + obstacle_radii
                + safety_margin
            )
            clearances = distances - combined_radii
            minimum_clearance = min(
                minimum_clearance,
                float(np.min(clearances)),
            )

            active = np.flatnonzero(clearances <= activation_distance)
            if active.size == 0:
                continue

            candidate_arm_indices.append(
                np.full(active.size, arm_index, dtype=int)
            )
            candidate_obstacle_indices.append(active)
            candidate_deltas.append(deltas[active])
            candidate_h.append(
                distance_squared[active] - combined_radii[active] ** 2
            )
            candidate_clearances.append(clearances[active])

        if not candidate_arm_indices:
            return (
                np.empty((0, 4), dtype=float),
                np.empty(0, dtype=float),
                np.empty(0, dtype=int),
                np.empty(0, dtype=int),
                minimum_clearance,
            )

        arm_indices = np.concatenate(candidate_arm_indices)
        obstacle_indices = np.concatenate(candidate_obstacle_indices)
        deltas = np.concatenate(candidate_deltas, axis=0)
        h_values = np.concatenate(candidate_h)
        clearances = np.concatenate(candidate_clearances)

        if max_constraints > 0 and arm_indices.size > max_constraints:
            selected = np.argsort(clearances)[:max_constraints]
            arm_indices = arm_indices[selected]
            obstacle_indices = obstacle_indices[selected]
            deltas = deltas[selected]
            h_values = h_values[selected]

        # h_dot = 2(p_arm-p_obs)^T J_arm qdot
        cbf_matrix = 2.0 * np.einsum(
            "ij,ijk->ik",
            deltas,
            arm_jacobians[arm_indices],
        )
        cbf_lower_bound = -alpha * h_values

        return (
            cbf_matrix,
            cbf_lower_bound,
            arm_indices,
            obstacle_indices,
            minimum_clearance,
        )

    def publish_cbf_markers(
        self,
        q: np.ndarray,
        obstacle_centers: np.ndarray,
    ) -> None:
        """R1～R8の球と有効CBFペアをRVizへ配信する。"""

        stamp = self.get_clock().now().to_msg()
        arm_positions = self.kinematics.CBF_kinematics(q)
        active_arm_set = set(self.active_arm_indices.tolist())
        arm_markers = MarkerArray()

        for index, (position, radius) in enumerate(
            zip(arm_positions, self.arm_cbf_radii)
        ):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.kinematics_frame
            marker.ns = "arm_cbf_spheres"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(position[0])
            marker.pose.position.y = float(position[1])
            marker.pose.position.z = float(position[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.0 * float(radius)
            marker.scale.y = 2.0 * float(radius)
            marker.scale.z = 2.0 * float(radius)
            marker.color.r = 1.0 if index in active_arm_set else 0.1
            marker.color.g = 0.15 if index in active_arm_set else 0.45
            marker.color.b = 0.05 if index in active_arm_set else 1.0
            marker.color.a = 0.7
            arm_markers.markers.append(marker)

        self.arm_cbf_marker_pub.publish(arm_markers)

        active_lines = Marker()
        active_lines.header.stamp = stamp
        active_lines.header.frame_id = self.kinematics_frame
        active_lines.ns = "active_cbf_constraints"
        active_lines.id = 0
        active_lines.type = Marker.LINE_LIST
        active_lines.action = Marker.ADD
        active_lines.scale.x = 0.006
        active_lines.color.r = 1.0
        active_lines.color.g = 0.0
        active_lines.color.b = 0.0
        active_lines.color.a = 1.0

        for arm_index, obstacle_index in zip(
            self.active_arm_indices,
            self.active_obstacle_indices,
        ):
            if obstacle_index >= obstacle_centers.shape[0]:
                continue
            active_lines.points.append(
                Point(
                    x=float(arm_positions[arm_index, 0]),
                    y=float(arm_positions[arm_index, 1]),
                    z=float(arm_positions[arm_index, 2]),
                )
            )
            active_lines.points.append(
                Point(
                    x=float(obstacle_centers[obstacle_index, 0]),
                    y=float(obstacle_centers[obstacle_index, 1]),
                    z=float(obstacle_centers[obstacle_index, 2]),
                )
            )

        self.active_cbf_marker_pub.publish(
            MarkerArray(markers=[active_lines])
        )

    def get_target_in_kinematics_frame(
        self,
    ) -> np.ndarray | None:
        """
        受信したターゲットPoseStampedを、
        AmirKinematicsと同じ座標系へ変換する。
        """

        if self.target_pose is None:
            return None

        source_frame = (
            self.target_pose.header.frame_id
            .strip("/")
        )

        target_frame = (
            self.kinematics_frame.strip("/")
        )

        # すでに同じ座標系なら変換不要
        if source_frame == target_frame:
            transformed_pose = self.target_pose

        else:
            try:
                transformed_pose = (
                    self.tf_buffer.transform(
                        self.target_pose,
                        target_frame,
                        timeout=RclpyDuration(
                            seconds=0.05
                        ),
                    )
                )

                self.tf_warning_logged = False

            except TransformException as error:
                if not self.tf_warning_logged:
                    self.get_logger().warning(
                        "ターゲットを運動学座標系へ"
                        "変換できません。"
                        f" {source_frame} -> {target_frame}: "
                        f"{error}"
                    )

                    self.get_logger().warning(
                        "ros2 run tf2_ros tf2_echo "
                        f"{target_frame} {source_frame} "
                        "でTFを確認してください。"
                    )

                    self.tf_warning_logged = True

                return None

        target_position = np.array(
            [
                transformed_pose.pose.position.x,
                transformed_pose.pose.position.y,
                transformed_pose.pose.position.z,
            ],
            dtype=float,
        )

        if not np.all(
            np.isfinite(target_position)
        ):
            return None

        return target_position

    def solve_qp(
        self,
        q: np.ndarray,
        target: np.ndarray,
        obstacle_centers: np.ndarray,
        obstacle_radii: np.ndarray,
    ) -> tuple[np.ndarray, float]:
        """
        QPを解いて関節速度qdotを求める。

        Returns
        -------
        qdot : numpy.ndarray, shape (4,)
            関節速度
        error_norm : float
            手先位置誤差のノルム
        """

        self.nominal_qdot = np.zeros(4)
        self.cbf_constraint_matrix = np.empty((0, 4))
        self.cbf_constraint_lower_bound = np.empty(0)
        current_position = (
            self.kinematics.forward_kinematics(q)
        )

        jacobian = self.kinematics.jacobian(q)

        position_error = (
            target - current_position
        )

        error_norm = float(
            np.linalg.norm(position_error)
        )

        if error_norm <= self.position_tolerance:
            desired_velocity = np.zeros(3, dtype=float)
        else:
            desired_velocity = self.kp * position_error

        velocity_norm = float(
            np.linalg.norm(desired_velocity)
        )

        if (
            velocity_norm
            > self.max_cartesian_velocity
            and velocity_norm > 1.0e-9
        ):
            desired_velocity *= (
                self.max_cartesian_velocity
                / velocity_norm
            )

        qdot = cp.Variable(4)

        tracking_error = (
            jacobian @ qdot
            - desired_velocity
        )

        objective = cp.Minimize(
            0.5
            * cp.sum_squares(cp.multiply(np.sqrt(self.tracking_weights), tracking_error))
            + 0.5
            * self.regularization
            * cp.sum_squares(qdot)
        )

        constraints = [
            qdot <= self.qdot_max,
            qdot >= -self.qdot_max,

            q + self.dt * qdot <= self.q_max,
            q + self.dt * qdot >= self.q_min,
        ]
        if self.position_command_mode == "integrated":
            # Even a single QP step must fit within the position lead bound.
            constraints += [self.dt * qdot <= self.max_position_lead,
                            self.dt * qdot >= -self.max_position_lead]

        self.active_arm_indices = np.empty(0, dtype=int)
        self.active_obstacle_indices = np.empty(0, dtype=int)
        self.active_cbf_constraint_count = 0
        self.minimum_cbf_clearance = float("inf")

        if self.cbf_enabled:
            arm_positions = self.kinematics.CBF_kinematics(q)
            arm_jacobians = self.kinematics.CBF_jacobian(q)
            (
                cbf_matrix,
                cbf_lower_bound,
                self.active_arm_indices,
                self.active_obstacle_indices,
                self.minimum_cbf_clearance,
            ) = self.build_cbf_constraint_data(
                arm_positions,
                arm_jacobians,
                self.arm_cbf_radii,
                obstacle_centers,
                obstacle_radii,
                self.cbf_safety_margin,
                self.cbf_activation_distance,
                self.cbf_alpha,
                self.max_active_cbf_constraints,
            )

            self.active_cbf_constraint_count = cbf_matrix.shape[0]
            self.cbf_constraint_matrix = cbf_matrix
            self.cbf_constraint_lower_bound = cbf_lower_bound
            if self.active_cbf_constraint_count > 0:
                constraints.append(
                    cbf_matrix @ qdot >= cbf_lower_bound
                )

        problem = cp.Problem(
            objective,
            constraints,
        )

        try:
            problem.solve(
                solver=cp.OSQP,
                warm_start=True,
                verbose=False,
            )

        except cp.error.SolverError as error:
            self.get_logger().error(
                f"QP solver error: {error}"
            )

            return (
                np.zeros(4, dtype=float),
                error_norm,
            )

        valid_statuses = {
            cp.OPTIMAL,
            cp.OPTIMAL_INACCURATE,
        }

        if (
            problem.status not in valid_statuses
            or qdot.value is None
        ):
            self.get_logger().warning(
                f"QP failed: {problem.status}"
            )

            return (
                np.zeros(4, dtype=float),
                error_norm,
            )

        qdot_result = np.asarray(
            qdot.value,
            dtype=float,
        ).reshape(4)

        if not np.all(
            np.isfinite(qdot_result)
        ):
            self.get_logger().warning(
                "QP結果にNaNまたはInfが含まれています。"
            )

            return (
                np.zeros(4, dtype=float),
                error_norm,
            )

        self.nominal_qdot = qdot_result.copy()
        if self.joint_speed_mode == "fixed":
            selected = project_joint_speeds(qdot_result, self.joint_speeds, qdot, constraints)
            if selected is None:
                self.get_logger().warning(
                    "指定速度・QP方向・CBF/関節制限を同時に満たせないため保持します。",
                    throttle_duration_sec=2.0,
                )
                return np.zeros(4), error_norm
            qdot_result = selected

        return qdot_result, error_norm

    def publish_command(
        self,
        target_q: np.ndarray,
        qdot: np.ndarray = None,
    ) -> None:
        """関節位置指令をFloat64MultiArrayで配信する。"""

        if qdot is None:
            self.position_reference.reset()

        if self.joint5_hold_position is None:
            return

        message = Float64MultiArray()
        message.data = [
            float(target_q[0]),
            float(target_q[1]),
            float(target_q[2]),
            float(target_q[3]),
            float(self.joint5_hold_position),
        ]
        self.command_pub.publish(message)
        velocity = Float64MultiArray()
        velocity.data = [float(v) for v in (np.zeros(4) if qdot is None else qdot)] + [0.0]
        self.velocity_command_pub.publish(velocity)

    def publish_fk_pose(
        self,
        q: np.ndarray,
    ) -> None:
        """現在の関節角から求めたEE位置をPoseStampedで配信する。"""

        position = self.kinematics.forward_kinematics(q)

        if not np.all(np.isfinite(position)):
            self.get_logger().warning(
                "順運動学で計算したEE位置が不正です。"
            )
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.kinematics_frame
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.w = 1.0

        self.fk_pose_pub.publish(pose)

    def set_enabled_callback(self, request, response):
        self.qp_enabled = bool(request.data)
        # Both transitions require a fresh target before moving again.
        self.target_pose = None
        self.position_reference.reset()
        self.target_reached_cycle_count = 0
        self.controller_stopped = False
        self.timer.reset()
        self.publish_stop_state()
        response.success = True
        response.message = 'QP ON: waiting for a new target' if self.qp_enabled else 'QP OFF: continuous stop'
        return response

    def publish_stop_state(self):
        message = Bool()
        message.data = not self.qp_enabled
        self.stop_pub.publish(message)

    def control_callback(self) -> None:
        """
        制御周期ごとにQPを解いて関節位置指令を送る。
        """

        self.publish_stop_state()
        if not self.qp_enabled:
            self.position_reference.reset()
            if self.current_q is not None:
                self.publish_command(self.current_q.copy())
            return

        if self.current_q is None:
            return
        if self.controller_stopped:
            self.publish_command(self.current_q.copy())
            return

        current_q = self.current_q.copy()
        self.publish_fk_pose(current_q)

        obstacles = self.get_obstacles_in_kinematics_frame()
        if obstacles is None:
            self.active_arm_indices = np.empty(0, dtype=int)
            self.active_obstacle_indices = np.empty(0, dtype=int)
            self.active_cbf_constraint_count = 0
            self.publish_cbf_markers(
                current_q,
                np.empty((0, 3), dtype=float),
            )

            # 地図またはTFが必須のときは現在姿勢を保持する。
            if self.cbf_enabled and self.cbf_require_obstacle_map:
                self.publish_command(current_q)
                return

            obstacle_centers = np.empty((0, 3), dtype=float)
            obstacle_radii = np.empty(0, dtype=float)
        else:
            obstacle_centers, obstacle_radii = obstacles

        if self.target_pose is None:
            self.active_arm_indices = np.empty(0, dtype=int)
            self.active_obstacle_indices = np.empty(0, dtype=int)
            self.active_cbf_constraint_count = 0
            self.publish_cbf_markers(current_q, obstacle_centers)
            self.publish_command(current_q)
            return

        target_position = (
            self.get_target_in_kinematics_frame()
        )

        if target_position is None:
            self.publish_command(current_q)
            return

        qdot, error_norm = self.solve_qp(
            current_q,
            target_position,
            obstacle_centers,
            obstacle_radii,
        )

        self.publish_cbf_markers(current_q, obstacle_centers)

        if self.position_command_mode == "integrated":
            # Keep integrating with active CBFs too. Reject an old accumulated
            # endpoint only when it conflicts with the current local CBF model.
            target_q, command_qdot = self.position_reference.advance(
                current_q, qdot, self.dt, self.q_min, self.q_max,
            )
            if not cbf_command_compatible(
                current_q, target_q, command_qdot, self.dt,
                self.cbf_constraint_matrix, self.cbf_constraint_lower_bound, self.cbf_alpha,
            ):
                self.get_logger().warning(
                    "蓄積した位置目標が現在のCBF制約と矛盾するため、先行分を更新します。",
                    throttle_duration_sec=2.0,
                )
                self.position_reference.reset()
                target_q, command_qdot = self.position_reference.advance(
                    current_q, qdot, self.dt, self.q_min, self.q_max,
                )
                if not cbf_command_compatible(
                    current_q, target_q, command_qdot, self.dt,
                    self.cbf_constraint_matrix, self.cbf_constraint_lower_bound, self.cbf_alpha,
                ):
                    self.get_logger().warning(
                        "位置・速度指令のCBF整合を確認できないため保持します。",
                        throttle_duration_sec=2.0,
                    )
                    self.publish_command(current_q)
                    return
        else:
            self.position_reference.reset()
            target_q = np.clip(current_q + self.dt * qdot, self.q_min, self.q_max)
            command_qdot = (target_q - current_q) / self.dt

        if not np.all(np.isfinite(target_q)):
            self.get_logger().error(
                "計算した関節位置指令が不正です。"
            )
            self.publish_command(current_q)
            return

        self.publish_command(target_q, command_qdot)

        if error_norm <= self.position_tolerance:
            self.target_reached_cycle_count += 1
        else:
            self.target_reached_cycle_count = 0

        if (
            self.stop_after_reaching_target
            and self.target_reached_cycle_count >= self.target_hold_cycles
        ):
            # 最後に現在姿勢を送信し、position controllerに保持させる。
            self.publish_command(current_q)

            if self.enable_cbf_after_reaching_target:
                self.cbf_enabled = True
                self.stop_after_reaching_target = False
                self.target_reached_cycle_count = 0
                self.get_logger().info(
                    "赤い球へのCBFなし初期位置決めが完了しました。"
                    f" error={error_norm:.4f}m。CBF-QPを開始します。"
                )
                return

            self.controller_stopped = True
            self.timer.cancel()
            self.get_logger().info(
                "赤い球への初期位置決めが完了しました。"
                f" error={error_norm:.4f}m。QP制御を停止します。"
            )
            return

        # 約1秒おきに追従状態を表示
        if not hasattr(
            self,
            "_status_counter",
        ):
            self._status_counter = 0

        self._status_counter += 1

        if self._status_counter >= int(
            self.frequency
        ):
            self._status_counter = 0

            clearance_text = (
                "n/a"
                if not np.isfinite(self.minimum_cbf_clearance)
                else f"{self.minimum_cbf_clearance:.4f}m"
            )

            self.get_logger().info(
                f"target={np.round(target_position, 3)}, "
                f"error={error_norm:.4f} m, "
                f"error_xyz={np.round(target_position - self.kinematics.forward_kinematics(current_q), 4)} m, "
                f"qdot={np.round(qdot, 3)}, "
                f"nominal_qdot={np.round(self.nominal_qdot, 3)}, "
                f"sent_qdot={np.round(command_qdot, 3)}, "
                f"command_lead={np.round(target_q - current_q, 4)} rad, "
                f"cbf_active={self.active_cbf_constraint_count}, "
                f"cbf_min_clearance={clearance_text}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = QPTrackingController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            "QP tracking controller stopped."
        )

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
