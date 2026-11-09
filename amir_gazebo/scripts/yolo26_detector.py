#!/usr/bin/env python3
"""Run Ultralytics YOLO26 detection on a ROS 2 image topic."""

import json
import os
import time
from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from ultralytics import YOLO


class Yolo26Detector(Node):
    def __init__(self):
        super().__init__("yolo26_detector")

        self.declare_parameter("model_path", "yolo26n.pt")
        self.declare_parameter("image_topic", "/amir_camera/image")
        self.declare_parameter("annotated_topic", "/yolo26/annotated_image")
        self.declare_parameter("detections_topic", "/yolo26/detections")
        self.declare_parameter("bottle_rgb_topic", "")
        self.declare_parameter("bottle_annotated_topic", "")
        self.declare_parameter("snapshot_complete_topic", "")
        self.declare_parameter("confidence", 0.35)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("max_fps", 5.0)
        self.declare_parameter("input_watchdog_seconds", 15.0)

        # launch 引数に ``~/...`` を渡しても ROS 2 launch はシェルの
        # ようにはチルダを展開しない。そのまま Ultralytics に渡すと
        # モデル読込時に例外でノードが終了し、後段の点群フィルタも
        # 入力を受け取れなくなるため、ここで絶対パスへ正規化する。
        configured_model_path = self.get_parameter("model_path").value
        self.model_path = str(
            Path(os.path.expandvars(configured_model_path)).expanduser().resolve()
        )
        self.image_topic = self.get_parameter("image_topic").value
        self.annotated_topic = self.get_parameter("annotated_topic").value
        self.detections_topic = self.get_parameter("detections_topic").value
        self.bottle_rgb_topic = self.get_parameter("bottle_rgb_topic").value
        self.bottle_annotated_topic = self.get_parameter("bottle_annotated_topic").value
        self.snapshot_complete_topic = self.get_parameter("snapshot_complete_topic").value
        self.stop_requested = False
        self.confidence = float(self.get_parameter("confidence").value)
        self.iou = float(self.get_parameter("iou").value)
        self.image_size = int(self.get_parameter("image_size").value)
        self.device = self.get_parameter("device").value
        max_fps = float(self.get_parameter("max_fps").value)
        self.minimum_period = 1.0 / max_fps if max_fps > 0.0 else 0.0
        self.last_inference_time = 0.0
        self.input_watchdog_seconds = float(
            self.get_parameter("input_watchdog_seconds").value
        )
        if self.input_watchdog_seconds <= 0.0:
            raise ValueError("input_watchdog_seconds must be greater than zero")
        self.last_image_received_time = time.monotonic()

        if not Path(self.model_path).is_file():
            raise FileNotFoundError(
                f"YOLO model file was not found: {self.model_path} "
                f"(configured as: {configured_model_path})"
            )

        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()

        self.camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, self.camera_qos
        )
        self.annotated_publisher = self.create_publisher(
            Image, self.annotated_topic, 1
        )
        self.detections_publisher = self.create_publisher(
            String, self.detections_topic, 10
        )
        self.bottle_rgb_publisher = (
            self.create_publisher(Image, self.bottle_rgb_topic, 1)
            if self.bottle_rgb_topic else None
        )
        self.bottle_annotated_publisher = (
            self.create_publisher(Image, self.bottle_annotated_topic, 1)
            if self.bottle_annotated_topic else None
        )
        if self.snapshot_complete_topic:
            complete_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                Bool, self.snapshot_complete_topic,
                self.snapshot_complete_callback, complete_qos,
            )

        self.get_logger().info(
            f"YOLO26 ready: {self.image_topic} -> "
            f"{self.annotated_topic}, {self.detections_topic} "
            f"(device={self.device}, max_fps={max_fps:g})"
        )

    def image_callback(self, message):
        now = time.monotonic()
        self.last_image_received_time = now
        if now - self.last_inference_time < self.minimum_period:
            return
        self.last_inference_time = now

        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            result = self.model.predict(
                source=image,
                conf=self.confidence,
                iou=self.iou,
                imgsz=self.image_size,
                device=self.device,
                verbose=False,
            )[0]

            detections = []
            if result.boxes is not None:
                boxes = result.boxes.xyxy.cpu().tolist()
                confidences = result.boxes.conf.cpu().tolist()
                class_ids = result.boxes.cls.int().cpu().tolist()
                for xyxy, confidence, class_id in zip(
                    boxes, confidences, class_ids
                ):
                    x1, y1, x2, y2 = xyxy
                    detections.append(
                        {
                            "class_id": class_id,
                            "class_name": result.names[class_id],
                            "confidence": round(float(confidence), 5),
                            "bbox": {
                                "x_min": round(float(x1), 2),
                                "y_min": round(float(y1), 2),
                                "x_max": round(float(x2), 2),
                                "y_max": round(float(y2), 2),
                                "center_x": round(float((x1 + x2) / 2.0), 2),
                                "center_y": round(float((y1 + y2) / 2.0), 2),
                                "width": round(float(x2 - x1), 2),
                                "height": round(float(y2 - y1), 2),
                            },
                        }
                    )

            detection_message = String()
            detection_message.data = json.dumps(
                {
                    "stamp": {
                        "sec": message.header.stamp.sec,
                        "nanosec": message.header.stamp.nanosec,
                    },
                    "frame_id": message.header.frame_id,
                    "model": self.model_path,
                    "image_width": int(message.width),
                    "image_height": int(message.height),
                    "count": len(detections),
                    "detections": detections,
                },
                ensure_ascii=False,
            )
            self.detections_publisher.publish(detection_message)

            annotated_message = self.bridge.cv2_to_imgmsg(
                result.plot(), encoding="bgr8"
            )
            annotated_message.header = message.header
            self.annotated_publisher.publish(annotated_message)
            if any(
                str(detection["class_name"]).casefold() == "bottle"
                for detection in detections
            ):
                if self.bottle_rgb_publisher is not None:
                    self.bottle_rgb_publisher.publish(message)
                if self.bottle_annotated_publisher is not None:
                    self.bottle_annotated_publisher.publish(annotated_message)
        except Exception as exception:  # Keep processing subsequent frames.
            self.get_logger().error(f"YOLO inference failed: {exception}")

    def refresh_image_subscription(self):
        self.get_logger().warning(
            "No RGB image received for "
            f"{self.input_watchdog_seconds:g}s; recreating the image subscriber"
        )
        self.destroy_subscription(self.image_subscription)
        self.image_subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, self.camera_qos
        )
        self.last_image_received_time = time.monotonic()

    def snapshot_complete_callback(self, message):
        if message.data:
            self.get_logger().info("OctoMap snapshot complete; stopping YOLO")
            self.stop_requested = True


def main(args=None):
    rclpy.init(args=args)
    node = Yolo26Detector()
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.5)
            if not node.stop_requested and (
                time.monotonic() - node.last_image_received_time
                > node.input_watchdog_seconds
            ):
                node.refresh_image_subscription()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
