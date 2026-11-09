#!/usr/bin/env python3
"""Reconstruct a YOLO-detected bottle from synchronized RealSense RGB-D.

The node uses the official gCasp segmentation checkpoint and the bottle DualSDF
primitive decoder.  It converts the reconstructed primitive surfaces to a
PointCloud2 so octomap_server can integrate the completed bottle geometry.
"""

import json
import math
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class GcaspBottleReconstruction(Node):
    """Estimate a YOLO-detected bottle shape from a RealSense RGB-D frame."""

    def __init__(self):
        super().__init__("gcasp_bottle_reconstruction")

        self.declare_parameter("gcasp_root", "")
        self.declare_parameter("segmentation_checkpoint", "")
        self.declare_parameter("bottle_checkpoint", "")
        self.declare_parameter("device", "auto")
        self.declare_parameter("rgb_topic", "/amir_camera/image")
        self.declare_parameter("depth_topic", "/amir_camera/depth_image")
        self.declare_parameter("camera_info_topic", "/amir_camera/camera_info")
        self.declare_parameter("detections_topic", "/yolo26/detections")
        self.declare_parameter("output_topic", "/gcasp/bottle_points")
        self.declare_parameter("marker_topic", "/gcasp/bottle_primitives")
        self.declare_parameter("minimum_confidence", 0.35)
        self.declare_parameter("max_inference_rate", 0.2)
        self.declare_parameter("maximum_frame_age", 0.15)
        self.declare_parameter("minimum_depth", 0.15)
        self.declare_parameter("maximum_depth", 3.0)
        self.declare_parameter("minimum_points", 200)
        self.declare_parameter("sample_points", 1024)
        self.declare_parameter("min_points_per_primitive", 4)
        self.declare_parameter("max_primitives", 48)
        self.declare_parameter("optimization_steps", 30)
        self.declare_parameter("surface_samples_per_primitive", 24)
        self.declare_parameter("minimum_primitive_radius", 0.004)
        self.declare_parameter("maximum_primitive_radius", 0.08)
        self.declare_parameter("maximum_fit_rmse", 0.12)

        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_camera_info = None
        self.latest_header = None
        self.last_inference_time = 0.0
        self.random = np.random.default_rng(42)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            Image,
            str(self.get_parameter("rgb_topic").value),
            self.rgb_callback,
            sensor_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self.depth_callback,
            sensor_qos,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self.camera_info_callback,
            sensor_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("detections_topic").value),
            self.detections_callback,
            10,
        )
        self.points_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("output_topic").value),
            sensor_qos,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("marker_topic").value),
            latched_qos,
        )

        self.load_models()

    def load_models(self):
        gcasp_root = Path(str(self.get_parameter("gcasp_root").value)).expanduser()
        segmentation_checkpoint = Path(
            str(self.get_parameter("segmentation_checkpoint").value)
        ).expanduser()
        bottle_checkpoint = Path(
            str(self.get_parameter("bottle_checkpoint").value)
        ).expanduser()

        if not gcasp_root.is_dir():
            raise RuntimeError(f"gCasp root does not exist: {gcasp_root}")
        if not segmentation_checkpoint.is_file():
            raise RuntimeError(
                f"gCasp segmentation checkpoint does not exist: "
                f"{segmentation_checkpoint}"
            )
        if not bottle_checkpoint.is_file():
            raise RuntimeError(
                f"gCasp bottle checkpoint does not exist: {bottle_checkpoint}"
            )

        sys.path.insert(0, str(gcasp_root))
        sys.path.insert(0, str(gcasp_root / "models" / "gcn3d"))
        try:
            import torch
            from models.deepsdf_mlp9 import Decoder
            from models.gcn3d.gcn3d import get_model
        except ImportError as error:
            raise RuntimeError(
                "gCasp dependencies are unavailable. "
                "Install the Python packages required by gCasp."
            ) from error

        requested_device = str(self.get_parameter("device").value).lower()
        if requested_device == "auto":
            requested_device = "cuda" if torch.cuda.is_available() else "cpu"
        if requested_device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("gCasp device=cuda was requested, but CUDA is unavailable")

        self.torch = torch
        self.device = torch.device(requested_device)
        self.gcn_model = get_model(
            SimpleNamespace(num_classes=257, support_num=1, neighbor_num=50)
        ).to(self.device)
        segmentation_data = torch.load(
            segmentation_checkpoint,
            map_location=self.device,
        )
        segmentation_state = {
            key.removeprefix("ae."): value
            for key, value in segmentation_data["trainer_state_dict"].items()
            if key.startswith("ae.")
        }
        self.gcn_model.load_state_dict(segmentation_state, strict=True)
        self.gcn_model.eval()

        attr_config = SimpleNamespace(
            dropout=False,
            dropout_prob=None,
            use_tanh=False,
            in_ch=128,
            out_ch=1024,
            hidden_ch=512,
        )
        self.primitive_decoder = Decoder(attr_config).to(self.device)
        bottle_data = torch.load(bottle_checkpoint, map_location=self.device)
        primitive_state = {
            key.removeprefix("prim_attr_net."): value
            for key, value in bottle_data["trainer_state_dict"].items()
            if key.startswith("prim_attr_net.")
        }
        self.primitive_decoder.load_state_dict(primitive_state, strict=True)
        self.primitive_decoder.eval()

        self.get_logger().info(
            "gCasp bottle reconstruction is ready "
            f"(device={self.device.type}, model={segmentation_checkpoint.name})"
        )
        self.get_logger().info(
            "Waiting for a YOLO bottle detection and synchronized RealSense RGB-D"
        )

    def rgb_callback(self, message):
        self.latest_rgb = message

    def camera_info_callback(self, message):
        self.latest_camera_info = message

    @staticmethod
    def stamp_to_nanoseconds(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def depth_callback(self, message):
        self.latest_depth = message

    def detections_callback(self, message):
        try:
            result = json.loads(message.data)
            bottles = self.select_bottle_detections(result)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(f"Invalid YOLO detection message: {error}")
            return

        if not bottles:
            return
        if (
            self.latest_rgb is None
            or self.latest_depth is None
            or self.latest_camera_info is None
        ):
            return

        max_age_ns = int(float(self.get_parameter("maximum_frame_age").value) * 1e9)
        detection_stamp = result.get("stamp", {})
        detection_stamp_ns = (
            int(detection_stamp.get("sec", 0)) * 1_000_000_000
            + int(detection_stamp.get("nanosec", 0))
        )
        if any(
            abs(self.stamp_to_nanoseconds(header.stamp) - detection_stamp_ns)
            > max_age_ns
            for header in (
                self.latest_rgb.header,
                self.latest_depth.header,
                self.latest_camera_info.header,
            )
        ):
            return

        now = time.monotonic()
        maximum_rate = float(self.get_parameter("max_inference_rate").value)
        if maximum_rate > 0.0 and now - self.last_inference_time < 1.0 / maximum_rate:
            return
        self.last_inference_time = now
        try:
            self.latest_header = self.latest_depth.header
            reconstructed_centers = []
            reconstructed_radii = []
            reconstructed_surfaces = []
            reconstruction_errors = []
            for bottle in bottles:
                points = self.extract_bottle_points(
                    bottle["bbox"],
                    int(result.get("image_width", 0)),
                    int(result.get("image_height", 0)),
                )
                if points is None:
                    continue
                centers, radii, rmse = self.reconstruct_primitives(points)
                if centers is None:
                    continue
                reconstructed_centers.append(centers)
                reconstructed_radii.append(radii)
                reconstruction_errors.append(rmse)
                reconstructed_surfaces.append(
                    self.sample_sphere_surfaces(centers, radii)
                )

            if not reconstructed_surfaces:
                self.get_logger().info(
                    "No detected bottle produced a valid gCasp reconstruction"
                )
                return

            centers = np.concatenate(reconstructed_centers, axis=0)
            radii = np.concatenate(reconstructed_radii, axis=0)
            surface_points = np.concatenate(reconstructed_surfaces, axis=0)
            mean_rmse = float(np.mean(reconstruction_errors))
            cloud = point_cloud2.create_cloud_xyz32(
                self.latest_depth.header,
                surface_points.tolist(),
            )
            self.points_publisher.publish(cloud)
            self.publish_markers(centers, radii, mean_rmse)
            self.get_logger().info(
                "gCasp bottle geometry published: "
                f"bottles={len(reconstructed_surfaces)}, "
                f"primitives={len(centers)}, points={len(surface_points)}, "
                f"mean_fit_rmse={mean_rmse:.3f} m"
            )
        except Exception as error:
            self.get_logger().error(f"gCasp bottle reconstruction failed: {error}")

    def select_bottle_detections(self, result):
        confidence_threshold = float(
            self.get_parameter("minimum_confidence").value
        )
        bottles = [
            detection
            for detection in result.get("detections", [])
            if str(detection.get("class_name", "")).lower() == "bottle"
            and float(detection.get("confidence", 0.0)) >= confidence_threshold
        ]
        return sorted(
            bottles,
            key=lambda detection: float(detection["confidence"]),
            reverse=True,
        )

    def extract_bottle_points(self, bbox, detection_width, detection_height):
        """Back-project only the YOLO bottle ROI in the synchronized depth frame."""

        depth = self.bridge.imgmsg_to_cv2(
            self.latest_depth,
            desired_encoding="passthrough",
        )
        depth = np.asarray(depth)
        if depth.ndim != 2:
            self.get_logger().warning("Depth image must have one channel")
            return None
        if np.issubdtype(depth.dtype, np.integer):
            depth = depth.astype(np.float32) * 0.001
        else:
            depth = depth.astype(np.float32)

        height, width = depth.shape
        scale_x = width / detection_width if detection_width > 0 else 1.0
        scale_y = height / detection_height if detection_height > 0 else 1.0
        x_min = max(0, int(math.floor(float(bbox["x_min"]) * scale_x)))
        x_max = min(width, int(math.ceil(float(bbox["x_max"]) * scale_x)))
        y_min = max(0, int(math.floor(float(bbox["y_min"]) * scale_y)))
        y_max = min(height, int(math.ceil(float(bbox["y_max"]) * scale_y)))
        if x_max <= x_min or y_max <= y_min:
            return None

        crop = depth[y_min:y_max, x_min:x_max]
        minimum_depth = float(self.get_parameter("minimum_depth").value)
        maximum_depth = float(self.get_parameter("maximum_depth").value)
        valid = np.isfinite(crop) & (crop >= minimum_depth) & (crop <= maximum_depth)
        if not np.any(valid):
            return None
        median_depth = float(np.median(crop[valid]))
        valid &= np.abs(crop - median_depth) <= 0.12
        rows, cols = np.nonzero(valid)
        if len(rows) < int(self.get_parameter("minimum_points").value):
            return None

        z = crop[rows, cols]
        camera = self.latest_camera_info
        fx, fy, cx, cy = camera.k[0], camera.k[4], camera.k[2], camera.k[5]
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warning("CameraInfo has invalid focal lengths")
            return None
        points = np.column_stack(
            (
                ((cols + x_min).astype(np.float64) - cx) * z / fx,
                ((rows + y_min).astype(np.float64) - cy) * z / fy,
                z,
            )
        )
        sample_count = int(self.get_parameter("sample_points").value)
        return points[
            self.random.choice(
                len(points),
                sample_count,
                replace=len(points) < sample_count,
            )
        ]

    def reconstruct_primitives(self, points_m):
        torch = self.torch
        points_mm = points_m * 1000.0
        input_points = torch.from_numpy(points_mm.astype(np.float32)).to(self.device)
        input_points = input_points.unsqueeze(0)
        bottle_onehot = torch.tensor(
            [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
            device=self.device,
        )
        with torch.inference_mode():
            prediction = self.gcn_model(input_points, bottle_onehot)
            labels = torch.argmax(prediction[0], dim=1).cpu().numpy()

        dust_label = 256
        minimum_group_size = int(
            self.get_parameter("min_points_per_primitive").value
        )
        selected_labels = []
        for label in np.unique(labels):
            if label == dust_label:
                continue
            if np.count_nonzero(labels == label) >= minimum_group_size:
                selected_labels.append(int(label))
        if len(selected_labels) < 4:
            return None, None, None

        selected_labels.sort(
            key=lambda label: np.count_nonzero(labels == label),
            reverse=True,
        )
        selected_labels = selected_labels[
            : int(self.get_parameter("max_primitives").value)
        ]
        selected_mask = np.isin(labels, selected_labels)
        labels = labels[selected_mask]
        points_mm = points_mm[selected_mask]
        if len(points_mm) < int(self.get_parameter("minimum_points").value):
            return None, None, None

        centers_by_label = np.array(
            [np.mean(points_mm[labels == label], axis=0) for label in selected_labels],
            dtype=np.float32,
        )
        latent = self.optimize_shape(np.asarray(selected_labels), centers_by_label)
        if latent is None:
            return None, None, None

        with torch.inference_mode():
            attributes = self.primitive_decoder(latent).view(-1, 4).cpu().numpy()

        source_centers = attributes[labels, 1:4]
        scale, rotation, translation = self.umeyama(source_centers, points_mm)
        transformed_points_mm = (
            scale * source_centers @ rotation.T + translation
        )
        rmse_m = float(
            np.sqrt(np.mean(np.sum((transformed_points_mm - points_mm) ** 2, axis=1)))
            / 1000.0
        )
        if rmse_m > float(self.get_parameter("maximum_fit_rmse").value):
            self.get_logger().warning(
                f"Rejected gCasp shape with fit RMSE {rmse_m:.3f} m"
            )
            return None, None, None

        primitive_indices = np.asarray(selected_labels, dtype=np.int64)
        centers_mm = (
            scale * attributes[primitive_indices, 1:4] @ rotation.T + translation
        )
        radii_m = (
            abs(scale) * np.exp(attributes[primitive_indices, 0]) / 1000.0
        )
        radii_m = np.clip(
            radii_m,
            float(self.get_parameter("minimum_primitive_radius").value),
            float(self.get_parameter("maximum_primitive_radius").value),
        )
        return centers_mm / 1000.0, radii_m, rmse_m

    def optimize_shape(self, labels, centers):
        torch = self.torch
        label_tensor = torch.from_numpy(labels).to(self.device, dtype=torch.long)
        center_tensor = torch.from_numpy(centers).to(self.device)
        pair_indices = self.make_pair_indices(len(labels))
        if len(pair_indices) == 0:
            return None
        pair_tensor = torch.from_numpy(pair_indices).to(self.device, dtype=torch.long)

        def invariant_feature(values):
            vectors = values[pair_tensor[:, 1]] - values[pair_tensor[:, 0]]
            normalized = vectors / torch.clamp(
                torch.linalg.norm(vectors, dim=1, keepdim=True), min=1.0e-8
            )
            return normalized @ normalized.T

        target_feature = invariant_feature(center_tensor).detach()
        latent = torch.zeros((1, 128), device=self.device, requires_grad=True)
        optimizer = torch.optim.Adam([latent], lr=0.3)
        steps = int(self.get_parameter("optimization_steps").value)
        for _ in range(steps):
            attributes = self.primitive_decoder(latent).view(-1, 4)
            predicted_centers = attributes[label_tensor, 1:4]
            loss = torch.mean((invariant_feature(predicted_centers) - target_feature) ** 2)
            loss += 1.0e-4 * torch.mean(latent**2)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
        return latent.detach()

    @staticmethod
    def make_pair_indices(count):
        pairs = [(first, second) for first in range(count) for second in range(count)]
        return np.asarray(
            [pair for pair in pairs if pair[0] != pair[1]],
            dtype=np.int64,
        )

    @staticmethod
    def umeyama(source, target):
        """Return the similarity transform that maps source row vectors to target."""

        source_mean = np.mean(source, axis=0)
        target_mean = np.mean(target, axis=0)
        source_centered = source - source_mean
        target_centered = target - target_mean
        covariance = target_centered.T @ source_centered / len(source)
        left, singular_values, right_t = np.linalg.svd(covariance)
        correction = np.eye(3)
        if np.linalg.det(left) * np.linalg.det(right_t) < 0.0:
            correction[-1, -1] = -1.0
        rotation = left @ correction @ right_t
        source_variance = np.mean(np.sum(source_centered**2, axis=1))
        if source_variance <= 1.0e-12:
            raise ValueError("Degenerate gCasp primitive geometry")
        scale = float(np.sum(singular_values * np.diag(correction)) / source_variance)
        translation = target_mean - scale * (rotation @ source_mean)
        return scale, rotation, translation

    def sample_sphere_surfaces(self, centers, radii):
        samples_per_sphere = int(
            self.get_parameter("surface_samples_per_primitive").value
        )
        indices = np.arange(samples_per_sphere, dtype=np.float64) + 0.5
        phi = np.arccos(1.0 - 2.0 * indices / samples_per_sphere)
        theta = math.pi * (1.0 + math.sqrt(5.0)) * indices
        unit_sphere = np.column_stack(
            (
                np.cos(theta) * np.sin(phi),
                np.sin(theta) * np.sin(phi),
                np.cos(phi),
            )
        )
        return np.concatenate(
            [center + radius * unit_sphere for center, radius in zip(centers, radii)],
            axis=0,
        ).astype(np.float32)

    def publish_markers(self, centers, radii, rmse):
        output = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        output.markers.append(clear)
        for marker_id, (center, radius) in enumerate(zip(centers, radii)):
            marker = Marker()
            marker.header = self.latest_header
            marker.ns = "gcasp_bottle_primitives"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.0 * float(radius)
            marker.scale.y = 2.0 * float(radius)
            marker.scale.z = 2.0 * float(radius)
            marker.color.r = 0.05
            marker.color.g = 0.85
            marker.color.b = 0.25
            marker.color.a = 0.7
            marker.text = f"gCasp RMSE={rmse:.3f} m"
            output.markers.append(marker)
        self.marker_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = GcaspBottleReconstruction()
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
