"""Launch YOLO26 object detection for a RealSense RGB image topic."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_model = PathJoinSubstitution(
        [FindPackageShare("amir_gazebo"), "models", "yolo", "yolo26n.pt"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value=default_model),
            DeclareLaunchArgument("image_topic", default_value="/amir_camera/image"),
            DeclareLaunchArgument(
                "annotated_topic", default_value="/yolo26/annotated_image"
            ),
            DeclareLaunchArgument(
                "detections_topic", default_value="/yolo26/detections"
            ),
            DeclareLaunchArgument("confidence", default_value="0.35"),
            DeclareLaunchArgument("iou", default_value="0.45"),
            DeclareLaunchArgument("image_size", default_value="640"),
            DeclareLaunchArgument("device", default_value="cpu"),
            DeclareLaunchArgument("max_fps", default_value="5.0"),
            Node(
                package="amir_gazebo",
                executable="yolo26_detector.py",
                name="yolo26_detector",
                output="screen",
                parameters=[
                    {
                        "model_path": LaunchConfiguration("model"),
                        "image_topic": LaunchConfiguration("image_topic"),
                        "annotated_topic": LaunchConfiguration("annotated_topic"),
                        "detections_topic": LaunchConfiguration("detections_topic"),
                        "confidence": ParameterValue(
                            LaunchConfiguration("confidence"), value_type=float
                        ),
                        "iou": ParameterValue(
                            LaunchConfiguration("iou"), value_type=float
                        ),
                        "image_size": ParameterValue(
                            LaunchConfiguration("image_size"), value_type=int
                        ),
                        "device": LaunchConfiguration("device"),
                        "max_fps": ParameterValue(
                            LaunchConfiguration("max_fps"), value_type=float
                        ),
                        "use_sim_time": True,
                    }
                ],
            ),
        ]
    )
