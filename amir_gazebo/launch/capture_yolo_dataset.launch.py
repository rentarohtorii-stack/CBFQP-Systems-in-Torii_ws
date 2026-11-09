"""Capture AMIR RGB frames for custom YOLO bottle fine-tuning."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic", default_value="/amir_camera/image"
            ),
            DeclareLaunchArgument(
                "output_dir",
                default_value="~/ros2_ws/datasets/amir_bottle/images/raw",
            ),
            DeclareLaunchArgument("capture_fps", default_value="1.0"),
            DeclareLaunchArgument("max_images", default_value="300"),
            Node(
                package="amir_gazebo",
                executable="capture_yolo_dataset.py",
                name="yolo_dataset_capture",
                output="screen",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "output_dir": LaunchConfiguration("output_dir"),
                        "capture_fps": ParameterValue(
                            LaunchConfiguration("capture_fps"), value_type=float
                        ),
                        "max_images": ParameterValue(
                            LaunchConfiguration("max_images"), value_type=int
                        ),
                        "use_sim_time": True,
                    }
                ],
            ),
        ]
    )
