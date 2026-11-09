"""Start YOLO and OctoMap after an external RealSense publishes RGB and points."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    cached_image_topic = LaunchConfiguration("cached_image_topic")
    cached_pointcloud_topic = LaunchConfiguration("cached_pointcloud_topic")
    retain_frames = LaunchConfiguration("retain_frames")
    active_image_topic = IfElseSubstitution(
        retain_frames, if_value=cached_image_topic, else_value=image_topic
    )
    active_pointcloud_topic = IfElseSubstitution(
        retain_frames, if_value=cached_pointcloud_topic, else_value=pointcloud_topic
    )
    octomap_frame = LaunchConfiguration("octomap_frame")

    frame_cache = Node(
        package="amir_driver",
        executable="realsense_frame_cache.py",
        name="realsense_frame_cache",
        output="screen",
        condition=IfCondition(retain_frames),
        parameters=[
            {
                "image_topic": image_topic,
                "pointcloud_topic": pointcloud_topic,
                "cached_image_topic": cached_image_topic,
                "cached_pointcloud_topic": cached_pointcloud_topic,
                "use_sim_time": False,
            }
        ],
    )

    # A camera process can exist before its streams actually publish.
    camera_ready = Node(
        package="amir_driver",
        executable="wait_for_realsense.py",
        name="wait_for_realsense",
        output="screen",
        parameters=[
            {
                "image_topic": active_image_topic,
                "pointcloud_topic": active_pointcloud_topic,
                "use_sim_time": False,
            }
        ],
    )

    yolo_detector = Node(
        package="amir_gazebo",
        executable="yolo26_detector.py",
        name="yolo26_detector",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_yolo")),
        parameters=[
            {
                "model_path": LaunchConfiguration("yolo_model"),
                "image_topic": active_image_topic,
                "annotated_topic": "/yolo26/annotated_image",
                "detections_topic": "/yolo26/detections",
                "confidence": 0.35,
                "max_fps": 5.0,
                "use_sim_time": False,
            }
        ],
    )

    # With no robot TF, keep the map in the RealSense point cloud frame.
    octomap_server = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_octomap")),
        remappings=[("cloud_in", active_pointcloud_topic)],
        parameters=[
            {
                "frame_id": octomap_frame,
                "base_frame_id": octomap_frame,
                "resolution": 0.03,
                "sensor_model.max_range": 3.0,
                "filter_ground_plane": False,
                "filter_speckles": True,
                "latch": True,
                "use_sim_time": False,
            }
        ],
    )

    cbf_sphere_generator = Node(
        package="amir_driver",
        executable="octomap_cbf_sphere_generator.py",
        name="octomap_cbf_sphere_generator",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_octomap")),
        parameters=[
            {
                "input_topic": "/occupied_cells_vis_array",
                "output_topic": "/cbf/obstacle_spheres",
                "sphere_padding": 0.0,
                "use_sim_time": False,
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="realsense_octomap_rviz",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("amir_driver"), "rviz", "realsense_octomap.rviz"]
            ),
            "-f",
            octomap_frame,
        ],
        remappings=[
            ("/amir/realsense/depth/points_cached", active_pointcloud_topic)
        ],
        parameters=[{"use_sim_time": False}],
    )

    def start_perception_when_ready(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg="RealSense readiness check stopped; "
                    "YOLO, OctoMap and RViz were not started"
                )
            ]
        return [
            LogInfo(msg="RealSense RGB and point cloud ready; starting perception"),
            octomap_server,
            cbf_sphere_generator,
            yolo_detector,
            rviz,
        ]

    # Register first so already-running cameras cannot race the exit event.
    start_perception_after_camera = RegisterEventHandler(
        OnProcessExit(target_action=camera_ready, on_exit=start_perception_when_ready)
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_yolo", default_value="true"),
            DeclareLaunchArgument("use_octomap", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "retain_frames",
                default_value="true",
                description=(
                    "true: keep and relay the latest RGB image and point cloud; "
                    "false: use live RealSense topics without retaining frames"
                ),
            ),
            DeclareLaunchArgument(
                "yolo_model",
                default_value=(
                    "/home/dars-note-12/ros2_ws/datasets/amir_bottle/split_110/"
                    "runs/bottle_all110_finetune/weights/best.pt"
                ),
            ),
            DeclareLaunchArgument(
                "image_topic", default_value="/camera/camera/color/image_raw"
            ),
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="/camera/camera/depth/color/points",
            ),
            DeclareLaunchArgument(
                "cached_image_topic",
                default_value="/amir/realsense/color/image_cached",
            ),
            DeclareLaunchArgument(
                "cached_pointcloud_topic",
                default_value="/amir/realsense/depth/points_cached",
            ),
            DeclareLaunchArgument(
                "octomap_frame",
                default_value="camera_depth_optical_frame",
            ),
            start_perception_after_camera,
            frame_cache,
            camera_ready,
        ]
    )
