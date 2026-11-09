"""Build an OctoMap on this PC from an external RealSense depth stream.

Only the depth image and its matching CameraInfo are received from the camera
machine. The point cloud is generated locally; RGB, YOLO, and the RealSense
pointcloud stream are not used.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    points_topic = LaunchConfiguration("points_topic")
    confirmed_points_topic = LaunchConfiguration("confirmed_points_topic")
    map_frame = LaunchConfiguration("map_frame")

    depth_to_points = Node(
        package="depth_image_proc",
        executable="point_cloud_xyz_node",
        name="amir_pc_depth_to_points",
        output="screen",
        remappings=[
            # On Humble, image_transport can resolve this subscription as
            # /camera_info even when the depth image itself is remapped.
            ("image_rect", depth_topic),
            ("/camera_info", camera_info_topic),
            ("points", points_topic),
        ],
        parameters=[{"use_sim_time": False}],
    )

    temporal_filter = Node(
        package="amir_driver",
        executable="temporal_pointcloud_filter.py",
        name="pc_temporal_pointcloud_filter",
        output="screen",
        parameters=[{
            "input_topic": points_topic,
            "output_topic": confirmed_points_topic,
            "map_frame": map_frame,
            "voxel_size": ParameterValue(
                LaunchConfiguration("resolution"), value_type=float
            ),
            "confirmation_time": ParameterValue(
                LaunchConfiguration("point_confirmation_time"), value_type=float
            ),
            "max_observation_gap": ParameterValue(
                LaunchConfiguration("point_max_gap"), value_type=float
            ),
            "use_sim_time": False,
        }],
    )

    octomap_server = Node(
        package="octomap_server",
        executable="octomap_server_node",
        namespace="pc_octomap",
        name="octomap_server",
        output="screen",
        remappings=[("cloud_in", confirmed_points_topic)],
        parameters=[
            {
                "frame_id": map_frame,
                "base_frame_id": map_frame,
                "resolution": ParameterValue(
                    LaunchConfiguration("resolution"), value_type=float
                ),
                "sensor_model.max_range": ParameterValue(
                    LaunchConfiguration("max_range"), value_type=float
                ),
                "filter_ground_plane": False,
                # Merge eight sibling voxels with identical occupancy values.
                "compress_map": True,
                "latch": True,
                "use_sim_time": False,
            }
        ],
    )

    cbf_sphere_generator = Node(
        package="amir_driver",
        executable="octomap_cbf_sphere_generator.py",
        name="pc_octomap_cbf_sphere_generator",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_cbf_spheres")),
        parameters=[
            {
                "input_topic": "/pc_octomap/occupied_cells_vis_array",
                "output_topic": "/cbf/obstacle_spheres",
                "sphere_padding": ParameterValue(
                    LaunchConfiguration("cbf_sphere_padding"), value_type=float
                ),
                "use_sim_time": False,
            }
        ],
    )

    cbf_snapshot_sender = Node(
        package="amir_driver",
        executable="cbf_octomap_snapshot.py",
        name="pc_cbf_snapshot_sender",
        output="screen",
        condition=IfCondition(LaunchConfiguration("send_cbf_snapshot")),
        parameters=[{
            "mode": "send",
            "input_topic": "/cbf/obstacle_spheres",
            "output_topic": LaunchConfiguration("snapshot_topic"),
            "capture_delay": ParameterValue(
                LaunchConfiguration("snapshot_delay"), value_type=float
            ),
            "use_sim_time": False,
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="depth_octomap_pc_rviz",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("amir_driver"), "rviz", "depth_octomap_pc.rviz"]
            ),
            "-f",
            map_frame,
        ],
        parameters=[{"use_sim_time": False}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "depth_topic", default_value="/camera/camera/depth/image_rect_raw"
            ),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/camera/depth/camera_info"
            ),
            DeclareLaunchArgument(
                "points_topic", default_value="/amir_pc/depth/points"
            ),
            DeclareLaunchArgument(
                "confirmed_points_topic",
                default_value="/amir_pc/depth/confirmed_points",
            ),
            DeclareLaunchArgument(
                "point_confirmation_time", default_value="1.0",
                description="Seconds a voxel must persist before mapping; 0 disables the wait",
            ),
            DeclareLaunchArgument(
                "point_max_gap", default_value="0.2",
                description="Observation gap in seconds that resets voxel confirmation",
            ),
            DeclareLaunchArgument(
                "map_frame", default_value="camera_depth_optical_frame"
            ),
            DeclareLaunchArgument("resolution", default_value="0.03"),
            DeclareLaunchArgument("max_range", default_value="1.5"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "send_cbf_snapshot", default_value="true",
                description="Send one frozen CBF map; requires use_cbf_spheres=true",
            ),
            DeclareLaunchArgument(
                "snapshot_delay", default_value="10.0",
                description="Capture delay after the first occupied OctoMap/CBF data, in seconds",
            ),
            DeclareLaunchArgument(
                "snapshot_topic", default_value="/cbf/octomap_snapshot"
            ),
            DeclareLaunchArgument(
                "use_cbf_spheres", default_value="true",
                description="Generate CBF obstacle spheres from occupied OctoMap voxels",
            ),
            DeclareLaunchArgument(
                "cbf_sphere_padding", default_value="0.0",
                description="Extra CBF obstacle sphere radius in meters",
            ),
            depth_to_points,
            temporal_filter,
            octomap_server,
            cbf_sphere_generator,
            cbf_snapshot_sender,
            rviz,
        ]
    )
