"""既存の統合launchへコピーする2台のRGB-Dカメラ設定。"""

from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


amir_gazebo_share = FindPackageShare("amir_gazebo")

tripod_camera_sdf_path = PathJoinSubstitution(
    [amir_gazebo_share, "models", "realsense_tripod", "model.sdf"]
)

# Gazebo起動後に三脚カメラを生成。光軸はモデル+x方向。
spawn_tripod_camera = TimerAction(
    period=3.0,
    actions=[
        Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_tripod_camera",
            output="screen",
            arguments=[
                "-world", "default",
                "-name", "realsense_tripod",
                "-file", tripod_camera_sdf_path,
                "-x", "1.50",
                "-y", "0.00",
                "-z", "0.00",
                "-Y", "3.14159",
                "-allow_renaming", "false",
            ],
        )
    ],
)

# Gazebo TransportのRGB・深度・CameraInfo・点群をROS 2へ変換。
dual_camera_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    name="dual_realsense_bridge",
    output="screen",
    parameters=[
        {
            "config_file": PathJoinSubstitution(
                [amir_gazebo_share, "config", "dual_realsense_bridge.yaml"]
            ),
            "use_sim_time": True,
        }
    ],
)

# 既存launchのLaunchDescription配列へ以下の2つを追加:
#     spawn_tripod_camera,
#     dual_camera_bridge,
