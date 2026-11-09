"""パラメータ化した単一ロボット bringup (マルチロボット対応の基盤)。

引数:
  namespace   ロボットの namespace (例 amir1)。空なら従来どおり単体・無 prefix。
  x, y, z, yaw  Gazebo へのスポーン位置・向き。
  world       world SDF ファイル名 (amir_gazebo/worlds/ 配下)。
  world_name  world SDF 内の <world name="..."> (pose_bridge 用)。
  pose_bridge  true で /world/<world_name>/pose/info を TF へブリッジ。
  launch_sim  true で gz_sim 本体 + /clock ブリッジ + EGL 環境 + 終了処理を起動。
              マルチロボットでは先頭 1 台だけ true、残りは false にする。

namespace を付けると:
  - robot_description は `xacro ... namespace:=<ns>` で生成 (Ignition 側の
    sensor topic / frame_id / gz_ros2_control namespace が prefix される)。
  - robot_state_publisher に frame_prefix=<ns>/ を与え URDF 由来の TF フレームを prefix。
  - controller spawner は /<ns>/controller_manager を対象にする。
  - bridge は /<ns>/scan, /<ns>/odom, /<ns>/d435/* の完全修飾名で張る。
  - TF はグローバル /tf に集約 (フレームが prefix 済みなので衝突しない)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _safe_kill_ignition(context, *args, **kwargs):
    os.system("pkill -9 -f 'ign gazebo' 2>/dev/null")
    os.system("pkill -9 -f 'ruby.*ign' 2>/dev/null")
    os.system("rm -rf /dev/shm/fastrtps_port* 2>/dev/null")
    return [LogInfo(msg="[amir_gazebo] Safe kill done")]


def launch_setup(context, *args, **kwargs):
    ns = LaunchConfiguration("namespace").perform(context)
    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)
    yaw = LaunchConfiguration("yaw").perform(context)
    world = LaunchConfiguration("world").perform(context)
    world_name = LaunchConfiguration("world_name").perform(context)
    pose_bridge = LaunchConfiguration("pose_bridge").perform(context).lower() in ("true", "1", "yes")
    launch_sim = LaunchConfiguration("launch_sim").perform(context).lower() in ("true", "1", "yes")

    # prefix 文字列: ns 空→"" , "amir1"→"amir1/"
    prefix = (ns + "/") if ns else ""
    # namespace 空のときは絶対名を無 prefix にして従来挙動を保つ
    cm = ("/" + ns + "/controller_manager") if ns else "/controller_manager"
    model_name = ns if ns else "amir_mecanum3"
    robot_desc_topic = ("/" + ns + "/robot_description") if ns else "/robot_description"

    amir_description_dir = get_package_share_directory("amir_description")
    amir_gazebo_dir = get_package_share_directory("amir_gazebo")
    world_file = os.path.join(amir_gazebo_dir, "worlds", world)
    arm_controllers_yaml = os.path.join(amir_gazebo_dir, "config", "arm_controllers.yaml")
    xacro_file = os.path.join(amir_description_dir, "urdf", "amir_mecanum3_sim.xacro")

    # xacro → robot_description (namespace を渡して Ignition 側を prefix)
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ", xacro_file, " namespace:=", ns,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_content,
            "use_sim_time": True,
        }],
        # TF はフレーム prefix せず、ロボットごとの /<ns>/tf に載せる
        # (絶対 /tf を相対 tf に remap → namespace 配下へ)。
        # ns="" なら /tf のまま (従来どおり単体)。
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=ns,
        output="screen",
        arguments=[
            "-topic", robot_desc_topic,
            "-name", model_name,
            "-x", x, "-y", y, "-z", z, "-Y", yaw,
            "-allow_renaming", "false",
        ],
    )

    # ── bridge (完全修飾名。Ignition transport は ROS namespace と別バス) ──
    scan_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        namespace=ns, name="scan_bridge", output="screen",
        arguments=[f"/{prefix}scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
    )

    d435_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        namespace=ns, name="d435_bridge", output="screen",
        arguments=[
            f"/{prefix}d435/image@sensor_msgs/msg/Image[gz.msgs.Image",
            f"/{prefix}d435/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            f"/{prefix}d435/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
    )

    # 真値オドメトリブリッジ。/<ns>/odom/tf をロボットごとの /<ns>/tf へ転送
    # (フレームは無 prefix の標準名 odom→base_footprint。tf topic が分離されるので
    #  複数ロボットでも衝突しない)
    odom_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        namespace=ns, name="odom_bridge", output="screen",
        arguments=[
            f"/{prefix}odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            f"/{prefix}odom/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        remappings=[(f"/{prefix}odom/tf", "tf")],
        parameters=[{"use_sim_time": True}],
    )

    # /<ns>/rover_twist → /<ns>/mecanum_drive_controller/reference_unstamped
    rover_twist_relay = Node(
        package="mecanumrover_description",
        executable="rover_twist_relay_ign.py",
        namespace=ns, name="rover_twist_relay_ign", output="log",
        parameters=[{"use_sim_time": True}],
    )

    # ── コントローラ (OnProcessExit で順番に起動、対象 CM を明示) ──
    def spawner(controller, extra=None):
        args = [controller, "--controller-manager", cm,
                "--controller-manager-timeout", "60", "--service-call-timeout", "60.0"]
        if extra:
            args[1:1] = extra
        return Node(package="controller_manager", executable="spawner",
                    namespace=ns, output="screen", arguments=args)

    jsb = spawner("joint_state_broadcaster")
    arm = spawner("arm_controller",
                  extra=["-t", "joint_trajectory_controller/JointTrajectoryController",
                         "-p", arm_controllers_yaml,
                         "--inactive"])

    forward_position = spawner(
        "forward_position_controller",
        extra=[
            "-t",
            "position_controllers/JointGroupPositionController",
            "-p",
            arm_controllers_yaml,
        ],
    )

    mecanum = spawner("mecanum_drive_controller")
    gripper = spawner("gripper_controller",
                      extra=["-t", "position_controllers/GripperActionController",
                             "-p", arm_controllers_yaml])

    # /gripper/open と /gripper/close を既存の GripperCommand action へ変換する。
    # namespace 付きの場合は /<namespace>/gripper/{open,close} になる。
    gripper_command_server = Node(
        package="amir_gazebo",
        executable="gripper_command_server.py",
        namespace=ns,
        name="gripper_command_server",
        output="screen",
        parameters=[{
            "action_name": "gripper_controller/gripper_cmd",
            "open_position": -1.0471,
            "closed_position": 0.261799,
            "max_effort": 0.4,
            "use_sim_time": True,
        }],
    )

    actions = [SetParameter(name="use_sim_time", value=True)]

    # ── sim 本体 (先頭ロボットのみ) ──
    if launch_sim:
        actions += [
            SetEnvironmentVariable("__EGL_VENDOR_LIBRARY_FILENAMES",
                                   "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"),
            SetEnvironmentVariable("__NV_PRIME_RENDER_OFFLOAD", "1"),
            SetEnvironmentVariable("__GLX_VENDOR_LIBRARY_NAME", "nvidia"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
                ),
                launch_arguments=[
                    ("gz_args", " -r -v 1 " + world_file),
                    ("gz_version", "6"),
                    ("on_exit_shutdown", "true"),
                ],
            ),
            Node(package="ros_gz_bridge", executable="parameter_bridge",
                 name="clock_bridge", output="screen",
                 arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]),
            RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=_safe_kill_ignition)])),
        ]

    if pose_bridge:
        actions.append(Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="pose_bridge", output="screen",
            arguments=[f"/world/{world_name}/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
        ))

    actions += [
        robot_state_publisher,
        spawn_robot,
        scan_bridge,
        d435_bridge,
        odom_bridge,
        rover_twist_relay,
        gripper_command_server,

        # スポーン完了 → jsb → (arm[inactive], mecanum)
        # → (forward_position[active], gripper)
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm, mecanum])),
        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=[forward_position, gripper])),
    ]

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.03"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("world", default_value="amir_world.sdf"),
        DeclareLaunchArgument("world_name", default_value="default"),
        DeclareLaunchArgument("pose_bridge", default_value="false"),
        DeclareLaunchArgument("launch_sim", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
