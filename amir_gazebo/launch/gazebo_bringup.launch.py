"""AMIR、Gazebo、QP追従ターゲット、QP制御をまとめて起動するlaunchファイル。"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # amir_gazeboパッケージのshareディレクトリ
    amir_gazebo_share = FindPackageShare("amir_gazebo")
    # 110枚のbottleデータセットで追加学習した最良重み。
    # 標準モデルも models/yolo/yolo26n.pt に残している。
    default_yolo_model = (
        "/home/dars-note-12/ros2_ws/datasets/amir_bottle/split_110/"
        "runs/bottle_all110_finetune/weights/best.pt"
    )
    yolo_model_argument = DeclareLaunchArgument(
        "yolo_model",
        default_value=default_yolo_model,
        description="YOLO model (.pt) used for bottle detection",
    )

    # 赤い球のSDFファイル
    target_sdf_path = PathJoinSubstitution(
        [
            amir_gazebo_share,
            "models",
            "ee_target",
            "model.sdf",
        ]
    )

    tripod_camera_sdf_path = PathJoinSubstitution(
        [
            amir_gazebo_share,
            "models",
            "realsense_tripod",
            "model.sdf",
        ]
    )

    # ---------------------------------------------------------
    # 1. AMIR本体・Gazebo・ros2_controlを起動
    # ---------------------------------------------------------
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    amir_gazebo_share,
                    "launch",
                    "robot_bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "namespace": "",
            "world": "amir_world.sdf",
            "world_name": "default",
            "launch_sim": "true",
        }.items(),
    )

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

    # ---------------------------------------------------------
    # 2. Gazebo Sim内に赤い球を生成
    # ---------------------------------------------------------
    spawn_ee_target = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_ee_target",
        output="screen",
        arguments=[
            "-world",
            "default",
            "-name",
            "ee_target",
            "-file",
            target_sdf_path,
            "-x",
            "0.2",
            "-y",
            "0.0",
            "-z",
            "0.65",
        ],
    )

    # 順運動学で計算したEE位置を示す青い球
    fk_visual_sdf_path = PathJoinSubstitution(
        [
            amir_gazebo_share,
            "models",
            "viz_sphere",
            "model.sdf",
        ]
    )

    spawn_fk_visual = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_fk_ee_visual",
        output="screen",
        arguments=[
            "-world", "default",
            "-name", "viz_sphere",
            "-file", fk_visual_sdf_path,
            "-x", "0.0",
            "-y", "0.0",
            "-z", "-10.0",
            "-allow_renaming", "false",
        ],
    )

    # 障害物モデルを obstacle_demo.sdf と同じ位置に生成
    obstacle_poses = [
        ("obstacle_box_1", "0.40", "-0.4", "0.08", "0.20"),
        ("obstacle_box_2", "0.55", "0.05", "0.125", "0.0"),
        ("obstacle_box_3", "0.35", "0.18", "0.06", "-0.4"),
        ("obstacle_box_4", "0.15", "0.50", "0.12", "0.30"),
        # obstacle_box_2上面(z=0.28 m)の直上へ置き、落下による横倒しを防ぐ。
        ("pet_bottle", "0.48", "0.08", "0.282", "0.0"),
    ]
    spawn_obstacles = []
    for model_name, x, y, z, yaw in obstacle_poses:
        model_sdf_path = PathJoinSubstitution(
            [amir_gazebo_share, "models", model_name, "model.sdf"]
        )
        spawn_obstacles.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                name=f"spawn_{model_name}",
                output="screen",
                arguments=[
                    "-world", "default",
                    "-name", model_name,
                    "-file", model_sdf_path,
                    "-x", x,
                    "-y", y,
                    "-z", z,
                    "-Y", yaw,
                    "-allow_renaming", "false",
                ],
            )
        )

    # ---------------------------------------------------------
    # 3. Gazeboのモデル姿勢をROS 2へbridge
    # ---------------------------------------------------------
    target_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ee_target_pose_bridge",
        output="screen",
        arguments=[
            (
                "/world/default/pose/info"
                "@tf2_msgs/msg/TFMessage"
                "[gz.msgs.Pose_V"
            )
        ],
    )

    # Gazeboのset_pose serviceをROS 2 serviceとして公開
    set_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="set_pose_bridge",
        output="screen",
        arguments=[
            (
                "/world/default/set_pose"
                "@ros_gz_interfaces/srv/SetEntityPose"
            )
        ],
    )

    # ---------------------------------------------------------
    # 4. Gazeboモデル姿勢からee_targetだけ抽出して配信
    # ---------------------------------------------------------
    target_pose_publisher = Node(
        package="amir_gazebo",
        executable="target_pose_publisher.py",
        name="target_pose_publisher",
        output="screen",
        parameters=[
            {
                "target_name": "ee_target",
                "gazebo_pose_topic": "/world/default/pose/info",
                "output_topic": "/ee_target_pose",
                # RVizの赤いInteractive Markerをドラッグしている間も
                # 連続して受け取り、QPとGazeboの赤球へ同時に反映する。
                "command_topic": "/ee_target_pose_command",
                "set_pose_service": "/world/default/set_pose",
                "command_override_duration": 0.2,
                # 赤球の移動中は即時、停止中も最新位置を50 Hzで
                # QPコントローラへ送信し続ける。
                "publish_rate": 50.0,
                "use_sim_time": True,
            }
        ],
    )

    # Gazebo GUIでモデルをドラッグすると、姿勢がマウスを離した時だけ
    # 確定する場合がある。RVizのInteractive MarkerならPOSE_UPDATEを
    # ドラッグ中も配信できるため、アームが移動中の赤球を追従できる。
    rviz_target = Node(
        package="amir_driver",
        executable="rviz_ee_target.py",
        name="gazebo_rviz_ee_target",
        output="screen",
        parameters=[
            {
                "frame_id": "world",
                "target_topic": "/ee_target_pose_command",
                "clicked_point_topic": "/clicked_point",
                "interactive_marker_namespace": "ee_target_marker",
                "initial_x": 0.2,
                "initial_y": 0.0,
                "initial_z": 0.65,
                "marker_scale": 0.16,
                "publish_while_dragging": True,
                "drag_update_rate": 30.0,
                "use_sim_time": True,
            }
        ],
    )

    arm_visual_publisher = Node(
        package="amir_gazebo",
        executable="arm_visual_publisher.py",
        name="arm_visual_publisher",
        output="screen",
        parameters=[
            {
                "fk_pose_topic": "/fk_ee_pose",
                "world_frame": "world",
                "model_name": "viz_sphere",
                "set_pose_service": "/world/default/set_pose",
                "update_rate": 30.0,
                "use_sim_time": True,
            }
        ],
    )

    # ---------------------------------------------------------
    # 5. worldとロボットのTFツリーを接続
    #
    # Gazebo内でAMIRを
    # x=0.0, y=0.0, z=0.03
    # に固定して使用する場合の静的TF
    # ---------------------------------------------------------
    world_to_base_tf = Node(
       package="tf2_ros",
       executable="static_transform_publisher",
       name="world_to_0dom_tf",
       output="screen",
       parameters=[
           {
               "use_sim_time": True,
           }
       ],
       arguments=[
           "--x", "0.0",
           "--y", "0.0",
           "--z", "0.0",
           "--roll", "0.0",
           "--pitch", "0.0",
           "--yaw", "0.0",
           "--frame-id", "world",
           "--child-frame-id", "odom",
        ],
    )



    # ---------------------------------------------------------
    # 6. QP追従制御ノード
    # ---------------------------------------------------------
    qp_tracking_controller = Node(
        package="amir_qp_control",
        executable="qp_tracking_controller.py",
        name="qp_tracking_controller",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,

                # 入出力トピック
                "joint_state_topic": "/joint_states",
                "target_topic": "/ee_target_pose",
                "fk_pose_topic": "/fk_ee_pose",

                # QPの運動学座標系
                "kinematics_frame": "link0_1",

                # 使用する関節
                "joint_names": [
                    "Joint_1",
                    "Joint_2",
                    "Joint_3",
                    "Joint_4",
                ],

                # 制御周期
                "control_frequency": 50.0,

                # 追従ゲイン
                "kp": 2.0,

                # QP正則化係数
                "regularization": 1.0e-3,

                # 初期試験用の速度制限
                "max_cartesian_velocity": 10.0,
                "max_joint_velocity": 10.0,

                # 目標到達判定
                "position_tolerance": 0.005,
                "stop_after_reaching_target": True,
                "target_hold_time": 0.2,
                "enable_cbf_after_reaching_target": True,

                # 起動時の初期位置決めではCBF制約を使用しない
                "cbf_enabled": False,
                "cbf_obstacle_topic": "/cbf/obstacle_spheres",
                "arm_cbf_marker_topic": "/cbf/arm_spheres",
                "active_cbf_marker_topic": "/cbf/active_constraints",
                "arm_cbf_radii": [0.057] * 8,
                "cbf_safety_margin": 0.03,
                "cbf_activation_distance": 0.08,
                "cbf_alpha": 5.0,
                "max_active_cbf_constraints": 128,
                "cbf_require_obstacle_map": True,
            }
        ],
    )
    # ---------------------------------------------------------
    # 7. YOLOで検出したボトルの深度点だけを除外し、残りをOctoMapへ渡す。
    semantic_pointcloud_filter = Node(
        package="amir_gazebo",
        executable="semantic_pointcloud_filter.py",
        name="semantic_pointcloud_filter",
        output="screen",
        parameters=[
            {
                "cloud_topic": "/amir_camera/points",
                "detections_topic": "/yolo26/detections",
                "output_topic": "/amir_camera/points_without_bottles",
                "excluded_classes": ["bottle"],
                "minimum_confidence": 0.35,
                # YOLO推論完了まで、撮影済みの点群を保持する時間。
                "cloud_buffer_duration": 3.0,
                # RGB画像と点群が同時刻とみなせる最大差。
                "maximum_cloud_stamp_difference": 0.5,
                "maximum_buffered_clouds": 60,
                "bbox_padding": 3,
                "depth_band": 0.12,
                # ros_gz_bridgeのPointCloud2 publisherはRELIABLE。
                "input_reliability": "reliable",
                "use_sim_time": True,
            }
        ],
    )

    octomap_server = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        remappings=[
            ("cloud_in", "/amir_camera/points_without_bottles"),
        ],
        parameters=[
            {
                "frame_id": "world",
                "base_frame_id": "base_footprint",
                "resolution": 0.03,
                "sensor_model.max_range": 3.0,
                "filter_ground_plane": False,
                "filter_speckles": True,
                "latch": True,
                "use_sim_time": True,
            }
        ],
    )

    # OctoMapの占有ボクセルをCBF用の近似球へ変換する。
    cbf_sphere_generator = Node(
        package="amir_gazebo",
        executable="octomap_cbf_sphere_generator.py",
        name="octomap_cbf_sphere_generator",
        output="screen",
        parameters=[
            {
                "input_topic": "/occupied_cells_vis_array",
                "output_topic": "/cbf/obstacle_spheres",
                "sphere_padding": 0.0,
                "use_sim_time": True,
            }
        ],
    )
    """
    frame_id: world
OctoMapを作成する基準座標系です。

base_frame_id: base_footprint
ロボット基準座標系です。主に地面除去などで使われます。

resolution: 0.03
OctoMapの1ボクセルを3 cmにします。

sensor_model.max_range: 3.0
カメラから3 m以内の点群を障害物として登録します。

filter_ground_plane: False
地面もOctoMapへ登録します。

filter_speckles: True
周囲から孤立したノイズボクセルを除去します。

latch: True
RVizが後から起動しても、最新のOctoMapを受け取れるようにします。

use_sim_time: True
Gazeboの /clock を使用します。
    """
    octomap_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="octomap_rviz",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [amir_gazebo_share, "rviz", "octomap.rviz"]
            ),
        ],
        parameters=[{"use_sim_time": True}],
    )

    # RViz を遅く起動し過ぎると、YOLO の処理待ち後に出力される
    # 点群の撮影時刻よりも RViz の動的 TF キャッシュ開始時刻の方が
    # 新しくなり、"earlier than all the data in the transform cache" で
    # 破棄される。ロボット / TF の起動だけ待ち、早めに起動する。
    delayed_octomap_rviz = TimerAction(
        period=5.0,
        actions=[octomap_rviz],
    )

    # 起動2秒後、CBFなしQPで赤球への初期位置決めを開始する。
    # 目標到達後は同じcontrollerがCBF-QPへ切り替わる。
    delayed_qp_tracking_controller = TimerAction(
        period=2.0,
        actions=[
            qp_tracking_controller,
        ],
    )

    # YOLOのbottle検出矩形を深度点群フィルタへ渡す。
    delayed_yolo26_detector = TimerAction(
        period=25.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            amir_gazebo_share,
                            "launch",
                            "yolo26_realsense.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "model": LaunchConfiguration("yolo_model"),
                }.items(),
            )
        ],
    )

    return LaunchDescription(
        [
            # Gazebo側の全ノードを専用ROSドメインで起動する。
            SetEnvironmentVariable("ROS_DOMAIN_ID", "50"),
            yolo_model_argument,
            robot_bringup,
            spawn_ee_target,
            spawn_fk_visual,
            *spawn_obstacles,
            target_pose_bridge,
            set_pose_bridge,
            target_pose_publisher,
            rviz_target,
            arm_visual_publisher,
            world_to_base_tf,
            delayed_qp_tracking_controller,
            delayed_yolo26_detector,
            spawn_tripod_camera,
            dual_camera_bridge,
            semantic_pointcloud_filter,
            octomap_server,
            cbf_sphere_generator,
            delayed_octomap_rviz,
        ]
    )
