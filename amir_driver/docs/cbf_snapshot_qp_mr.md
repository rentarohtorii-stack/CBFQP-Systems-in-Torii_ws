# MR hand target -> RViz / Gazebo / QP

## 入力と座標

`Meta-p-main/Assets/Scripts/ROS/handPosePublisher.cs` は MRTK の手関節を
Origin 基準の位置（m）へ変換し、一度だけ Unity2Ros で ROS FLU に変換します。
既定は左手 MiddleProximal、30 Hz、`/palm_pose`、frame `Unity` です。
実際の値は Unity Inspector の設定を使用します。

ROS の `mr_ee_target.py` は PoseStamped を受信し、最新 TF で表示フレームへ変換します。
MR の時刻と Gazebo のシミュレーション時刻は異なるため、入力時刻は TF 検索に使わず、
出力に ROS 時刻を付けます。MR Origin は静的に校正する前提です。
空フレーム、NaN/Inf、TF 不足の目標は捨てます。
赤い Interactive Marker と出力目標は同じ変換済み位置です。姿勢は制御しません。

経路:
`Unity -> rosbridge -> /palm_pose -> mr_ee_target.py -> /ee_target_pose_command
-> target_pose_publisher.py -> /ee_target_pose -> QP`
`target_pose_publisher.py` は Gazebo の赤球も同時に動かします。
MR モードでは RViz のドラッグと Publish Point を無効にし、入力の競合を避けます。
手を見失うと Unity は送信を止め、Gazebo 側は最後の目標を保持します。
自動で現在 EE 位置に停止する処理は追加していません。

## 準備

```bash
sudo apt-get install ros-humble-rosbridge-server
cd /home/dars-note-12/Torii_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select amir_driver amir_gazebo --symlink-install
source install/setup.bash
```

Unity の main シーンの handPosePublisher で Origin を必ず割り当ててください。
保存済みシーンにはまだ Origin がありません。未設定ではエラーを出し送信しません。
Origin の +Z は前、+X は右、+Y は上。Transform の見た目の拡大縮小は距離に掛けません。
Topic=`/palm_pose`、FrameId=`Unity` にし、RosConnector を同じ GameObject に付けます。
RosConnector の接続先は `ws://<ROS PC の IP>:9090`、ROS 2 対応設定にします。
PC と MR 端末のネットワークで TCP 9090 が通る必要があります。

## 起動

```bash
# bringup2 は既定で MR モード
ros2 launch amir_gazebo gazebo_bringup2.launch.py
# 通常の bringup は従来の RViz モードが既定。MR を明示しても起動可能
ros2 launch amir_gazebo gazebo_bringup.launch.py target_source:=mr
# bringup2 で従来の RViz 手動操作へ戻す
ros2 launch amir_gazebo gazebo_bringup2.launch.py target_source:=rviz
```

上記は同時起動しないでください。Gazebo が起動したら Unity/MR アプリを実行します。
rosbridge も launch 内で ROS_DOMAIN_ID=50 で起動します。
別起動の rosbridge を使う場合は `start_rosbridge:=false` とし、そのプロセスも
`ROS_DOMAIN_ID=50` にします。

既定の校正 TF は `world -> Unity` の位置・回転ゼロです。つまり MR Origin と Gazebo
world の原点・方向を一致させる必要があります。異なる場合は ROS FLU で
`mr_origin_x/y/z` (m)、`mr_origin_roll/pitch/yaw` (rad) を指定してください。
`mr_frame` は Unity の FrameId に合わせます。動的な追跡基準の TF には未対応です。

```bash
ROS_DOMAIN_ID=50 ros2 topic echo /palm_pose
ROS_DOMAIN_ID=50 ros2 topic echo /ee_target_pose_command
```

赤球の初期位置への移動や CBF 初期化は従来の Gazebo QP の動作を引き継ぎます。
Gazebo 自体の動作、MR 端末からの通信、原点校正は実環境で確認してください。

## 実機のスナップショット MR launch

`amir_cbf_snapshot_qp_mr.launch.py` も同じ受信ノードを使用しますが、
rosbridge と MR Origin の校正 TF は自動起動しません。
その環境に合わせて別途用意してください。元の `rviz_ee_target.py` と
`amir_cbf_snapshot_qp.launch.py` は変更していません。
