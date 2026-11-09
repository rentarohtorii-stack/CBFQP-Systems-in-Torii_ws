# MR 用開発コピー

既存の RViz 手動操作版を残し、MR 入力の開発用に次のファイルを複製しています。

| 既存版 | MR 開発版 |
| --- | --- |
| `launch/amir_cbf_snapshot_qp.launch.py` | `launch/amir_cbf_snapshot_qp_mr.launch.py` |
| `scripts/rviz_ee_target.py` | `scripts/mr_ee_target.py` |
| `rviz/amir_cbf_snapshot.rviz` | `rviz/amir_cbf_snapshot_mr.rviz` |

## 現在の動作

MR 受信はまだ未実装です。MR 開発版も、現時点では RViz の赤い球をドラッグして
目標を送る既存版と同じ操作で動きます。初期位置は表示のみで、起動だけでは目標を送りません。
マーカーの namespace は `/mr_ee_target_marker`、Publish Point は `/mr/clicked_point`
に分けています。QP への出力は従来と同じ `/ee_target_pose` (`geometry_msgs/msg/PoseStamped`) です。
`use_rviz_target` など既存の launch 引数も利用できます。

QP 本体、速度設定、bringup、CBF スナップショット処理は共通です。
同じ実機を操作する既存版と MR 開発版を同時起動しないでください。
QP やコントローラへの指令先は共通です。

## ビルド・起動

```bash
cd /home/dars-note-12/Torii_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select amir_driver --symlink-install
source install/setup.bash
```

実機 bringup とコントローラ切り替えは `cbf_snapshot_qp.md` の手順と同じです。
その後、用途に応じてどちらか一方を起動します。

```bash
# 既存の RViz 手動操作版
ros2 launch amir_driver amir_cbf_snapshot_qp.launch.py

# MR 開発版（現在は RViz 手動操作）
ros2 launch amir_driver amir_cbf_snapshot_qp_mr.launch.py
```

## 今後の編集箇所

- `mr_ee_target.py`: MR 目標の購読、MR 座標系から `frame_id` への変換を追加します。
  同じ変換済み目標を `server.setPose("ee_target", pose)` と `server.applyChanges()`
  で赤い球へ反映し、`publish_target()` から QP に送る想定です。
  MR のメッセージ型・トピック・軸・単位・原点の仕様は未確定です。
- `amir_cbf_snapshot_qp_mr.launch.py`: MR 接続用ノードや入力パラメータを追加します。
- `amir_cbf_snapshot_mr.rviz`: MR 用表示を変更します。

MR 操作と RViz 手動操作を MR 版の中で併用する場合は、目標入力の優先順位も決めてください。
既存版のファイルを編集する必要はありません。
