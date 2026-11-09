# Gazebo obstacle models

AMIRの障害物回避実験用モデルです。Gazebo Sim（Ignition Fortress）向けの
SDF 1.7形式で、4個の箱と500 mL程度のPETボトルを含みます。

## モデル

| モデル名 | 寸法（幅×奥行×高さ） |
|---|---|
| obstacle_box_1 | 0.40 × 0.25 × 0.16 m |
| obstacle_box_2 | 0.22 × 0.22 × 0.35 m |
| obstacle_box_3 | 0.50 × 0.12 × 0.12 m |
| obstacle_box_4 | 0.18 × 0.40 × 0.24 m |
| pet_bottle | 約 φ0.068 × 0.225 m |

## 配置方法

`models` 内の5フォルダを、ROS 2パッケージのモデルフォルダへコピーします。

```bash
cp -r models/* ~/ros2_humble_ws/src/humbleble_ws/amir740_ros/amir_gazebo/models/
```

サンプル配置は `worlds/obstacle_demo.sdf` に記載しています。各 `<pose>` は
`x y z roll pitch yaw` の順です。AMIRの既存worldへ `<include>` をコピーして
使う場合、床の高さに合わせてz座標を調整してください。

モデル検索パスが通っていない場合は、起動前に次を設定します。

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ros2_humble_ws/src/humbleble_ws/amir740_ros/amir_gazebo/models
```

## YOLO26によるRealSense画像認識

`gazebo_bringup.launch.py` は設定された待機時間後にYOLO26nを起動し、車体固定カメラの
`/amir_camera/image` を認識します。

| トピック | 内容 |
|---|---|
| `/yolo26/annotated_image` | 検出枠とラベルを描画した画像 |
| `/yolo26/detections` | クラス、信頼度、画像上の矩形を含むJSON |

YOLOだけを起動する場合:

```bash
ros2 launch amir_gazebo yolo26_realsense.launch.py
```

手先のD435画像を使う場合:

```bash
ros2 launch amir_gazebo yolo26_realsense.launch.py image_topic:=/d435/image
```

検出画像は次のコマンドで確認できます。

```bash
ros2 run rqt_image_view rqt_image_view /yolo26/annotated_image
```

### AMIRのPETボトルを追加学習する

同梱の `yolo26n.pt` はCOCOで学習済みの汎用モデルです。Gazeboの500 mL PETボトルを
安定して検出するには、AMIRカメラ画像で `bottle` クラスを追加学習します。クラス名を
`bottle` のままにするため、学習後もOctoMap用の点群フィルタ設定を変更する必要はありません。

まずGazeboを起動し、別ターミナルでRGB画像を収集します。カメラまたはアームを動かし、
ボトルの位置・向き・距離・背景を変えながら、少なくとも200枚（推奨500枚以上）を集めます。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch amir_gazebo capture_yolo_dataset.launch.py \
  output_dir:=~/ros2_ws/datasets/amir_bottle/images/raw \
  capture_fps:=1.0 max_images:=500
```

次に、追加インストール不要のラベル付けツールを起動します。マウス左ドラッグでボトルを囲み、
`s` で保存して次の画像へ進みます。ボトルが写っていない画像は `d` を押すと空ラベル（負例）として
保存されます。`u` は最後の矩形を取り消し、`r` は全矩形を消去、`q` は終了です。

```bash
ros2 run amir_gazebo label_bottle_images.py \
  --images ~/ros2_ws/datasets/amir_bottle/images/raw \
  --labels ~/ros2_ws/datasets/amir_bottle/labels/raw
```

PETボトルの矩形はすべて `bottle`（クラスID `0`）として保存されます。ラベル付け後に次を実行すると、
raw画像を残したまま、同名の画像・ラベルをランダムな学習:検証 = 8:2 へコピーします。

```bash
ros2 run amir_gazebo split_yolo_dataset.py \
  --images ~/ros2_ws/datasets/amir_bottle/images/raw \
  --labels ~/ros2_ws/datasets/amir_bottle/labels/raw \
  --output ~/ros2_ws/datasets/amir_bottle/split
```

出力の構成は次のとおりです。

```text
~/ros2_ws/datasets/amir_bottle/split/
├── images/train/  # 例: bottle_....jpg
├── images/val/
├── labels/train/  # 同名の bottle_....txt
└── labels/val/
```

CPU環境ではまず480 px、batch 4で学習します。GPUを使える場合は `--device 0 --batch 16` 等へ
変更できます。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run amir_gazebo train_bottle_yolo.py \
  --dataset ~/ros2_ws/datasets/amir_bottle/split \
  --epochs 100 --imgsz 480 --batch 4 --device cpu
```

終了時に表示される `runs/.../weights/best.pt` を使って起動します。

```bash
ros2 launch amir_gazebo gazebo_bringup.launch.py \
  yolo_model:=~/ros2_ws/datasets/amir_bottle/runs/bottle_finetune/weights/best.pt
```

`yolo26_realsense.launch.py` 単体でも同様に `model:=.../best.pt` を指定できます。

### BottleをOctoMapから除外

YOLO26nが `bottle` として検出した矩形領域をRealSense点群から除外し、
`/amir_camera/points_without_bottles` に配信します。`octomap_server` はこのフィルタ済み
点群から地図を生成します。

```bash
ros2 topic hz /amir_camera/points_without_bottles
ros2 topic echo /yolo26/detections
```
