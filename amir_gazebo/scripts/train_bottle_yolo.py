#!/usr/bin/env python3
"""Fine-tune YOLO26n for AMIR's simulated or physical PET bottles."""

import argparse
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO


IMAGE_EXTENSIONS = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"}


def image_files(directory):
    if not directory.is_dir():
        return []
    return sorted(
        path for path in directory.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    )


def validate_split(dataset_root, split):
    images_dir = dataset_root / "images" / split
    labels_dir = dataset_root / "labels" / split
    images = image_files(images_dir)
    if not images:
        raise ValueError(f"No images found in {images_dir}")
    if not labels_dir.is_dir():
        raise ValueError(f"Label directory is missing: {labels_dir}")

    missing_labels = [
        image for image in images if not (labels_dir / f"{image.stem}.txt").is_file()
    ]
    if missing_labels:
        example = ", ".join(path.name for path in missing_labels[:3])
        raise ValueError(
            f"{len(missing_labels)} image(s) in {split} have no YOLO label file "
            f"(for example: {example})"
        )
    return len(images)


def create_dataset_yaml(dataset_root):
    data = {
        "path": str(dataset_root),
        "train": "images/train",
        "val": "images/val",
        "names": {0: "bottle"},
    }
    yaml_path = dataset_root / "bottle_dataset.yaml"
    with yaml_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
    return yaml_path


def parse_arguments():
    package_share = Path(get_package_share_directory("amir_gazebo"))
    parser = argparse.ArgumentParser(
        description="Fine-tune YOLO26n using a one-class AMIR bottle dataset."
    )
    parser.add_argument(
        "--dataset",
        type=Path,
        default=Path("~/ros2_ws/datasets/amir_bottle").expanduser(),
        help="Dataset root containing images/{train,val} and labels/{train,val}.",
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=package_share / "models" / "yolo" / "yolo26n.pt",
        help="Pretrained .pt model used as the initial weight.",
    )
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=480)
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--patience", type=int, default=25)
    parser.add_argument(
        "--project",
        type=Path,
        default=None,
        help="Directory for training runs (default: <dataset>/runs).",
    )
    parser.add_argument("--name", default="bottle_finetune")
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    dataset_root = arguments.dataset.expanduser().resolve()
    model_path = arguments.model.expanduser().resolve()
    if not model_path.is_file():
        raise FileNotFoundError(f"Initial model was not found: {model_path}")

    train_count = validate_split(dataset_root, "train")
    validation_count = validate_split(dataset_root, "val")
    data_yaml = create_dataset_yaml(dataset_root)
    project = (arguments.project or dataset_root / "runs").expanduser().resolve()

    print(
        f"Training bottle model with {train_count} train and {validation_count} "
        f"validation images."
    )
    model = YOLO(str(model_path))
    model.train(
        data=str(data_yaml),
        epochs=arguments.epochs,
        imgsz=arguments.imgsz,
        batch=arguments.batch,
        workers=arguments.workers,
        device=arguments.device,
        patience=arguments.patience,
        project=str(project),
        name=arguments.name,
    )
    print(
        "Training complete. Use the printed run directory's weights/best.pt "
        "as yolo_model:=... when launching Gazebo."
    )


if __name__ == "__main__":
    main()
