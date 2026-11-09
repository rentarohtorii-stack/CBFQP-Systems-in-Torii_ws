#!/usr/bin/env python3
"""Copy labelled raw AMIR images into reproducible YOLO train/validation splits."""

import argparse
from pathlib import Path
import random
import shutil


IMAGE_EXTENSIONS = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"}


def image_files(directory):
    if not directory.is_dir():
        return []
    return sorted(
        path for path in directory.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    )


def recursive_image_names(directory):
    """Return image basenames below a train/validation image directory."""
    if not directory.is_dir():
        raise ValueError(f"Exclude image directory is missing: {directory}")
    return {
        path.name
        for path in directory.rglob("*")
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    }


def ensure_empty(directory):
    if directory.exists() and any(directory.iterdir()):
        raise RuntimeError(
            f"Destination is not empty: {directory}. "
            "Choose a new --output directory; raw images and labels are untouched."
        )
    directory.mkdir(parents=True, exist_ok=True)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Copy matching YOLO image/label pairs into train and val directories."
    )
    dataset_root = Path("~/ros2_ws/datasets/amir_bottle").expanduser()
    parser.add_argument("--images", type=Path, default=dataset_root / "images" / "raw")
    parser.add_argument("--labels", type=Path, default=dataset_root / "labels" / "raw")
    parser.add_argument("--output", type=Path, default=dataset_root / "split")
    parser.add_argument(
        "--exclude-images-dir",
        type=Path,
        default=None,
        help="Exclude images whose names exist below this directory.",
    )
    parser.add_argument("--train-ratio", type=float, default=0.8)
    parser.add_argument("--seed", type=int, default=42)
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    images_dir = arguments.images.expanduser().resolve()
    labels_dir = arguments.labels.expanduser().resolve()
    output_dir = arguments.output.expanduser().resolve()
    if not 0.0 < arguments.train_ratio < 1.0:
        raise ValueError("--train-ratio must be between 0 and 1")

    images = image_files(images_dir)
    if arguments.exclude_images_dir is not None:
        excluded_names = recursive_image_names(
            arguments.exclude_images_dir.expanduser().resolve()
        )
        images = [image for image in images if image.name not in excluded_names]
        print(f"Excluded {len(excluded_names)} previously split image name(s).")
    if len(images) < 2:
        raise ValueError(f"At least two labelled images are needed; found {len(images)}")
    missing_labels = [
        image for image in images if not (labels_dir / f"{image.stem}.txt").is_file()
    ]
    if missing_labels:
        example = ", ".join(image.name for image in missing_labels[:3])
        raise ValueError(
            f"{len(missing_labels)} image(s) do not have a label file "
            f"in {labels_dir}; for example: {example}"
        )

    shuffled_images = images[:]
    random.Random(arguments.seed).shuffle(shuffled_images)
    train_count = round(len(shuffled_images) * arguments.train_ratio)
    train_count = max(1, min(len(shuffled_images) - 1, train_count))
    splits = {
        "train": shuffled_images[:train_count],
        "val": shuffled_images[train_count:],
    }

    destinations = [
        output_dir / kind / split
        for kind in ("images", "labels")
        for split in splits
    ]
    for destination in destinations:
        ensure_empty(destination)

    for split, split_images in splits.items():
        for image in split_images:
            label = labels_dir / f"{image.stem}.txt"
            shutil.copy2(image, output_dir / "images" / split / image.name)
            shutil.copy2(label, output_dir / "labels" / split / label.name)

    print(
        f"Copied {len(splits['train'])} train and {len(splits['val'])} validation "
        f"pairs to {output_dir}. Raw images and labels were not changed."
    )
    print(f"Train with: ros2 run amir_gazebo train_bottle_yolo.py --dataset {output_dir}")


if __name__ == "__main__":
    main()
