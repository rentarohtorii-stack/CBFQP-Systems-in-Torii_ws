#!/usr/bin/env python3
"""Label PET-bottle bounding boxes in AMIR images and save YOLO text labels."""

import argparse
from pathlib import Path

import cv2


IMAGE_EXTENSIONS = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"}


def image_files(directory):
    return sorted(
        path for path in directory.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    )


class BottleImageLabeler:
    """Minimal mouse-driven YOLO box annotator for one `bottle` class."""

    def __init__(self, images_dir, labels_dir):
        self.images = image_files(images_dir)
        if not self.images:
            raise ValueError(f"No images found in {images_dir}")
        self.labels_dir = labels_dir
        self.labels_dir.mkdir(parents=True, exist_ok=True)
        self.window_name = "AMIR bottle labeller"
        self.index = 0
        self.image = None
        self.boxes = []
        self.drawing = False
        self.start_point = None
        self.current_point = None

        for index, image_path in enumerate(self.images):
            if not (self.labels_dir / f"{image_path.stem}.txt").exists():
                self.index = index
                break
        else:
            self.index = 0

    @property
    def image_path(self):
        return self.images[self.index]

    @property
    def label_path(self):
        return self.labels_dir / f"{self.image_path.stem}.txt"

    def load_current_image(self):
        self.image = cv2.imread(str(self.image_path), cv2.IMREAD_COLOR)
        if self.image is None:
            raise ValueError(f"Failed to read {self.image_path}")
        self.boxes = self.load_boxes()
        self.drawing = False
        self.start_point = None
        self.current_point = None

    def load_boxes(self):
        if not self.label_path.is_file():
            return []
        height, width = self.image.shape[:2]
        boxes = []
        for line in self.label_path.read_text(encoding="utf-8").splitlines():
            values = line.split()
            if len(values) != 5 or values[0] != "0":
                continue
            _, center_x, center_y, box_width, box_height = map(float, values)
            x1 = round((center_x - box_width / 2.0) * width)
            y1 = round((center_y - box_height / 2.0) * height)
            x2 = round((center_x + box_width / 2.0) * width)
            y2 = round((center_y + box_height / 2.0) * height)
            boxes.append(self.clamp_box((x1, y1, x2, y2)))
        return boxes

    def clamp_box(self, box):
        height, width = self.image.shape[:2]
        x1, y1, x2, y2 = box
        x1, x2 = sorted((max(0, min(width - 1, x1)), max(0, min(width - 1, x2))))
        y1, y2 = sorted((max(0, min(height - 1, y1)), max(0, min(height - 1, y2))))
        return x1, y1, x2, y2

    def mouse_callback(self, event, x, y, _flags, _userdata):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.start_point = (x, y)
            self.current_point = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            self.current_point = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self.drawing:
            self.drawing = False
            self.current_point = (x, y)
            x1, y1 = self.start_point
            x2, y2 = self.current_point
            box = self.clamp_box((x1, y1, x2, y2))
            if box[2] - box[0] >= 3 and box[3] - box[1] >= 3:
                self.boxes.append(box)

    def render(self):
        canvas = self.image.copy()
        for x1, y1, x2, y2 in self.boxes:
            cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                canvas, "bottle", (x1, max(18, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2,
            )
        if self.drawing and self.start_point and self.current_point:
            cv2.rectangle(canvas, self.start_point, self.current_point, (0, 255, 255), 1)
        text = (
            f"{self.index + 1}/{len(self.images)}  boxes={len(self.boxes)}  "
            "drag: add | u: undo | r: clear | s: save+next | d: empty+next | "
            "a: previous | q: quit"
        )
        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 28), (20, 20, 20), -1)
        cv2.putText(
            canvas, text, (5, 20), cv2.FONT_HERSHEY_SIMPLEX,
            0.42, (255, 255, 255), 1,
        )
        return canvas

    def save_current_label(self, empty=False):
        height, width = self.image.shape[:2]
        lines = []
        if not empty:
            for x1, y1, x2, y2 in self.boxes:
                center_x = ((x1 + x2) / 2.0) / width
                center_y = ((y1 + y2) / 2.0) / height
                box_width = (x2 - x1) / width
                box_height = (y2 - y1) / height
                lines.append(
                    f"0 {center_x:.6f} {center_y:.6f} "
                    f"{box_width:.6f} {box_height:.6f}"
                )
        self.label_path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")

    def change_image(self, next_index):
        self.index = max(0, min(len(self.images) - 1, next_index))
        self.load_current_image()

    def run(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        self.load_current_image()
        print("Class ID 0 is always 'bottle'. Press q to finish at any time.")

        while True:
            cv2.imshow(self.window_name, self.render())
            key = cv2.waitKey(20) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("u") and self.boxes:
                self.boxes.pop()
            elif key == ord("r"):
                self.boxes.clear()
            elif key == ord("a"):
                self.change_image(self.index - 1)
            elif key in (ord("s"), ord("d")):
                self.save_current_label(empty=key == ord("d"))
                if self.index == len(self.images) - 1:
                    print("All images have labels. Press q to close.")
                else:
                    self.change_image(self.index + 1)

        cv2.destroyAllWindows()


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Draw PET-bottle rectangles and write class-0 YOLO labels."
    )
    parser.add_argument(
        "--images",
        type=Path,
        default=Path("~/ros2_ws/datasets/amir_bottle/images/raw").expanduser(),
    )
    parser.add_argument(
        "--labels",
        type=Path,
        default=Path("~/ros2_ws/datasets/amir_bottle/labels/raw").expanduser(),
    )
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    BottleImageLabeler(
        arguments.images.expanduser().resolve(),
        arguments.labels.expanduser().resolve(),
    ).run()


if __name__ == "__main__":
    main()
