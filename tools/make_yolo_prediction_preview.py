from __future__ import annotations

import argparse
import os
import random
import sys
from pathlib import Path

import cv2
import numpy as np


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def load_ultralytics():
    local_root = Path(r"E:\Yolo\ultralytics-8.3.163\ultralytics-8.3.163")
    os.environ.setdefault("YOLO_CONFIG_DIR", str(Path(r"C:\Users\Administrator\Desktop\Yolo\.yolo_config")))
    if local_root.exists():
        sys.path.insert(0, str(local_root))
    from ultralytics import YOLO

    return YOLO


def read_classes(path: Path) -> list[str]:
    return [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def draw_label(img, x1, y1, text, color):
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = max(0.65, min(1.0, img.shape[1] / 900.0))
    thickness = 2
    (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
    tx = max(0, min(int(x1), img.shape[1] - tw - 4))
    ty = max(th + baseline + 4, int(y1) - 4)
    cv2.rectangle(img, (tx, ty - th - baseline - 4), (tx + tw + 6, ty + 4), (0, 0, 0), -1)
    cv2.putText(img, text, (tx + 3, ty - baseline), font, scale, color, thickness, cv2.LINE_AA)


def draw_predictions(img, result, classes: list[str]):
    if result.boxes is None:
        return img
    boxes = result.boxes.xyxy.cpu().numpy()
    cls_ids = result.boxes.cls.cpu().numpy()
    confs = result.boxes.conf.cpu().numpy()
    for coords, cls_id, conf in zip(boxes, cls_ids, confs):
        cls = int(cls_id)
        color = ((37 * cls) % 255, (91 * cls + 80) % 255, (163 * cls + 120) % 255)
        x1, y1, x2, y2 = map(int, coords)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
        name = classes[cls] if 0 <= cls < len(classes) else str(cls)
        draw_label(img, x1, y1, f"{name} {conf:.2f}", color)
    return img


def main() -> None:
    parser = argparse.ArgumentParser(description="Make a YOLO prediction contact sheet.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--image-dir", type=Path, required=True)
    parser.add_argument("--classes", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--count", type=int, default=16)
    parser.add_argument("--seed", type=int, default=20260527)
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--cols", type=int, default=2)
    parser.add_argument("--thumb-w", type=int, default=640)
    parser.add_argument("--thumb-h", type=int, default=360)
    args = parser.parse_args()

    images = sorted(p for p in args.image_dir.rglob("*") if p.is_file() and p.suffix.lower() in IMAGE_EXTS)
    rng = random.Random(args.seed)
    rng.shuffle(images)
    images = images[: args.count]
    classes = read_classes(args.classes)
    YOLO = load_ultralytics()
    model = YOLO(str(args.model))

    thumbs = []
    for image in images:
        original = cv2.imread(str(image))
        if original is None:
            continue
        result = model.predict(str(image), imgsz=args.imgsz, conf=args.conf, iou=0.45, verbose=False)[0]
        drawn = draw_predictions(original, result, classes)
        thumbs.append(cv2.resize(drawn, (args.thumb_w, args.thumb_h)))
    if not thumbs:
        raise RuntimeError("no preview images")

    blank = np.zeros_like(thumbs[0])
    rows = []
    for i in range(0, len(thumbs), args.cols):
        row = thumbs[i:i + args.cols]
        while len(row) < args.cols:
            row.append(blank.copy())
        rows.append(np.hstack(row))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.out), np.vstack(rows))
    print(args.out)


if __name__ == "__main__":
    main()
