from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def read_classes(path: Path) -> list[str]:
    if not path.exists():
        return []
    return [x.strip() for x in path.read_text(encoding="utf-8").splitlines() if x.strip()]


def draw_labels(img, label_path: Path, classes: list[str]):
    h, w = img.shape[:2]
    if not label_path.exists():
        return img
    for line in label_path.read_text(encoding="utf-8").splitlines():
        parts = line.strip().split()
        if len(parts) < 5:
            continue
        cls = int(float(parts[0]))
        x, y, bw, bh = map(float, parts[1:5])
        x1 = int((x - bw / 2.0) * w)
        y1 = int((y - bh / 2.0) * h)
        x2 = int((x + bw / 2.0) * w)
        y2 = int((y + bh / 2.0) * h)
        color = ((37 * cls) % 255, (91 * cls + 80) % 255, (163 * cls + 120) % 255)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
        name = classes[cls] if 0 <= cls < len(classes) else str(cls)
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = max(0.65, min(1.0, w / 900.0))
        thickness = 2
        (tw, th), baseline = cv2.getTextSize(name, font, scale, thickness)
        tx = max(0, min(x1, w - tw - 4))
        ty = max(th + baseline + 4, y1 - 6)
        if ty > y1 and y2 + th + baseline + 8 < h:
            ty = y2 + th + baseline + 6
        cv2.rectangle(img, (tx, ty - th - baseline - 4), (tx + tw + 6, ty + 4), (0, 0, 0), -1)
        cv2.putText(img, name, (tx + 3, ty - baseline), font, scale, (255, 255, 255), thickness, cv2.LINE_AA)
    return img


def main() -> None:
    parser = argparse.ArgumentParser(description="Make a contact sheet for YOLO labeled images.")
    parser.add_argument("--image-dir", type=Path, required=True)
    parser.add_argument("--classes", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--cols", type=int, default=3)
    parser.add_argument("--thumb-w", type=int, default=426)
    parser.add_argument("--thumb-h", type=int, default=240)
    args = parser.parse_args()

    classes = read_classes(args.classes)
    images = sorted(p for p in args.image_dir.iterdir() if p.is_file() and p.suffix.lower() in IMAGE_EXTS)
    thumbs = []
    for image in images:
        img = cv2.imread(str(image))
        if img is None:
            continue
        img = cv2.resize(img, (args.thumb_w, args.thumb_h))
        img = draw_labels(img, image.with_suffix(".txt"), classes)
        thumbs.append(img)
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
