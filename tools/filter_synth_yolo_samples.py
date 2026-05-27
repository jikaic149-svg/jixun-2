from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def label_path(image: Path) -> Path:
    return image.with_suffix(".txt")


def read_boxes(path: Path, w: int, h: int) -> list[tuple[int, float, float, float, float]]:
    boxes = []
    if not path.exists():
        return boxes
    for line in path.read_text(encoding="utf-8").splitlines():
        parts = line.strip().split()
        if len(parts) < 5:
            continue
        cls = int(float(parts[0]))
        cx = float(parts[1]) * w
        cy = float(parts[2]) * h
        bw = float(parts[3]) * w
        bh = float(parts[4]) * h
        boxes.append((cls, cx - bw / 2.0, cy - bh / 2.0, cx + bw / 2.0, cy + bh / 2.0))
    return boxes


def gray_patch_score(img: np.ndarray) -> float:
    # Detect large flat neutral blocks from failed object removal.
    small = cv2.resize(img, (160, 90), interpolation=cv2.INTER_AREA)
    b, g, r = cv2.split(small)
    neutral = (np.abs(b.astype(np.int16) - g.astype(np.int16)) < 4) & (np.abs(g.astype(np.int16) - r.astype(np.int16)) < 4)
    grayish = neutral & (small.mean(axis=2) > 110) & (small.mean(axis=2) < 235)
    return float(grayish.mean())


def check_image(image: Path, min_boxes: int, max_gray_score: float) -> tuple[bool, str, int, float]:
    img = cv2.imread(str(image))
    if img is None:
        return False, "unreadable", 0, 1.0
    h, w = img.shape[:2]
    boxes = read_boxes(label_path(image), w, h)
    if len(boxes) < min_boxes:
        return False, "too_few_boxes", len(boxes), 0.0

    for cls, x1, y1, x2, y2 in boxes:
        bw = max(0.0, x2 - x1)
        bh = max(0.0, y2 - y1)
        area_ratio = (bw * bh) / float(w * h)
        if bw < 8 or bh < 8:
            return False, "box_too_small", len(boxes), 0.0
        if area_ratio > 0.20:
            return False, "box_too_large", len(boxes), 0.0
        if x1 < -1 or y1 < -1 or x2 > w + 1 or y2 > h + 1:
            return False, "box_outside", len(boxes), 0.0

    gray_score = gray_patch_score(img)
    if gray_score > max_gray_score:
        return False, "gray_patch_like", len(boxes), gray_score
    return True, "ok", len(boxes), gray_score


def main() -> None:
    parser = argparse.ArgumentParser(description="Filter bad synthetic YOLO samples and delete rejected pairs.")
    parser.add_argument("--dir", type=Path, required=True)
    parser.add_argument("--min-boxes", type=int, default=5)
    parser.add_argument("--max-gray-score", type=float, default=0.38)
    parser.add_argument("--report", type=Path, default=None)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    report = args.report or (args.dir / "filter_report.csv")
    images = sorted(p for p in args.dir.iterdir() if p.is_file() and p.suffix.lower() in IMAGE_EXTS)
    kept = 0
    deleted = 0
    rows = []
    for image in images:
        ok, reason, box_count, gray_score = check_image(image, args.min_boxes, args.max_gray_score)
        rows.append([image.name, "keep" if ok else "delete", reason, box_count, f"{gray_score:.6f}"])
        if ok:
            kept += 1
            continue
        deleted += 1
        if not args.dry_run:
            txt = label_path(image)
            image.unlink(missing_ok=True)
            txt.unlink(missing_ok=True)

    report.parent.mkdir(parents=True, exist_ok=True)
    with report.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["image", "action", "reason", "box_count", "gray_score"])
        writer.writerows(rows)
    print(f"checked={len(images)} kept={kept} deleted={deleted} report={report}")


if __name__ == "__main__":
    main()
