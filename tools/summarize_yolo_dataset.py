from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import cv2


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def read_classes(path: Path) -> list[str]:
    return [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def summarize_dir(image_root: Path, classes: list[str], label_root: Path | None = None) -> dict:
    images = sorted(p for p in image_root.rglob("*") if p.is_file() and p.suffix.lower() in IMAGE_EXTS)
    class_counts = {name: 0 for name in classes}
    box_areas = []
    box_wh = []
    missing = 0
    unreadable = 0
    for image in images:
        img = cv2.imread(str(image))
        if img is None:
            unreadable += 1
            continue
        h, w = img.shape[:2]
        if label_root:
            try:
                label = (label_root / image.relative_to(image_root)).with_suffix(".txt")
            except ValueError:
                label = label_root / f"{image.stem}.txt"
        else:
            label = image.with_suffix(".txt")
        if not label.exists():
            missing += 1
            continue
        for line in label.read_text(encoding="utf-8").splitlines():
            parts = line.strip().split()
            if len(parts) < 5:
                continue
            cls = int(float(parts[0]))
            bw = float(parts[3])
            bh = float(parts[4])
            if 0 <= cls < len(classes):
                class_counts[classes[cls]] += 1
            box_areas.append(bw * bh)
            box_wh.append((bw, bh))
    box_areas_sorted = sorted(box_areas)

    def pct(values, q):
        if not values:
            return 0.0
        idx = min(len(values) - 1, max(0, int((len(values) - 1) * q)))
        return values[idx]

    return {
        "image_root": str(image_root),
        "images": len(images),
        "missing_labels": missing,
        "unreadable_images": unreadable,
        "boxes": len(box_areas),
        "boxes_per_image": len(box_areas) / len(images) if images else 0.0,
        "class_counts": class_counts,
        "box_area_norm_p10": pct(box_areas_sorted, 0.10),
        "box_area_norm_p50": pct(box_areas_sorted, 0.50),
        "box_area_norm_p90": pct(box_areas_sorted, 0.90),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Summarize YOLO dataset class and box distribution.")
    parser.add_argument("--project", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--source", action="append", nargs="+", metavar="SOURCE")
    args = parser.parse_args()

    classes = read_classes(args.project / "classes.txt")
    summaries = {}
    for source in args.source or []:
        name = source[0]
        image_root = Path(source[1])
        label_root = Path(source[2]) if len(source) > 2 else None
        summaries[name] = summarize_dir(image_root, classes, label_root)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(summaries, ensure_ascii=False, indent=2), encoding="utf-8")
    print(args.out)


if __name__ == "__main__":
    main()
