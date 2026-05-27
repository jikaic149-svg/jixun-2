from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


@dataclass
class Box:
    cls: int
    x1: float
    y1: float
    x2: float
    y2: float
    conf: float = 1.0

    @property
    def area(self) -> float:
        return max(0.0, self.x2 - self.x1) * max(0.0, self.y2 - self.y1)


def load_ultralytics():
    local_root = Path(r"E:\Yolo\ultralytics-8.3.163\ultralytics-8.3.163")
    os.environ.setdefault("YOLO_CONFIG_DIR", str(Path(r"C:\Users\Administrator\Desktop\Yolo\.yolo_config")))
    if local_root.exists():
        sys.path.insert(0, str(local_root))
    from ultralytics import YOLO

    return YOLO


def read_classes(path: Path) -> list[str]:
    return [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def read_labels(path: Path, width: int, height: int, class_count: int) -> tuple[list[Box], list[str]]:
    boxes: list[Box] = []
    errors: list[str] = []
    if not path.exists():
        return boxes, ["missing_label"]
    for idx, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        parts = line.strip().split()
        if len(parts) < 5:
            errors.append(f"bad_line_{idx}")
            continue
        try:
            cls = int(float(parts[0]))
            cx, cy, bw, bh = map(float, parts[1:5])
        except ValueError:
            errors.append(f"parse_error_{idx}")
            continue
        if cls < 0 or cls >= class_count:
            errors.append(f"class_oob_{idx}")
            continue
        if bw <= 0 or bh <= 0 or bw > 1.0 or bh > 1.0 or cx < 0 or cx > 1.0 or cy < 0 or cy > 1.0:
            errors.append(f"coord_oob_{idx}")
            continue
        x1 = (cx - bw * 0.5) * width
        y1 = (cy - bh * 0.5) * height
        x2 = (cx + bw * 0.5) * width
        y2 = (cy + bh * 0.5) * height
        if x1 < -1 or y1 < -1 or x2 > width + 1 or y2 > height + 1:
            errors.append(f"box_outside_{idx}")
        if (x2 - x1) < 4 or (y2 - y1) < 4:
            errors.append(f"box_too_small_{idx}")
        boxes.append(Box(cls, x1, y1, x2, y2))
    if not boxes:
        errors.append("empty_label")
    return boxes, errors


def iou(a: Box, b: Box) -> float:
    x1 = max(a.x1, b.x1)
    y1 = max(a.y1, b.y1)
    x2 = min(a.x2, b.x2)
    y2 = min(a.y2, b.y2)
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    union = a.area + b.area - inter
    return inter / union if union > 0 else 0.0


def label_for_image(image: Path, image_root: Path, label_root: Path | None) -> Path:
    if label_root is None:
        return image.with_suffix(".txt")
    try:
        rel = image.relative_to(image_root)
        return (label_root / rel).with_suffix(".txt")
    except ValueError:
        return label_root / f"{image.stem}.txt"


def geometry_check(images: list[Path], class_count: int, image_root: Path, label_root: Path | None) -> tuple[list[dict], dict]:
    rows = []
    bad_images = 0
    total_boxes = 0
    for image in images:
        img = cv2.imread(str(image))
        if img is None:
            rows.append({"image": str(image), "box_count": 0, "errors": "unreadable"})
            bad_images += 1
            continue
        h, w = img.shape[:2]
        boxes, errors = read_labels(label_for_image(image, image_root, label_root), w, h, class_count)
        total_boxes += len(boxes)
        if errors:
            bad_images += 1
        rows.append({"image": str(image), "box_count": len(boxes), "errors": ";".join(errors)})
    summary = {
        "images": len(images),
        "bad_label_images": bad_images,
        "bad_label_rate": bad_images / len(images) if images else 0.0,
        "total_label_boxes": total_boxes,
    }
    return rows, summary


def select_eval_images(images: list[Path], max_images: int) -> list[Path]:
    if max_images <= 0 or len(images) <= max_images:
        return images
    base = [p for p in images if "synth_mask_random_v2" not in str(p)]
    synth = [p for p in images if "synth_mask_random_v2" in str(p)]
    half = max_images // 2
    selected = base[:half] + synth[: max_images - min(half, len(base))]
    if len(selected) < max_images:
        rest = [p for p in images if p not in selected]
        selected += rest[: max_images - len(selected)]
    return selected


def predict_consistency(
    images: list[Path],
    classes: list[str],
    model_path: Path,
    image_root: Path,
    label_root: Path | None,
    imgsz: int,
    conf: float,
    iou_match: float,
    out_dir: Path,
) -> tuple[list[dict], dict]:
    YOLO = load_ultralytics()
    model = YOLO(str(model_path))
    rows = []
    issue_images = []
    total_labels = 0
    matched = 0
    class_mismatch = 0
    missed = 0
    extra = 0
    low_iou = 0

    for image in images:
        img = cv2.imread(str(image))
        if img is None:
            continue
        h, w = img.shape[:2]
        gt, label_errors = read_labels(label_for_image(image, image_root, label_root), w, h, len(classes))
        total_labels += len(gt)
        result = model.predict(str(image), imgsz=imgsz, conf=conf, iou=0.45, verbose=False)[0]
        preds: list[Box] = []
        if result.boxes is not None and len(result.boxes) > 0:
            xyxy = result.boxes.xyxy.cpu().numpy()
            cls_ids = result.boxes.cls.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy()
            for coords, cls_id, pred_conf in zip(xyxy, cls_ids, confs):
                preds.append(Box(int(cls_id), float(coords[0]), float(coords[1]), float(coords[2]), float(coords[3]), float(pred_conf)))

        used_preds: set[int] = set()
        image_missed = 0
        image_mismatch = 0
        image_low_iou = 0
        for g in gt:
            best_idx = -1
            best_iou = 0.0
            for idx, p in enumerate(preds):
                if idx in used_preds:
                    continue
                score = iou(g, p)
                if score > best_iou:
                    best_iou = score
                    best_idx = idx
            if best_idx < 0 or best_iou < 0.10:
                missed += 1
                image_missed += 1
                continue
            used_preds.add(best_idx)
            if best_iou < iou_match:
                low_iou += 1
                image_low_iou += 1
            if preds[best_idx].cls != g.cls:
                class_mismatch += 1
                image_mismatch += 1
            else:
                matched += 1

        image_extra = max(0, len(preds) - len(used_preds))
        extra += image_extra
        issue_score = image_missed + image_mismatch + image_low_iou + image_extra + len(label_errors)
        rows.append(
            {
                "image": str(image),
                "labels": len(gt),
                "preds": len(preds),
                "matched_same_class": len(gt) - image_missed - image_mismatch,
                "missed": image_missed,
                "class_mismatch": image_mismatch,
                "low_iou": image_low_iou,
                "extra_preds": image_extra,
                "label_errors": ";".join(label_errors),
                "issue_score": issue_score,
            }
        )
        if issue_score > 0:
            issue_images.append((issue_score, image, gt, preds))

    out_dir.mkdir(parents=True, exist_ok=True)
    issue_images.sort(key=lambda x: x[0], reverse=True)
    make_issue_sheet(issue_images[:12], classes, out_dir / "issue_contact.jpg")

    eval_images = len(images)
    error_units = missed + class_mismatch + low_iou + extra
    denominator = max(1, total_labels + extra)
    summary = {
        "eval_images": eval_images,
        "model": str(model_path),
        "imgsz": imgsz,
        "conf": conf,
        "iou_match": iou_match,
        "total_labels": total_labels,
        "matched_same_class": matched,
        "missed": missed,
        "class_mismatch": class_mismatch,
        "low_iou": low_iou,
        "extra_preds": extra,
        "consistency_error_rate": error_units / denominator,
    }
    return rows, summary


def draw_box(img: np.ndarray, box: Box, text: str, color: tuple[int, int, int], thickness: int) -> None:
    x1, y1, x2, y2 = map(int, [box.x1, box.y1, box.x2, box.y2])
    cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = max(0.55, min(0.9, img.shape[1] / 1000.0))
    (tw, th), baseline = cv2.getTextSize(text, font, scale, 2)
    tx = max(0, min(x1, img.shape[1] - tw - 4))
    ty = max(th + baseline + 4, y1 - 4)
    cv2.rectangle(img, (tx, ty - th - baseline - 4), (tx + tw + 6, ty + 4), (0, 0, 0), -1)
    cv2.putText(img, text, (tx + 3, ty - baseline), font, scale, color, 2, cv2.LINE_AA)


def make_issue_sheet(items, classes: list[str], out: Path) -> None:
    thumbs = []
    for score, image, gt, preds in items:
        img = cv2.imread(str(image))
        if img is None:
            continue
        img = cv2.resize(img, (640, 360))
        sx = 640 / max(1, cv2.imread(str(image)).shape[1])
        sy = 360 / max(1, cv2.imread(str(image)).shape[0])
        for box in gt:
            scaled = Box(box.cls, box.x1 * sx, box.y1 * sy, box.x2 * sx, box.y2 * sy)
            draw_box(img, scaled, f"GT:{classes[box.cls]}", (0, 255, 255), 2)
        for box in preds:
            name = classes[box.cls] if 0 <= box.cls < len(classes) else str(box.cls)
            scaled = Box(box.cls, box.x1 * sx, box.y1 * sy, box.x2 * sx, box.y2 * sy)
            draw_box(img, scaled, f"P:{name} {box.conf:.2f}", (0, 80, 255), 2)
        cv2.putText(img, f"issue={score}", (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        thumbs.append(img)
    if not thumbs:
        return
    blank = np.zeros_like(thumbs[0])
    rows = []
    cols = 2
    for i in range(0, len(thumbs), cols):
        row = thumbs[i:i + cols]
        while len(row) < cols:
            row.append(blank.copy())
        rows.append(np.hstack(row))
    cv2.imwrite(str(out), np.vstack(rows))


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate YOLO label geometry and model-label consistency.")
    parser.add_argument("--project", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--image-root", type=Path, default=None)
    parser.add_argument("--label-root", type=Path, default=None)
    parser.add_argument("--max-images", type=int, default=260)
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou-match", type=float, default=0.45)
    args = parser.parse_args()

    classes = read_classes(args.project / "classes.txt")
    image_root = args.image_root or (args.project / "raw_images")
    label_root = args.label_root
    images = sorted(p for p in image_root.rglob("*") if p.is_file() and p.suffix.lower() in IMAGE_EXTS)
    geo_rows, geo_summary = geometry_check(images, len(classes), image_root, label_root)
    eval_images = select_eval_images(images, args.max_images)
    pred_rows, pred_summary = predict_consistency(
        eval_images, classes, args.model, image_root, label_root, args.imgsz, args.conf, args.iou_match, args.out_dir
    )

    write_csv(args.out_dir / "label_geometry.csv", geo_rows)
    write_csv(args.out_dir / "prediction_consistency.csv", pred_rows)
    summary = {"geometry": geo_summary, "prediction_consistency": pred_summary}
    (args.out_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
