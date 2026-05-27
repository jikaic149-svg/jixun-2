from __future__ import annotations

import argparse
import csv
import math
import os
import random
import shutil
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
YOLO_ROOT = Path(r"C:\Users\Administrator\Desktop\Yolo")
LOCAL_ULTRALYTICS_ROOT = Path(r"E:\Yolo\ultralytics-8.3.163\ultralytics-8.3.163")
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}
VIDEO_EXTS = {".mp4", ".avi", ".mov", ".mkv", ".wmv"}


@dataclass
class Box:
    cls: int
    x1: float
    y1: float
    x2: float
    y2: float


def log(msg: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def setup_ultralytics(project: Path) -> None:
    os.environ["YOLO_CONFIG_DIR"] = str(project / ".yolo_config")
    if LOCAL_ULTRALYTICS_ROOT.exists():
        sys.path.insert(0, str(LOCAL_ULTRALYTICS_ROOT))


def unique_project(root: Path, base_name: str) -> Path:
    candidate = root / base_name
    if not candidate.exists():
        return candidate
    idx = 1
    while True:
        candidate = root / f"{base_name}_{idx:02d}"
        if not candidate.exists():
            return candidate
        idx += 1


def ensure_project(project: Path, classes: list[str]) -> None:
    for sub in [
        "raw_images",
        "images/train",
        "images/val",
        "labels/train",
        "labels/val",
        "results/train",
        "results/deploy",
        "tools",
    ]:
        (project / sub).mkdir(parents=True, exist_ok=True)
    (project / "classes.txt").write_text("\n".join(classes) + "\n", encoding="utf-8")
    write_data_yaml(project, classes)


def read_classes(project: Path) -> list[str]:
    classes = [
        line.strip()
        for line in (project / "classes.txt").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if not classes:
        raise RuntimeError(f"empty classes file: {project / 'classes.txt'}")
    return classes


def write_data_yaml(project: Path, classes: list[str]) -> None:
    lines = [
        f"path: {project.as_posix()}",
        "train: images/train",
        "val: images/val",
        "",
        "names:",
    ]
    lines.extend(f"  {idx}: {name}" for idx, name in enumerate(classes))
    (project / "data.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")


def label_for_image(image_path: Path) -> Path:
    return image_path.with_suffix(".txt")


def safe_stem(prefix: str, path: Path) -> str:
    return f"{prefix}_{path.stem}".replace(" ", "_")


def copy_pair(image: Path, label: Path, out_raw: Path, prefix: str) -> bool:
    if not image.exists() or not label.exists():
        return False
    stem = safe_stem(prefix, image)
    out_image = out_raw / f"{stem}{image.suffix.lower()}"
    out_label = out_raw / f"{stem}.txt"
    if out_image.exists() or out_label.exists():
        return False
    shutil.copy2(image, out_image)
    shutil.copy2(label, out_label)
    return True


def copy_labeled_from_raw(source_project: Path, out_raw: Path, prefix: str) -> int:
    copied = 0
    raw = source_project / "raw_images"
    if not raw.exists():
        return 0
    for image in sorted(raw.rglob("*")):
        if not image.is_file() or image.suffix.lower() not in IMAGE_EXTS:
            continue
        label = label_for_image(image)
        if label.exists() and copy_pair(image, label, out_raw, prefix):
            copied += 1
    return copied


def copy_labeled_from_split(source_project: Path, out_raw: Path, prefix: str) -> int:
    copied = 0
    for split in ["train", "val"]:
        image_dir = source_project / "images" / split
        label_dir = source_project / "labels" / split
        if not image_dir.exists() or not label_dir.exists():
            continue
        for image in sorted(image_dir.iterdir()):
            if not image.is_file() or image.suffix.lower() not in IMAGE_EXTS:
                continue
            label = label_dir / f"{image.stem}.txt"
            if label.exists() and copy_pair(image, label, out_raw, f"{prefix}_{split}"):
                copied += 1
    return copied


def extract_frames(
    videos: list[Path],
    out_raw: Path,
    frame_step: int,
    offset: int,
    max_frames_per_video: int,
) -> list[Path]:
    frames: list[Path] = []
    for video in videos:
        if not video.exists() or video.suffix.lower() not in VIDEO_EXTS:
            log(f"skip non-video: {video}")
            continue
        cap = cv2.VideoCapture(str(video))
        if not cap.isOpened():
            log(f"skip unreadable video: {video}")
            continue
        frame_id = 0
        saved = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if frame_id >= offset and (frame_id - offset) % max(1, frame_step) == 0:
                out = out_raw / f"cont_{video.stem}_f{frame_id:06d}.jpg"
                cv2.imwrite(str(out), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 94])
                frames.append(out)
                saved += 1
                if max_frames_per_video > 0 and saved >= max_frames_per_video:
                    break
            frame_id += 1
        cap.release()
        log(f"extracted {saved} frames from {video.name}")
    return frames


def yolo_line(cls: int, xywhn: list[float]) -> str:
    x, y, w, h = xywhn
    return f"{cls} {x:.6f} {y:.6f} {w:.6f} {h:.6f}"


def pseudo_label_frames(
    frames: list[Path],
    model_path: Path,
    class_count: int,
    imgsz: int,
    conf: float,
    iou: float,
    min_classes: int,
    device: str,
    manifest: Path,
) -> int:
    from ultralytics import YOLO

    model = YOLO(str(model_path))
    accepted = 0
    partial = 0
    empty = 0
    manifest.parent.mkdir(parents=True, exist_ok=True)
    with manifest.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["image", "status", "classes", "min_conf", "mean_conf"])
        for idx, image_path in enumerate(frames, start=1):
            result = model.predict(
                source=str(image_path),
                imgsz=imgsz,
                conf=conf,
                iou=iou,
                max_det=30,
                device=device or None,
                verbose=False,
            )[0]

            best: dict[int, tuple[float, list[float]]] = {}
            if result.boxes is not None and len(result.boxes) > 0:
                cls_list = result.boxes.cls.cpu().numpy().astype(int).tolist()
                conf_list = result.boxes.conf.cpu().numpy().astype(float).tolist()
                xywhn_list = result.boxes.xywhn.cpu().numpy().tolist()
                for cls, score, xywhn in zip(cls_list, conf_list, xywhn_list):
                    if cls < 0 or cls >= class_count:
                        continue
                    prev = best.get(cls)
                    if prev is None or score > prev[0]:
                        best[cls] = (score, xywhn)

            found = sorted(best)
            if len(found) >= min_classes:
                lines = [yolo_line(cls, best[cls][1]) for cls in found]
                label_for_image(image_path).write_text("\n".join(lines) + "\n", encoding="utf-8")
                confs = [best[cls][0] for cls in found]
                writer.writerow(
                    [
                        image_path.name,
                        "accepted",
                        " ".join(map(str, found)),
                        f"{min(confs):.6f}",
                        f"{sum(confs) / len(confs):.6f}",
                    ]
                )
                accepted += 1
            else:
                status = "empty" if not found else "partial"
                if status == "empty":
                    empty += 1
                else:
                    partial += 1
                writer.writerow([image_path.name, status, " ".join(map(str, found)), "", ""])

            if idx % 50 == 0 or idx == len(frames):
                log(f"pseudo {idx}/{len(frames)} accepted={accepted} partial={partial} empty={empty}")
    return accepted


def read_yolo_boxes(label_path: Path, width: int, height: int) -> list[Box]:
    boxes: list[Box] = []
    for line in label_path.read_text(encoding="utf-8").splitlines():
        parts = line.strip().split()
        if len(parts) < 5:
            continue
        cls = int(float(parts[0]))
        cx = float(parts[1]) * width
        cy = float(parts[2]) * height
        bw = float(parts[3]) * width
        bh = float(parts[4]) * height
        boxes.append(Box(cls, cx - bw / 2.0, cy - bh / 2.0, cx + bw / 2.0, cy + bh / 2.0))
    return boxes


def write_yolo_boxes(label_path: Path, boxes: list[Box], width: int, height: int) -> None:
    lines = []
    for box in boxes:
        x1 = max(0.0, min(float(width - 1), box.x1))
        y1 = max(0.0, min(float(height - 1), box.y1))
        x2 = max(0.0, min(float(width - 1), box.x2))
        y2 = max(0.0, min(float(height - 1), box.y2))
        bw = max(1.0, x2 - x1)
        bh = max(1.0, y2 - y1)
        cx = x1 + bw / 2.0
        cy = y1 + bh / 2.0
        lines.append(f"{box.cls} {cx / width:.6f} {cy / height:.6f} {bw / width:.6f} {bh / height:.6f}")
    label_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def apply_motion_blur(img: np.ndarray, rng: random.Random, max_kernel: int) -> np.ndarray:
    k = rng.randrange(3, max_kernel + 1, 2)
    kernel = np.zeros((k, k), dtype=np.float32)
    angle = rng.uniform(0.0, math.pi)
    cx = (k - 1) * 0.5
    length = (k - 1) * 0.5
    x1 = int(cx - math.cos(angle) * length)
    y1 = int(cx - math.sin(angle) * length)
    x2 = int(cx + math.cos(angle) * length)
    y2 = int(cx + math.sin(angle) * length)
    cv2.line(kernel, (x1, y1), (x2, y2), 1.0, thickness=1)
    kernel /= max(kernel.sum(), 1.0)
    return cv2.filter2D(img, -1, kernel)


def color_jitter(img: np.ndarray, rng: random.Random) -> np.ndarray:
    out = cv2.convertScaleAbs(img, alpha=rng.uniform(0.86, 1.18), beta=rng.uniform(-18, 18))
    if rng.random() < 0.40:
        sigma = rng.uniform(1.0, 5.0)
        noise = np.random.normal(0, sigma, out.shape).astype(np.float32)
        out = np.clip(out.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    return out


def draw_laser_spot(img: np.ndarray, boxes: list[Box], rng: random.Random) -> np.ndarray:
    if not boxes:
        return img
    out = img.copy()
    h, w = out.shape[:2]
    box = rng.choice(boxes)
    bx1, by1, bx2, by2 = box.x1, box.y1, box.x2, box.y2
    cx = int((bx1 + bx2) * 0.5 + rng.uniform(-0.18, 0.18) * max(1.0, bx2 - bx1))
    cy = int((by1 + by2) * 0.5 + rng.uniform(-0.18, 0.18) * max(1.0, by2 - by1))
    cx = max(0, min(w - 1, cx))
    cy = max(0, min(h - 1, cy))
    radius = rng.randint(2, 5)
    color = (rng.randint(205, 255), rng.randint(60, 130), rng.randint(20, 65))
    cv2.circle(out, (cx, cy), radius + 5, color, -1, lineType=cv2.LINE_AA)
    cv2.circle(out, (cx, cy), radius + 2, (255, 150, 80), -1, lineType=cv2.LINE_AA)
    cv2.circle(out, (cx, cy), max(1, radius // 2), (255, 255, 255), -1, lineType=cv2.LINE_AA)
    out = cv2.GaussianBlur(out, (0, 0), sigmaX=rng.uniform(0.25, 0.8))
    return out


def make_augmented_samples(raw: Path, count: int, seed: int) -> int:
    rng = random.Random(seed)
    np.random.seed(seed & 0xFFFF)
    source = [
        p
        for p in raw.rglob("*")
        if p.is_file()
        and p.suffix.lower() in IMAGE_EXTS
        and label_for_image(p).exists()
        and "_preview" not in p.parts
        and "synth_random" not in p.parts
    ]
    rng.shuffle(source)
    if not source or count <= 0:
        return 0
    out_dir = raw / "laser_motion_aug"
    out_dir.mkdir(parents=True, exist_ok=True)
    made = 0
    idx = 0
    while made < count and idx < count * 4:
        image_path = source[idx % len(source)]
        idx += 1
        img = cv2.imread(str(image_path))
        if img is None:
            continue
        h, w = img.shape[:2]
        boxes = read_yolo_boxes(label_for_image(image_path), w, h)
        if not boxes:
            continue
        out = color_jitter(img, rng)
        if rng.random() < 0.70:
            out = draw_laser_spot(out, boxes, rng)
        if rng.random() < 0.75:
            out = apply_motion_blur(out, rng, max_kernel=13)
        stem = f"laser_motion_{made:06d}"
        out_image = out_dir / f"{stem}.jpg"
        out_label = out_dir / f"{stem}.txt"
        cv2.imwrite(str(out_image), out, [int(cv2.IMWRITE_JPEG_QUALITY), rng.randint(86, 95)])
        write_yolo_boxes(out_label, boxes, w, h)
        made += 1
        if made % 100 == 0:
            log(f"laser/motion aug {made}/{count}")
    return made


def prepare_dataset(project: Path, val_ratio: float, seed: int) -> tuple[int, int]:
    rng = random.Random(seed)
    pairs: list[tuple[Path, Path]] = []
    raw = project / "raw_images"
    for image in raw.rglob("*"):
        if image.is_file() and image.suffix.lower() in IMAGE_EXTS:
            label = label_for_image(image)
            if label.exists():
                pairs.append((image, label))
    if not pairs:
        raise RuntimeError("no labeled images found")
    rng.shuffle(pairs)
    val_count = max(1, int(len(pairs) * val_ratio)) if len(pairs) > 1 else 0
    val_stems = {image.stem for image, _ in pairs[:val_count]}

    for sub in ["images/train", "images/val", "labels/train", "labels/val"]:
        folder = project / sub
        for item in folder.iterdir():
            if item.is_file():
                item.unlink()

    train_count = 0
    actual_val = 0
    for image, label in pairs:
        split = "val" if image.stem in val_stems else "train"
        shutil.copy2(image, project / "images" / split / image.name)
        shutil.copy2(label, project / "labels" / split / f"{image.stem}.txt")
        if split == "val":
            actual_val += 1
        else:
            train_count += 1
    return train_count, actual_val


def run_synth_generator(project: Path, count: int, python: Path) -> None:
    if count <= 0:
        return
    import subprocess

    script = ROOT / "tools" / "synthesize_random_layouts_jixun2.py"
    cmd = [
        str(python),
        str(script),
        "--project",
        str(project),
        "--count",
        str(count),
        "--min-objects",
        "5",
        "--max-objects",
        "8",
        "--scale-min",
        "0.80",
        "--scale-max",
        "1.70",
        "--row-layout-prob",
        "0.35",
        "--motion-blur-prob",
        "0.55",
        "--motion-blur-max",
        "13",
    ]
    log("RUN: " + " ".join(cmd))
    subprocess.run(cmd, check=True, cwd=str(ROOT))


def train_model(project: Path, model_path: Path, epochs: int, imgsz: int, device: str, name: str) -> Path:
    from ultralytics import YOLO

    model = YOLO(str(model_path))
    result = model.train(
        data=str(project / "data.yaml"),
        imgsz=imgsz,
        epochs=epochs,
        project=str(project / "results"),
        name="train",
        exist_ok=True,
        workers=0,
        device=device or None,
        close_mosaic=8,
    )
    run_dir = Path(result.save_dir)
    best = run_dir / "weights" / "best.pt"
    if not best.exists():
        raise FileNotFoundError(f"training did not produce best.pt: {best}")
    log(f"best={best}")
    return best


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build and train a continued jixun2 YOLO dataset.")
    parser.add_argument("--project-name", default="jixun2_continue_auto_20260526")
    parser.add_argument("--source-project", type=Path, default=YOLO_ROOT / "projects" / "jixun2")
    parser.add_argument("--previous-project", type=Path, default=ROOT / ".yolo_auto" / "project")
    parser.add_argument("--model", type=Path, default=ROOT / ".yolo_auto" / "project" / "results" / "train" / "weights" / "best.pt")
    parser.add_argument("--python", type=Path, default=Path(r"E:\Yolo\Anaconda\envs\yolo\python.exe"))
    parser.add_argument("--video", type=Path, action="append", default=[])
    parser.add_argument("--frame-step", type=int, default=18)
    parser.add_argument("--frame-offset", type=int, default=7)
    parser.add_argument("--max-frames-per-video", type=int, default=280)
    parser.add_argument("--pseudo-conf", type=float, default=0.32)
    parser.add_argument("--pseudo-imgsz", type=int, default=960)
    parser.add_argument("--min-classes", type=int, default=8)
    parser.add_argument("--laser-motion-count", type=int, default=520)
    parser.add_argument("--synth-count", type=int, default=520)
    parser.add_argument("--val-ratio", type=float, default=0.18)
    parser.add_argument("--epochs", type=int, default=22)
    parser.add_argument("--train-imgsz", type=int, default=960)
    parser.add_argument("--device", default="")
    parser.add_argument("--skip-train", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.source_project.exists():
        raise FileNotFoundError(args.source_project)
    if not args.model.exists():
        raise FileNotFoundError(args.model)
    classes = read_classes(args.source_project)
    projects_root = YOLO_ROOT / "projects"
    project = unique_project(projects_root, args.project_name)
    ensure_project(project, classes)
    setup_ultralytics(project)

    raw = project / "raw_images"
    log(f"project={project}")
    log("copy formal labeled raw samples")
    formal_count = copy_labeled_from_raw(args.source_project, raw, "formal")
    log(f"formal labeled={formal_count}")

    log("copy previous prepared train/val samples")
    previous_count = copy_labeled_from_split(args.previous_project, raw, "prev")
    log(f"previous labeled={previous_count}")

    log("extract continuation frames")
    frames = extract_frames(args.video, raw, args.frame_step, args.frame_offset, args.max_frames_per_video)
    log(f"new frames={len(frames)}")

    log("pseudo-label continuation frames")
    pseudo_count = pseudo_label_frames(
        frames,
        args.model,
        class_count=len(classes),
        imgsz=args.pseudo_imgsz,
        conf=args.pseudo_conf,
        iou=0.45,
        min_classes=args.min_classes,
        device=args.device,
        manifest=project / "pseudo_manifest.csv",
    )
    log(f"pseudo accepted={pseudo_count}")

    log("make laser and motion blur augmentations")
    aug_count = make_augmented_samples(raw, args.laser_motion_count, seed=20260526)
    log(f"laser/motion augmented={aug_count}")

    log("make synthetic random-layout samples")
    run_synth_generator(project, args.synth_count, args.python)

    log("prepare train/val split")
    train_count, val_count = prepare_dataset(project, args.val_ratio, seed=20260526)
    write_data_yaml(project, classes)
    log(f"dataset ready train={train_count} val={val_count}")

    if not args.skip_train:
        log("train continued model")
        best = train_model(project, args.model, args.epochs, args.train_imgsz, args.device, args.project_name)
        log(f"continued best.pt={best}")

    log("done")


if __name__ == "__main__":
    main()
