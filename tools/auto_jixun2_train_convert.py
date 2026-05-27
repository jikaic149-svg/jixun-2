from __future__ import annotations

import argparse
import os
import random
import shutil
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WORK = ROOT / ".yolo_auto"
SOURCE_PROJECT = Path(r"C:\Users\Administrator\Desktop\Yolo\projects\jixun2")
CONVERTER_SOURCE = Path(r"C:\Users\Administrator\Desktop\Yolo\maixcam2_convert")
LOCAL_ULTRALYTICS_ROOT = Path(r"E:\Yolo\ultralytics-8.3.163\ultralytics-8.3.163")
DEFAULT_PYTHON = Path(r"E:\Yolo\Anaconda\envs\yolo\python.exe")
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}
VIDEO_EXTS = {".mp4", ".avi", ".mov", ".mkv", ".wmv"}


def log(msg: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def run(cmd: list[str], cwd: Path | None = None) -> None:
    log("RUN: " + " ".join(str(x) for x in cmd))
    subprocess.run([str(x) for x in cmd], cwd=str(cwd) if cwd else None, check=True)


def copy_tree_files(src: Path, dst: Path) -> None:
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(src, dst)


def read_classes(project: Path) -> list[str]:
    path = project / "classes.txt"
    classes = [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not classes:
        raise RuntimeError(f"empty classes file: {path}")
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


def copy_old_labeled_samples(source: Path, raw: Path) -> int:
    copied = 0
    for image in sorted((source / "raw_images").rglob("*")):
        if not image.is_file() or image.suffix.lower() not in IMAGE_EXTS:
            continue
        label = label_for_image(image)
        if not label.exists():
            continue
        name = f"old_{image.name}"
        out_image = raw / name
        out_label = raw / f"{Path(name).stem}.txt"
        shutil.copy2(image, out_image)
        shutil.copy2(label, out_label)
        copied += 1
    return copied


def extract_video_frames(incoming: Path, raw: Path, frame_step: int, max_frames_per_video: int) -> list[Path]:
    extracted: list[Path] = []
    for video in sorted(incoming.iterdir()):
        if not video.is_file() or video.suffix.lower() not in VIDEO_EXTS or video.stat().st_size <= 0:
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
            if frame_id % max(1, frame_step) == 0:
                out = raw / f"{video.stem}_frame_{frame_id:06d}.jpg"
                cv2.imwrite(str(out), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 94])
                extracted.append(out)
                saved += 1
                if max_frames_per_video > 0 and saved >= max_frames_per_video:
                    break
            frame_id += 1
        cap.release()
        log(f"extracted {saved} frames from {video.name}")
    return extracted


def setup_ultralytics(work: Path) -> None:
    os.environ["YOLO_CONFIG_DIR"] = str(work / ".yolo_config")
    if LOCAL_ULTRALYTICS_ROOT.exists():
        sys.path.insert(0, str(LOCAL_ULTRALYTICS_ROOT))


def yolo_line(cls: int, xywhn: list[float]) -> str:
    x, y, w, h = xywhn
    return f"{cls} {x:.6f} {y:.6f} {w:.6f} {h:.6f}"


def autolabel_frames(
    frames: list[Path],
    model_path: Path,
    class_count: int,
    imgsz: int,
    conf: float,
    iou: float,
    device: str,
) -> int:
    from ultralytics import YOLO

    model = YOLO(str(model_path))
    ok_count = 0
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

        if len(best) == class_count:
            lines = [yolo_line(cls, best[cls][1]) for cls in sorted(best)]
            label_for_image(image_path).write_text("\n".join(lines) + "\n", encoding="utf-8")
            ok_count += 1

        if idx % 50 == 0 or idx == len(frames):
            log(f"autolabel {idx}/{len(frames)} complete={ok_count}")
    return ok_count


def read_yolo_label(label_path: Path, width: int, height: int) -> list[tuple[int, float, float, float, float]]:
    boxes = []
    for line in label_path.read_text(encoding="utf-8").splitlines():
        parts = line.strip().split()
        if len(parts) < 5:
            continue
        cls = int(float(parts[0]))
        cx = float(parts[1]) * width
        cy = float(parts[2]) * height
        bw = float(parts[3]) * width
        bh = float(parts[4]) * height
        boxes.append((cls, cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2))
    return boxes


def write_yolo_boxes(label_path: Path, boxes: list[tuple[int, float, float, float, float]], width: int, height: int) -> None:
    lines = []
    for cls, x1, y1, x2, y2 in boxes:
        x1 = max(0.0, min(float(width - 1), x1))
        y1 = max(0.0, min(float(height - 1), y1))
        x2 = max(0.0, min(float(width - 1), x2))
        y2 = max(0.0, min(float(height - 1), y2))
        bw = max(1.0, x2 - x1)
        bh = max(1.0, y2 - y1)
        cx = x1 + bw / 2
        cy = y1 + bh / 2
        lines.append(f"{cls} {cx / width:.6f} {cy / height:.6f} {bw / width:.6f} {bh / height:.6f}")
    label_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def center_zoom_augment(raw: Path, limit: int, seed: int) -> int:
    rng = random.Random(seed)
    images = [p for p in raw.rglob("*") if p.is_file() and p.suffix.lower() in IMAGE_EXTS and label_for_image(p).exists()]
    rng.shuffle(images)
    made = 0
    for image_path in images[:limit]:
        img = cv2.imread(str(image_path))
        if img is None:
            continue
        h, w = img.shape[:2]
        boxes = read_yolo_label(label_for_image(image_path), w, h)
        for scale in (1.35, 1.60, 2.00):
            crop_w = int(w / scale)
            crop_h = int(h / scale)
            cx = int(w * (0.50 + rng.uniform(-0.04, 0.04)))
            cy = int(h * (0.50 + rng.uniform(-0.04, 0.04)))
            x0 = max(0, min(w - crop_w, cx - crop_w // 2))
            y0 = max(0, min(h - crop_h, cy - crop_h // 2))
            crop = img[y0:y0 + crop_h, x0:x0 + crop_w]
            if crop.size == 0:
                continue
            out = cv2.resize(crop, (w, h), interpolation=cv2.INTER_LINEAR)
            out_boxes = []
            for cls, x1, y1, x2, y2 in boxes:
                nx1 = (x1 - x0) * scale
                ny1 = (y1 - y0) * scale
                nx2 = (x2 - x0) * scale
                ny2 = (y2 - y0) * scale
                ix1 = max(0.0, nx1)
                iy1 = max(0.0, ny1)
                ix2 = min(float(w - 1), nx2)
                iy2 = min(float(h - 1), ny2)
                old_area = max(1.0, (x2 - x1) * (y2 - y1) * scale * scale)
                new_area = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
                if new_area / old_area >= 0.60:
                    out_boxes.append((cls, ix1, iy1, ix2, iy2))
            if len(out_boxes) < max(1, len(boxes) // 2):
                continue
            stem = f"zoom_s{int(scale * 100):03d}_{image_path.stem}"
            out_image = raw / f"{stem}.jpg"
            out_label = raw / f"{stem}.txt"
            cv2.imwrite(str(out_image), out, [int(cv2.IMWRITE_JPEG_QUALITY), 93])
            write_yolo_boxes(out_label, out_boxes, w, h)
            made += 1
    return made


def run_synthetic_generator(project: Path, count: int, python: Path) -> None:
    script = ROOT / "tools" / "synthesize_random_layouts_jixun2.py"
    if count <= 0:
        return
    run(
        [
            python,
            script,
            "--project",
            project,
            "--count",
            count,
            "--min-objects",
            5,
            "--max-objects",
            8,
            "--scale-min",
            0.85,
            "--scale-max",
            1.55,
            "--motion-blur-prob",
            0.45,
        ],
        cwd=ROOT,
    )


def prepare_dataset(project: Path, val_ratio: float, seed: int) -> tuple[int, int]:
    rng = random.Random(seed)
    pairs = []
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
        if folder.exists():
            shutil.rmtree(folder)
        folder.mkdir(parents=True, exist_ok=True)

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


def train_model(project: Path, work: Path, old_model: Path, model_name: str, epochs: int, imgsz: int, device: str):
    from ultralytics import YOLO

    model = YOLO(str(old_model))
    result = model.train(
        data=str(project / "data.yaml"),
        imgsz=imgsz,
        epochs=epochs,
        project=str(work / "runs"),
        name=model_name,
        exist_ok=True,
        workers=0,
        device=device or None,
        close_mosaic=10,
    )
    run_dir = Path(result.save_dir)
    best = run_dir / "weights" / "best.pt"
    if not best.exists():
        raise FileNotFoundError(f"training did not produce best.pt: {best}")
    results = project / "results" / "train"
    results.mkdir(parents=True, exist_ok=True)
    shutil.copytree(run_dir, results, dirs_exist_ok=True)
    return best


def prepare_converter(work: Path, project: Path, classes: list[str], best: Path, model_name: str, python: Path, calib_count: int) -> Path:
    local_converter = work / "maixcam2_convert"
    copy_tree_files(CONVERTER_SOURCE, local_converter)

    input_pt = local_converter / "input_pt"
    calib = local_converter / "calib_images"
    deploy = local_converter / "deploy"
    for folder in [input_pt, calib, deploy, local_converter / "work"]:
        if folder.exists():
            shutil.rmtree(folder)
        folder.mkdir(parents=True, exist_ok=True)

    shutil.copy2(best, input_pt / f"{model_name}.pt")
    (local_converter / "labels.txt").write_text("\n".join(classes) + "\n", encoding="utf-8")

    images = sorted((project / "images" / "train").glob("*"))
    images = [p for p in images if p.suffix.lower() in IMAGE_EXTS]
    if not images:
        images = sorted((project / "raw_images").rglob("*"))
        images = [p for p in images if p.suffix.lower() in IMAGE_EXTS]
    for idx, image_path in enumerate(images[:calib_count], start=1):
        shutil.copy2(image_path, calib / f"calib_{idx:03d}{image_path.suffix.lower()}")

    run(
        [
            "powershell",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            local_converter / "convert.ps1",
            "-ModelName",
            model_name,
            "-Python",
            python,
            "-Height",
            480,
            "-Width",
            640,
        ],
        cwd=local_converter,
    )

    local_deploy = project / "results" / "deploy"
    local_deploy.mkdir(parents=True, exist_ok=True)
    for file in (local_converter / "deploy").glob(f"{model_name}*"):
        if file.is_file():
            shutil.copy2(file, local_deploy / file.name)
    return local_deploy


def main() -> None:
    parser = argparse.ArgumentParser(description="Autonomous jixun2 YOLO fine-tune and MaixCam2 conversion.")
    parser.add_argument("--work", type=Path, default=DEFAULT_WORK)
    parser.add_argument("--source-project", type=Path, default=SOURCE_PROJECT)
    parser.add_argument("--incoming", type=Path, default=DEFAULT_WORK / "incoming")
    parser.add_argument("--old-model", type=Path, default=SOURCE_PROJECT / "results" / "train" / "weights" / "best.pt")
    parser.add_argument("--python", type=Path, default=DEFAULT_PYTHON)
    parser.add_argument("--model-name", default="jixun2_auto_stable")
    parser.add_argument("--frame-step", type=int, default=10)
    parser.add_argument("--max-frames-per-video", type=int, default=700)
    parser.add_argument("--pseudo-conf", type=float, default=0.23)
    parser.add_argument("--pseudo-imgsz", type=int, default=960)
    parser.add_argument("--train-imgsz", type=int, default=960)
    parser.add_argument("--epochs", type=int, default=35)
    parser.add_argument("--synthetic-count", type=int, default=900)
    parser.add_argument("--zoom-augment-limit", type=int, default=600)
    parser.add_argument("--val-ratio", type=float, default=0.18)
    parser.add_argument("--calib-count", type=int, default=120)
    parser.add_argument("--device", default="")
    parser.add_argument("--skip-train", action="store_true")
    parser.add_argument("--skip-convert", action="store_true")
    args = parser.parse_args()

    setup_ultralytics(args.work)
    classes = read_classes(args.source_project)

    project = args.work / "project"
    raw = project / "raw_images"
    raw.mkdir(parents=True, exist_ok=True)
    (project / "results" / "train").mkdir(parents=True, exist_ok=True)
    (project / "results" / "deploy").mkdir(parents=True, exist_ok=True)
    (project / "classes.txt").write_text("\n".join(classes) + "\n", encoding="utf-8")
    write_data_yaml(project, classes)

    log("copy old labeled samples")
    old_count = copy_old_labeled_samples(args.source_project, raw)
    log(f"old labeled samples: {old_count}")

    log("extract new video frames")
    frames = extract_video_frames(args.incoming, raw, args.frame_step, args.max_frames_per_video)
    log(f"new frames extracted: {len(frames)}")

    log("pseudo-label new frames with previous model")
    pseudo_count = autolabel_frames(
        frames=frames,
        model_path=args.old_model,
        class_count=len(classes),
        imgsz=args.pseudo_imgsz,
        conf=args.pseudo_conf,
        iou=0.45,
        device=args.device,
    )
    log(f"pseudo labeled complete frames: {pseudo_count}")

    log("make center zoom augmentations")
    zoom_count = center_zoom_augment(raw, args.zoom_augment_limit, seed=20260526)
    log(f"zoom augmented images: {zoom_count}")

    log("make synthetic random-layout samples")
    run_synthetic_generator(project, args.synthetic_count, args.python)

    log("prepare train/val split")
    train_count, val_count = prepare_dataset(project, args.val_ratio, seed=20260526)
    write_data_yaml(project, classes)
    log(f"dataset ready: train={train_count}, val={val_count}")

    best = project / "results" / "train" / "weights" / "best.pt"
    if not args.skip_train:
        log("train model")
        best = train_model(project, args.work, args.old_model, args.model_name, args.epochs, args.train_imgsz, args.device)
        log(f"best pt: {best}")

    if not args.skip_convert:
        log("convert to mud/axmodel")
        deploy = prepare_converter(args.work, project, classes, best, args.model_name, args.python, args.calib_count)
        log(f"deploy output: {deploy}")

    log("done")


if __name__ == "__main__":
    main()
