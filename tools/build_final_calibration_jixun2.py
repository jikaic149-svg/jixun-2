from __future__ import annotations

import argparse
import csv
import shutil
from pathlib import Path


YOLO_ROOT = Path(r"C:\Users\Administrator\Desktop\Yolo")
DEFAULT_FORMAL = YOLO_ROOT / "projects" / "jixun2"
DEFAULT_AUTO = Path(r"C:\Users\Administrator\Desktop\jixun2\.yolo_auto\project")
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def read_classes(project: Path) -> list[str]:
    path = project / "classes.txt"
    classes = [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not classes:
        raise RuntimeError(f"empty classes: {path}")
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


def label_for_image(image: Path) -> Path:
    return image.with_suffix(".txt")


def reset_project(project: Path, classes: list[str]) -> None:
    if project.exists():
        shutil.rmtree(project)
    for sub in ["raw_images", "images/train", "images/val", "labels/train", "labels/val", "results/train", "results/deploy", "tools"]:
        (project / sub).mkdir(parents=True, exist_ok=True)
    (project / "classes.txt").write_text("\n".join(classes) + "\n", encoding="utf-8")
    write_data_yaml(project, classes)


def copy_pair(image: Path, label: Path, out_raw: Path, prefix: str) -> tuple[str, str] | None:
    if not image.exists() or not label.exists():
        return None
    stem = f"{prefix}_{image.stem}".replace(" ", "_")
    out_image = out_raw / f"{stem}{image.suffix.lower()}"
    out_label = out_raw / f"{stem}.txt"
    if out_image.exists() or out_label.exists():
        return None
    shutil.copy2(image, out_image)
    shutil.copy2(label, out_label)
    return out_image.name, out_label.name


def copy_formal_raw(source: Path, out_raw: Path, rows: list[list[str]]) -> int:
    count = 0
    for image in sorted((source / "raw_images").rglob("*")):
        if not image.is_file() or image.suffix.lower() not in IMAGE_EXTS:
            continue
        label = label_for_image(image)
        if not label.exists():
            continue
        copied = copy_pair(image, label, out_raw, "formal")
        if copied:
            rows.append([copied[0], copied[1], "formal_raw", str(image)])
            count += 1
    return count


def copy_split(source: Path, out_raw: Path, rows: list[list[str]], prefix: str) -> int:
    count = 0
    for split in ["train", "val"]:
        image_dir = source / "images" / split
        label_dir = source / "labels" / split
        if not image_dir.exists() or not label_dir.exists():
            continue
        for image in sorted(image_dir.iterdir()):
            if not image.is_file() or image.suffix.lower() not in IMAGE_EXTS:
                continue
            label = label_dir / f"{image.stem}.txt"
            if not label.exists():
                continue
            copied = copy_pair(image, label, out_raw, f"{prefix}_{split}")
            if copied:
                rows.append([copied[0], copied[1], f"{prefix}_{split}", str(image)])
                count += 1
    return count


def main() -> None:
    parser = argparse.ArgumentParser(description="Merge formal and auto jixun2 labels into one final calibration project.")
    parser.add_argument("--formal-project", type=Path, default=DEFAULT_FORMAL)
    parser.add_argument("--auto-project", type=Path, default=DEFAULT_AUTO)
    parser.add_argument("--out-project", type=Path, default=YOLO_ROOT / "projects" / "jixun2_final_calib")
    args = parser.parse_args()

    if not args.formal_project.exists():
        raise FileNotFoundError(args.formal_project)
    if not args.auto_project.exists():
        raise FileNotFoundError(args.auto_project)

    classes = read_classes(args.formal_project)
    reset_project(args.out_project, classes)
    out_raw = args.out_project / "raw_images"

    rows: list[list[str]] = []
    formal_count = copy_formal_raw(args.formal_project, out_raw, rows)
    auto_count = copy_split(args.auto_project, out_raw, rows, "auto")

    manifest = args.out_project / "final_calibration_manifest.csv"
    with manifest.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["image", "label", "source", "source_path"])
        writer.writerows(rows)

    print(f"out_project={args.out_project}")
    print(f"formal_raw={formal_count}")
    print(f"auto_split={auto_count}")
    print(f"total={formal_count + auto_count}")
    print(f"manifest={manifest}")


if __name__ == "__main__":
    main()
