import argparse
import math
import random
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}
DEFAULT_PROJECT = Path(r"C:\Users\Administrator\Desktop\Yolo\projects\jixun2")


@dataclass
class Box:
    cls: int
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def w(self):
        return self.x2 - self.x1

    @property
    def h(self):
        return self.y2 - self.y1


@dataclass
class Template:
    cls: int
    img: np.ndarray
    mask: np.ndarray
    box: Box
    source: Path


@dataclass
class SeedImage:
    image_path: Path
    label_path: Path
    boxes: list


def read_classes(project: Path) -> list[str]:
    path = project / "classes.txt"
    if not path.exists():
        raise FileNotFoundError(f"missing classes.txt: {path}")
    classes = [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not classes:
        raise RuntimeError(f"empty classes.txt: {path}")
    return classes


def find_label(project: Path, image_path: Path) -> Path | None:
    candidates = [
        image_path.with_suffix(".txt"),
        project / "raw_labels" / f"{image_path.stem}.txt",
        project / "labels" / f"{image_path.stem}.txt",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def iter_raw_images(project: Path, input_subdir: str):
    root = project / input_subdir
    if not root.exists():
        return
    for path in sorted(root.rglob("*")):
        if path.is_file() and path.suffix.lower() in IMAGE_EXTS:
            yield path


def read_yolo_boxes(label_path: Path, width: int, height: int, class_count: int) -> list[Box]:
    boxes = []
    for line in label_path.read_text(encoding="utf-8").splitlines():
        parts = line.strip().split()
        if len(parts) < 5:
            continue
        cls = int(float(parts[0]))
        if cls < 0 or cls >= class_count:
            continue
        cx = float(parts[1]) * width
        cy = float(parts[2]) * height
        bw = float(parts[3]) * width
        bh = float(parts[4]) * height
        x1 = max(0.0, cx - bw * 0.5)
        y1 = max(0.0, cy - bh * 0.5)
        x2 = min(float(width - 1), cx + bw * 0.5)
        y2 = min(float(height - 1), cy + bh * 0.5)
        if x2 - x1 >= 3 and y2 - y1 >= 3:
            boxes.append(Box(cls, x1, y1, x2, y2))
    return boxes


def load_seed_images(project: Path, input_subdir: str, class_count: int) -> list[SeedImage]:
    seeds = []
    for image_path in iter_raw_images(project, input_subdir):
        label_path = find_label(project, image_path)
        if not label_path:
            continue
        img = cv2.imread(str(image_path))
        if img is None:
            continue
        h, w = img.shape[:2]
        boxes = read_yolo_boxes(label_path, w, h, class_count)
        if boxes:
            seeds.append(SeedImage(image_path, label_path, boxes))
    return seeds


def mask_bbox(mask: np.ndarray) -> tuple[int, int, int, int] | None:
    pts = cv2.findNonZero(mask)
    if pts is None:
        return None
    x, y, w, h = cv2.boundingRect(pts)
    if w < 3 or h < 3:
        return None
    return x, y, x + w, y + h


def segment_crop_foreground(crop: np.ndarray, rel: Box) -> np.ndarray:
    h, w = crop.shape[:2]
    mask = np.full((h, w), cv2.GC_PR_BGD, dtype=np.uint8)
    border = max(2, int(min(w, h) * 0.04))
    mask[:border, :] = cv2.GC_BGD
    mask[-border:, :] = cv2.GC_BGD
    mask[:, :border] = cv2.GC_BGD
    mask[:, -border:] = cv2.GC_BGD

    x1 = max(0, min(w - 1, int(rel.x1)))
    y1 = max(0, min(h - 1, int(rel.y1)))
    x2 = max(x1 + 1, min(w, int(rel.x2)))
    y2 = max(y1 + 1, min(h, int(rel.y2)))
    mask[y1:y2, x1:x2] = cv2.GC_PR_FGD

    core_pad_x = max(1, int((x2 - x1) * 0.18))
    core_pad_y = max(1, int((y2 - y1) * 0.18))
    cx1 = min(x2 - 1, x1 + core_pad_x)
    cy1 = min(y2 - 1, y1 + core_pad_y)
    cx2 = max(cx1 + 1, x2 - core_pad_x)
    cy2 = max(cy1 + 1, y2 - core_pad_y)
    mask[cy1:cy2, cx1:cx2] = cv2.GC_FGD

    try:
        bgd = np.zeros((1, 65), np.float64)
        fgd = np.zeros((1, 65), np.float64)
        cv2.grabCut(crop, mask, None, bgd, fgd, 3, cv2.GC_INIT_WITH_MASK)
        out = np.where((mask == cv2.GC_FGD) | (mask == cv2.GC_PR_FGD), 255, 0).astype(np.uint8)
    except Exception:
        out = np.zeros((h, w), dtype=np.uint8)
        out[y1:y2, x1:x2] = 255

    kernel = np.ones((3, 3), np.uint8)
    out = cv2.morphologyEx(out, cv2.MORPH_OPEN, kernel)
    out = cv2.morphologyEx(out, cv2.MORPH_CLOSE, kernel, iterations=2)

    bbox = mask_bbox(out)
    if bbox is None:
        out = np.zeros((h, w), dtype=np.uint8)
        out[y1:y2, x1:x2] = 255
    return out


def crop_templates(seeds: list[SeedImage], pad_ratio: float, max_per_class: int, rng: random.Random) -> list[Template]:
    templates = []
    per_class = {}
    shuffled = list(seeds)
    rng.shuffle(shuffled)
    for seed in shuffled:
        img = cv2.imread(str(seed.image_path))
        if img is None:
            continue
        h, w = img.shape[:2]
        for box in seed.boxes:
            if max_per_class > 0 and per_class.get(box.cls, 0) >= max_per_class:
                continue
            pad = int(max(box.w, box.h) * pad_ratio)
            x1 = max(0, int(box.x1) - pad)
            y1 = max(0, int(box.y1) - pad)
            x2 = min(w, int(box.x2) + pad)
            y2 = min(h, int(box.y2) + pad)
            crop = img[y1:y2, x1:x2].copy()
            if crop.size == 0:
                continue
            rel = Box(box.cls, box.x1 - x1, box.y1 - y1, box.x2 - x1, box.y2 - y1)
            mask = segment_crop_foreground(crop, rel)
            bbox = mask_bbox(mask)
            if bbox is not None:
                rel = Box(box.cls, float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3]))
            templates.append(Template(box.cls, crop, mask, rel, seed.image_path))
            per_class[box.cls] = per_class.get(box.cls, 0) + 1
    return templates


def union_roi(boxes: list[Box], width: int, height: int, pad_x: float, pad_y: float) -> tuple[int, int, int, int]:
    x1 = min(box.x1 for box in boxes)
    y1 = min(box.y1 for box in boxes)
    x2 = max(box.x2 for box in boxes)
    y2 = max(box.y2 for box in boxes)
    bw = x2 - x1
    bh = y2 - y1
    x1 -= bw * pad_x
    x2 += bw * pad_x
    y1 -= max(bh * pad_y, 40)
    y2 += max(bh * pad_y, 40)
    return (
        max(0, int(x1)),
        max(0, int(y1)),
        min(width - 1, int(x2)),
        min(height - 1, int(y2)),
    )


def fill_labeled_objects(img: np.ndarray, boxes: list[Box], roi: tuple[int, int, int, int]) -> np.ndarray:
    out = img.copy()
    rx1, ry1, rx2, ry2 = roi
    mask = np.zeros(out.shape[:2], dtype=np.uint8)
    for box in boxes:
        pad = int(max(box.w, box.h) * 0.45)
        x1 = max(0, int(box.x1) - pad)
        y1 = max(0, int(box.y1) - pad)
        x2 = min(out.shape[1], int(box.x2) + pad)
        y2 = min(out.shape[0], int(box.y2) + pad)
        mask[y1:y2, x1:x2] = 255

    mask[:ry1, :] = 0
    mask[ry2:, :] = 0
    mask[:, :rx1] = 0
    mask[:, rx2:] = 0
    if cv2.countNonZero(mask) <= 0:
        return out

    out = cv2.inpaint(out, mask, 5, cv2.INPAINT_TELEA)
    return cv2.GaussianBlur(out, (3, 3), 0)


def rect_iou(a, b) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    iw = max(0, ix2 - ix1)
    ih = max(0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    area_a = max(1, (ax2 - ax1) * (ay2 - ay1))
    area_b = max(1, (bx2 - bx1) * (by2 - by1))
    return inter / float(area_a + area_b - inter)


def feather_alpha(h: int, w: int, feather: int) -> np.ndarray:
    if feather <= 0:
        return np.ones((h, w, 1), dtype=np.float32)
    yy, xx = np.mgrid[0:h, 0:w]
    dist = np.minimum.reduce([xx, yy, w - 1 - xx, h - 1 - yy]).astype(np.float32)
    alpha = np.clip(dist / float(feather), 0.0, 1.0)
    return alpha[:, :, None]


def paste_patch(dst: np.ndarray, patch: np.ndarray, x: int, y: int, feather: int) -> None:
    h, w = patch.shape[:2]
    roi = dst[y:y + h, x:x + w]
    alpha = feather_alpha(h, w, feather)
    blended = roi.astype(np.float32) * (1.0 - alpha) + patch.astype(np.float32) * alpha
    dst[y:y + h, x:x + w] = np.clip(blended, 0, 255).astype(np.uint8)


def paste_masked_patch(dst: np.ndarray, patch: np.ndarray, mask: np.ndarray, x: int, y: int, feather: int) -> None:
    h, w = patch.shape[:2]
    roi = dst[y:y + h, x:x + w]
    if roi.shape[:2] != patch.shape[:2]:
        return
    alpha = mask.astype(np.float32) / 255.0
    if feather > 0:
        k = max(3, feather * 2 + 1)
        if k % 2 == 0:
            k += 1
        alpha = cv2.GaussianBlur(alpha, (k, k), 0)
    alpha = np.clip(alpha, 0.0, 1.0)[:, :, None]
    blended = roi.astype(np.float32) * (1.0 - alpha) + patch.astype(np.float32) * alpha
    dst[y:y + h, x:x + w] = np.clip(blended, 0, 255).astype(np.uint8)


def box_from_mask(mask: np.ndarray, cls: int) -> Box | None:
    bbox = mask_bbox(mask)
    if bbox is None:
        return None
    return Box(cls, float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3]))


def scale_template(template: Template, scale: float, rng: random.Random) -> tuple[np.ndarray, np.ndarray, Box]:
    patch = template.img
    mask = template.mask
    ph, pw = patch.shape[:2]
    nw = max(6, int(pw * scale))
    nh = max(6, int(ph * scale))
    interp = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(patch, (nw, nh), interpolation=interp)
    resized_mask = cv2.resize(mask, (nw, nh), interpolation=cv2.INTER_NEAREST)
    if rng.random() < 0.25:
        angle = rng.uniform(-4.0, 4.0)
        center = (nw * 0.5, nh * 0.5)
        mat = cv2.getRotationMatrix2D(center, angle, 1.0)
        resized = cv2.warpAffine(resized, mat, (nw, nh), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        resized_mask = cv2.warpAffine(resized_mask, mat, (nw, nh), flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    box = box_from_mask(resized_mask, template.cls)
    if box is None:
        box = Box(
            template.cls,
            template.box.x1 * scale,
            template.box.y1 * scale,
            template.box.x2 * scale,
            template.box.y2 * scale,
        )
    return resized, resized_mask, box


def choose_templates_by_class(templates: list[Template], count: int, rng: random.Random) -> list[Template]:
    by_class = {}
    for item in templates:
        by_class.setdefault(item.cls, []).append(item)
    classes = list(by_class.keys())
    rng.shuffle(classes)
    chosen = []
    while len(chosen) < count:
        if not classes:
            break
        cls = classes[len(chosen) % len(classes)]
        chosen.append(rng.choice(by_class[cls]))
    rng.shuffle(chosen)
    return chosen[:count]


def place_random(
    base: np.ndarray,
    templates: list[Template],
    roi: tuple[int, int, int, int],
    args,
    rng: random.Random,
) -> list[Box]:
    rx1, ry1, rx2, ry2 = roi
    placed_rects = []
    labels = []

    for template in templates:
        for _ in range(120):
            scale = rng.uniform(args.scale_min, args.scale_max)
            patch, mask, rel_box = scale_template(template, scale, rng)
            ph, pw = patch.shape[:2]
            if pw >= rx2 - rx1 or ph >= ry2 - ry1:
                continue
            x = rng.randint(rx1, max(rx1, rx2 - pw))
            y = rng.randint(ry1, max(ry1, ry2 - ph))
            rect = (
                int(x + rel_box.x1),
                int(y + rel_box.y1),
                int(x + rel_box.x2),
                int(y + rel_box.y2),
            )
            if all(rect_iou(rect, old) <= args.max_iou for old in placed_rects):
                paste_masked_patch(base, patch, mask, x, y, args.feather)
                placed_rects.append(rect)
                labels.append(Box(template.cls, x + rel_box.x1, y + rel_box.y1, x + rel_box.x2, y + rel_box.y2))
                break
    return labels


def place_row(
    base: np.ndarray,
    templates: list[Template],
    roi: tuple[int, int, int, int],
    args,
    rng: random.Random,
) -> list[Box]:
    rx1, ry1, rx2, ry2 = roi
    labels = []
    if not templates:
        return labels
    slots = len(templates)
    xs = sorted(rng.sample(range(rx1, max(rx1 + 1, rx2)), k=min(slots, max(1, rx2 - rx1))))
    if len(xs) < slots:
        step = max(1, (rx2 - rx1) // max(1, slots))
        xs = [rx1 + i * step for i in range(slots)]
    baseline = rng.randint(ry1, max(ry1, ry2))
    placed_rects = []
    for idx, template in enumerate(templates):
        for _ in range(80):
            scale = rng.uniform(args.scale_min, args.scale_max)
            patch, mask, rel_box = scale_template(template, scale, rng)
            ph, pw = patch.shape[:2]
            x = int(xs[idx] - pw * 0.5 + rng.uniform(-args.row_jitter, args.row_jitter))
            y = int(baseline - ph * 0.5 + rng.uniform(-args.row_jitter, args.row_jitter))
            x = max(rx1, min(rx2 - pw, x))
            y = max(ry1, min(ry2 - ph, y))
            rect = (
                int(x + rel_box.x1),
                int(y + rel_box.y1),
                int(x + rel_box.x2),
                int(y + rel_box.y2),
            )
            if pw <= 0 or ph <= 0:
                continue
            if all(rect_iou(rect, old) <= args.max_iou for old in placed_rects):
                paste_masked_patch(base, patch, mask, x, y, args.feather)
                placed_rects.append(rect)
                labels.append(Box(template.cls, x + rel_box.x1, y + rel_box.y1, x + rel_box.x2, y + rel_box.y2))
                break
    return labels


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
    alpha = rng.uniform(0.88, 1.16)
    beta = rng.uniform(-14, 14)
    out = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)
    if rng.random() < 0.35:
        noise = rng.normalvariate(0, 1)
        sigma = abs(noise) * 3.0 + 1.0
        gauss = np.random.normal(0, sigma, out.shape).astype(np.float32)
        out = np.clip(out.astype(np.float32) + gauss, 0, 255).astype(np.uint8)
    return out


def draw_laser_spot(img: np.ndarray, boxes: list[Box], rng: random.Random) -> np.ndarray:
    if not boxes:
        return img
    out = img.copy()
    h, w = out.shape[:2]
    box = rng.choice(boxes)
    cx = int((box.x1 + box.x2) * 0.5 + rng.uniform(-0.22, 0.22) * max(1.0, box.w))
    cy = int((box.y1 + box.y2) * 0.5 + rng.uniform(-0.22, 0.22) * max(1.0, box.h))
    cx = max(0, min(w - 1, cx))
    cy = max(0, min(h - 1, cy))
    r = rng.randint(2, 5)
    cv2.circle(out, (cx, cy), r + 5, (215, 70, 35), -1, lineType=cv2.LINE_AA)
    cv2.circle(out, (cx, cy), r + 2, (255, 135, 65), -1, lineType=cv2.LINE_AA)
    cv2.circle(out, (cx, cy), max(1, r // 2), (255, 255, 255), -1, lineType=cv2.LINE_AA)
    return out


def write_yolo_label(path: Path, boxes: list[Box], width: int, height: int) -> None:
    lines = []
    for box in boxes:
        x1 = max(0.0, min(float(width - 1), box.x1))
        y1 = max(0.0, min(float(height - 1), box.y1))
        x2 = max(0.0, min(float(width - 1), box.x2))
        y2 = max(0.0, min(float(height - 1), box.y2))
        bw = max(1.0, x2 - x1)
        bh = max(1.0, y2 - y1)
        cx = x1 + bw * 0.5
        cy = y1 + bh * 0.5
        lines.append(f"{box.cls} {cx / width:.6f} {cy / height:.6f} {bw / width:.6f} {bh / height:.6f}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def draw_preview(img: np.ndarray, boxes: list[Box], classes: list[str]) -> np.ndarray:
    out = img.copy()
    for box in boxes:
        x1, y1, x2, y2 = int(box.x1), int(box.y1), int(box.x2), int(box.y2)
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 255), 2)
        label = classes[box.cls] if 0 <= box.cls < len(classes) else str(box.cls)
        cv2.putText(out, label, (x1, max(20, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description="Build synthetic random-layout YOLO samples for jixun2.")
    parser.add_argument("--project", type=Path, default=DEFAULT_PROJECT)
    parser.add_argument("--input-subdir", default="raw_images")
    parser.add_argument("--output-subdir", default="raw_images/synth_random")
    parser.add_argument("--count", type=int, default=800)
    parser.add_argument("--seed", type=int, default=20260526)
    parser.add_argument("--min-objects", type=int, default=4)
    parser.add_argument("--max-objects", type=int, default=8)
    parser.add_argument("--template-pad", type=float, default=0.35)
    parser.add_argument("--max-seeds", type=int, default=0)
    parser.add_argument("--max-templates-per-class", type=int, default=0)
    parser.add_argument("--scale-min", type=float, default=0.75)
    parser.add_argument("--scale-max", type=float, default=1.70)
    parser.add_argument("--roi-pad-x", type=float, default=1.40)
    parser.add_argument("--roi-pad-y", type=float, default=2.50)
    parser.add_argument("--row-layout-prob", type=float, default=0.30)
    parser.add_argument("--row-jitter", type=int, default=24)
    parser.add_argument("--max-iou", type=float, default=0.03)
    parser.add_argument("--motion-blur-prob", type=float, default=0.55)
    parser.add_argument("--motion-blur-max", type=int, default=13)
    parser.add_argument("--laser-prob", type=float, default=0.45)
    parser.add_argument("--feather", type=int, default=5)
    parser.add_argument("--preview-every", type=int, default=25)
    args = parser.parse_args()

    project = args.project
    classes = read_classes(project)
    rng = random.Random(args.seed)
    np.random.seed(args.seed & 0xFFFF)

    seeds = load_seed_images(project, args.input_subdir, len(classes))
    if not seeds:
        raise RuntimeError("no labeled seed images found; label a small seed set first")
    if args.max_seeds > 0:
        rng.shuffle(seeds)
        seeds = seeds[:args.max_seeds]

    templates = crop_templates(seeds, args.template_pad, args.max_templates_per_class, rng)
    if not templates:
        raise RuntimeError("no object templates could be cropped from labels")

    out_dir = project / args.output_subdir
    preview_dir = out_dir / "_preview"
    out_dir.mkdir(parents=True, exist_ok=True)
    preview_dir.mkdir(parents=True, exist_ok=True)

    print("Project:", project)
    print("Seed images:", len(seeds))
    print("Templates:", len(templates))
    print("Output:", out_dir)

    made = 0
    attempts = 0
    while made < args.count and attempts < args.count * 5:
        attempts += 1
        seed = rng.choice(seeds)
        bg = cv2.imread(str(seed.image_path))
        if bg is None:
            continue
        h, w = bg.shape[:2]
        roi = union_roi(seed.boxes, w, h, args.roi_pad_x, args.roi_pad_y)
        base = fill_labeled_objects(bg, seed.boxes, roi)

        n = rng.randint(args.min_objects, args.max_objects)
        chosen = choose_templates_by_class(templates, n, rng)
        if rng.random() < args.row_layout_prob:
            labels = place_row(base, chosen, roi, args, rng)
        else:
            labels = place_random(base, chosen, roi, args, rng)

        if len(labels) < max(1, args.min_objects // 2):
            continue

        base = color_jitter(base, rng)
        if rng.random() < args.laser_prob:
            base = draw_laser_spot(base, labels, rng)
        if rng.random() < args.motion_blur_prob:
            base = apply_motion_blur(base, rng, args.motion_blur_max)

        stem = f"synth_random_{made:06d}"
        image_path = out_dir / f"{stem}.jpg"
        label_path = out_dir / f"{stem}.txt"
        cv2.imwrite(str(image_path), base, [int(cv2.IMWRITE_JPEG_QUALITY), rng.randint(86, 96)])
        write_yolo_label(label_path, labels, w, h)

        if args.preview_every > 0 and made % args.preview_every == 0:
            preview = draw_preview(base, labels, classes)
            cv2.imwrite(str(preview_dir / f"{stem}_preview.jpg"), preview)

        made += 1
        if made % 50 == 0:
            print(f"made={made}/{args.count}")

    print("Done.")
    print("Synthetic images:", made)
    print("Preview:", preview_dir)
    if made < args.count:
        print("Warning: generated fewer images than requested; add more seed labels or reduce placement constraints.")


if __name__ == "__main__":
    main()
