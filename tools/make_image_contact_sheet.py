from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def main() -> None:
    parser = argparse.ArgumentParser(description="Make an unlabeled image contact sheet.")
    parser.add_argument("--image-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--cols", type=int, default=4)
    parser.add_argument("--thumb-w", type=int, default=320)
    parser.add_argument("--thumb-h", type=int, default=180)
    args = parser.parse_args()

    images = sorted(p for p in args.image_dir.iterdir() if p.is_file() and p.suffix.lower() in IMAGE_EXTS)
    thumbs = []
    for image in images:
        img = cv2.imread(str(image))
        if img is None:
            continue
        thumbs.append(cv2.resize(img, (args.thumb_w, args.thumb_h)))
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
