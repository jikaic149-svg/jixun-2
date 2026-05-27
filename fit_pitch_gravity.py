import argparse
import math
import re
from pathlib import Path


GRAV_RE = re.compile(
    r"\bgenc=([-+]?\d+(?:\.\d+)?)\s+gcur=([-+]?\d+(?:\.\d+)?)\s+gp=(\d+)\s+gd=(\d+)"
)
LEGACY_RE = re.compile(r"enc=([-+]?\d+(?:\.\d+)?)\s+.*?\bip=([-+]?\d+(?:\.\d+)?)")


def solve_3x3(a, b):
    m = [a[0][:] + [b[0]], a[1][:] + [b[1]], a[2][:] + [b[2]]]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-9:
            raise ValueError("singular fit matrix")
        if pivot != col:
            m[col], m[pivot] = m[pivot], m[col]
        div = m[col][col]
        for c in range(col, 4):
            m[col][c] /= div
        for r in range(3):
            if r == col:
                continue
            factor = m[r][col]
            for c in range(col, 4):
                m[r][c] -= factor * m[col][c]
    return [m[0][3], m[1][3], m[2][3]]


def parse_samples(path):
    samples = []
    seen = set()
    for line in Path(path).read_text(encoding="utf-8", errors="ignore").splitlines():
        match = GRAV_RE.search(line)
        if match:
            enc = float(match.group(1))
            current = float(match.group(2))
            point = int(match.group(3))
            key = (point, round(enc, 3), round(current, 4))
            if key not in seen and abs(enc) > 0.001:
                samples.append((enc, current))
                seen.add(key)
            continue

        match = LEGACY_RE.search(line)
        if match:
            samples.append((float(match.group(1)), float(match.group(2))))
            continue

        parts = [p.strip() for p in line.split(",")]
        if len(parts) >= 2:
            try:
                samples.append((float(parts[0]), float(parts[1])))
            except ValueError:
                pass
    return samples


def fit(samples, min_deg, max_deg):
    rows = []
    y = []
    for enc_deg, current in samples:
        if enc_deg < min_deg or enc_deg > max_deg:
            continue
        rad = math.radians(enc_deg)
        rows.append((math.sin(rad), math.cos(rad), 1.0))
        y.append(current)

    if len(rows) < 6:
        raise ValueError("not enough samples in clamp range")

    ata = [[0.0, 0.0, 0.0] for _ in range(3)]
    aty = [0.0, 0.0, 0.0]
    for row, current in zip(rows, y):
        for i in range(3):
            aty[i] += row[i] * current
            for j in range(3):
                ata[i][j] += row[i] * row[j]

    coeff = solve_3x3(ata, aty)
    err2 = 0.0
    max_err = 0.0
    for row, current in zip(rows, y):
        pred = coeff[0] * row[0] + coeff[1] * row[1] + coeff[2]
        err = pred - current
        err2 += err * err
        max_err = max(max_err, abs(err))
    rms = math.sqrt(err2 / len(rows))
    return coeff, len(rows), rms, max_err


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="Maix log text, or CSV lines: enc,current")
    parser.add_argument("--min", type=float, default=19.0)
    parser.add_argument("--max", type=float, default=112.0)
    args = parser.parse_args()

    samples = parse_samples(args.input)
    coeff, count, rms, max_err = fit(samples, args.min, args.max)
    sin_k, cos_k, offset = coeff

    print("samples=%d range=%.2f..%.2f rms=%.6f max_err=%.6f" %
          (count, args.min, args.max, rms, max_err))
    print("#define PITCH_GRAVITY_FF_SIN_K %.7ff" % sin_k)
    print("#define PITCH_GRAVITY_FF_COS_K %.7ff" % cos_k)
    print("#define PITCH_GRAVITY_FF_OFFSET %.7ff" % offset)


if __name__ == "__main__":
    main()
