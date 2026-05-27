from core.types import TrackResult


def _clamp(value, min_value, max_value):
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value


def _pixel_rgb(pixel):
    if isinstance(pixel, tuple) or isinstance(pixel, list):
        if len(pixel) >= 3:
            return int(pixel[0]), int(pixel[1]), int(pixel[2])
        if len(pixel) == 1:
            v = int(pixel[0])
            return v, v, v
        return None
    r = (int(pixel) >> 16) & 0xff
    g = (int(pixel) >> 8) & 0xff
    b = int(pixel) & 0xff
    return r, g, b


def _rgb_to_lab_like(r, g, b):
    max_c = max(r, g, b)
    min_c = min(r, g, b)
    l = int((max_c + min_c) * 100 / 510)
    a = int((r - g) * 128 / 255)
    bb = int((b - g) * 128 / 255)
    return l, a, bb


class DynamicColorModel:
    def __init__(self, roi_scale=0.45, min_size=12,
                 l_margin=24, a_margin=18, b_margin=18):
        self.roi_scale = roi_scale
        self.min_size = min_size
        self.l_margin = l_margin
        self.a_margin = a_margin
        self.b_margin = b_margin
        self.thresholds = None

    def clear(self):
        self.thresholds = None

    def learn(self, img, obj, max_x, max_y):
        if obj is None:
            self.clear()
            return None

        sample_w = max(self.min_size, int(obj.w * self.roi_scale))
        sample_h = max(self.min_size, int(obj.h * self.roi_scale))
        cx = obj.x + obj.w // 2
        cy = obj.y + obj.h // 2
        x1 = _clamp(cx - sample_w // 2, 0, max_x - 1)
        y1 = _clamp(cy - sample_h // 2, 0, max_y - 1)
        x2 = _clamp(x1 + sample_w, 0, max_x)
        y2 = _clamp(y1 + sample_h, 0, max_y)

        count = 0
        sum_l = 0
        sum_a = 0
        sum_b = 0
        step_x = max((x2 - x1) // 6, 1)
        step_y = max((y2 - y1) // 6, 1)
        for yy in range(y1, y2, step_y):
            for xx in range(x1, x2, step_x):
                rgb = _pixel_rgb(img.get_pixel(xx, yy))
                if rgb is None:
                    continue
                r, g, b = rgb
                l, a, bb = _rgb_to_lab_like(r, g, b)
                sum_l += l
                sum_a += a
                sum_b += bb
                count += 1

        if count <= 0:
            self.clear()
            return None

        l = sum_l // count
        a = sum_a // count
        bb = sum_b // count
        self.thresholds = [[
            _clamp(l - self.l_margin, 0, 100),
            _clamp(l + self.l_margin, 0, 100),
            _clamp(a - self.a_margin, -128, 127),
            _clamp(a + self.a_margin, -128, 127),
            _clamp(bb - self.b_margin, -128, 127),
            _clamp(bb + self.b_margin, -128, 127),
        ]]
        return self.thresholds


class RoiColorTracker:
    def update(self, frame, roi, thresholds):
        if roi is None or thresholds is None:
            return TrackResult(reason="missing roi or thresholds")
        blobs = frame.find_blobs(thresholds, roi=roi, merge=True)
        if not blobs:
            return TrackResult(reason="no blob")
        blob = max(blobs, key=lambda b: b[4])
        return TrackResult(target=blob, valid=True, source="roi_color")
