class TargetCenterResult:
    def __init__(self, valid=False, cx=-1, cy=-1, roi=None, reason="missing"):
        self.valid = valid
        self.cx = int(cx)
        self.cy = int(cy)
        self.roi = roi
        self.reason = reason


class TargetCenterFinder:
    def __init__(self, thresholds):
        self.thresholds = thresholds
        self.last = TargetCenterResult()

    def _clamp(self, value, min_value, max_value):
        if value < min_value:
            return min_value
        if value > max_value:
            return max_value
        return value

    def _expanded_roi(self, obj, panel_w, panel_h):
        pad_x = max(8, obj.w // 5)
        pad_y = max(8, obj.h // 5)
        x1 = self._clamp(obj.x - pad_x, 0, panel_w - 1)
        y1 = self._clamp(obj.y - pad_y, 0, panel_h - 1)
        x2 = self._clamp(obj.x + obj.w + pad_x, 0, panel_w)
        y2 = self._clamp(obj.y + obj.h + pad_y, 0, panel_h)
        if x2 <= x1 or y2 <= y1:
            return None
        return [x1, y1, x2 - x1, y2 - y1]

    def update(self, img, obj, panel_w, panel_h):
        if obj is None:
            self.last = TargetCenterResult(reason="no target")
            return self.last

        roi = self._expanded_roi(obj, panel_w, panel_h)
        if roi is None:
            self.last = TargetCenterResult(reason="bad roi")
            return self.last

        try:
            blobs = img.find_blobs(
                self.thresholds,
                roi=roi,
                pixels_threshold=18,
                area_threshold=18,
                merge=True,
                margin=3,
            )
        except Exception as e:
            self.last = TargetCenterResult(roi=roi, reason="blob err")
            return self.last

        if len(blobs) == 0:
            self.last = TargetCenterResult(roi=roi, reason="no black")
            return self.last

        min_x = panel_w
        min_y = panel_h
        max_x = 0
        max_y = 0
        pixels = 0
        for blob in blobs:
            bx = blob[0]
            by = blob[1]
            bw = blob[2]
            bh = blob[3]
            bp = blob[4]
            if bw < 3 or bh < 3:
                continue
            min_x = min(min_x, bx)
            min_y = min(min_y, by)
            max_x = max(max_x, bx + bw)
            max_y = max(max_y, by + bh)
            pixels += bp

        if pixels < 30 or max_x <= min_x or max_y <= min_y:
            self.last = TargetCenterResult(roi=roi, reason="weak black")
            return self.last

        w = max_x - min_x
        h = max_y - min_y
        if w < obj.w * 0.35 or h < obj.h * 0.35:
            self.last = TargetCenterResult(roi=roi, reason="small ring")
            return self.last
        ratio = float(w) / float(h)
        if ratio < 0.65 or ratio > 1.55:
            self.last = TargetCenterResult(roi=roi, reason="bad ratio")
            return self.last

        cx = (min_x + max_x) // 2
        cy = (min_y + max_y) // 2
        self.last = TargetCenterResult(True, cx, cy, roi, "ok")
        return self.last
