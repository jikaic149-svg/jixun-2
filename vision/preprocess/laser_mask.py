from core.geometry import clamp, center, dist2


class LaserMask:
    def __init__(self, input_radius=18, reject_radius=34, max_box_size=42):
        self.input_radius = input_radius
        self.reject_radius = reject_radius
        self.max_box_size = max_box_size

    def mask_input(self, img, aim_point, max_x, max_y, color):
        ax, ay = aim_point
        r = self.input_radius
        x1 = clamp(ax - r, 0, max_x - 1)
        y1 = clamp(ay - r, 0, max_y - 1)
        x2 = clamp(ax + r + 1, 0, max_x)
        y2 = clamp(ay + r + 1, 0, max_y)
        if x2 <= x1 or y2 <= y1:
            return img
        try:
            out = img.copy()
        except Exception:
            out = img
        out.draw_rect(x1, y1, x2 - x1, y2 - y1, color=color, thickness=-1)
        return out

    def is_laser_box(self, obj, aim_point):
        if obj.w > self.max_box_size or obj.h > self.max_box_size:
            return False
        cx, cy = center(obj)
        ax, ay = aim_point
        return dist2(cx, cy, ax, ay) <= self.reject_radius * self.reject_radius
