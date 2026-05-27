from core.geometry import clamp, center
from core.types import AimError


class AimController:
    def __init__(self, x, y, max_x, max_y):
        self.x = x
        self.y = y
        self.max_x = max_x
        self.max_y = max_y
        self.calibrated = False

    def point(self):
        return clamp(self.x, 0, self.max_x - 1), clamp(self.y, 0, self.max_y - 1)

    def error_to(self, obj):
        if obj is None:
            return AimError()
        cx, cy = center(obj)
        ax, ay = self.point()
        return AimError(cx - ax, cy - ay, True)
