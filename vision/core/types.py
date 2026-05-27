class Detection:
    def __init__(self, x, y, w, h, class_id, score, source="unknown"):
        self.x = int(x)
        self.y = int(y)
        self.w = int(w)
        self.h = int(h)
        self.class_id = int(class_id)
        self.score = float(score)
        self.source = source

    @property
    def cx(self):
        return self.x + self.w // 2

    @property
    def cy(self):
        return self.y + self.h // 2


class GimbalState:
    def __init__(self):
        self.time_ms = 0
        self.pitch_abs = 0.0
        self.yaw_abs = 0.0
        self.dx = 0
        self.dy = 0
        self.state = 0
        self.valid = False


class TrackResult:
    def __init__(self, target=None, valid=False, source="none", reason=""):
        self.target = target
        self.valid = valid
        self.source = source
        self.reason = reason


class AimError:
    def __init__(self, dx=0, dy=0, valid=False):
        self.dx = int(dx)
        self.dy = int(dy)
        self.valid = valid
