from core.geometry import clamp


class MotionPredictor:
    def __init__(self, px_per_deg_yaw=18.0, px_per_deg_pitch=18.0):
        self.px_per_deg_yaw = px_per_deg_yaw
        self.px_per_deg_pitch = px_per_deg_pitch

    def predict_center(self, cx, cy, dyaw_deg, dpitch_deg, max_w, max_h):
        x = int(cx + dyaw_deg * self.px_per_deg_yaw)
        y = int(cy + dpitch_deg * self.px_per_deg_pitch)
        return clamp(x, 0, max_w - 1), clamp(y, 0, max_h - 1)
