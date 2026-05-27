class StrongSnapGate:
    def __init__(self, gate_pixels=80, confirm_frames=3, confirm_pixels=36):
        self.gate_pixels = int(gate_pixels)
        self.confirm_frames = int(confirm_frames)
        self.confirm_pixels = int(confirm_pixels)
        self.reset()

    def reset(self):
        self.pending_cx = -1
        self.pending_cy = -1
        self.pending_count = 0
        self.last_reason = "reset"
        self.last_dist = -1

    def miss(self, reason="missing"):
        self.pending_cx = -1
        self.pending_cy = -1
        self.pending_count = 0
        self.last_reason = reason
        self.last_dist = -1

    def accept(self, meas_cx, meas_cy, pred_cx, pred_cy, ready):
        if not ready or pred_cx < 0 or pred_cy < 0:
            self.reset()
            self.last_reason = "open"
            self.last_dist = 0
            return True, self.last_reason

        dx = meas_cx - pred_cx
        dy = meas_cy - pred_cy
        dist2 = dx * dx + dy * dy
        self.last_dist = int(dist2 ** 0.5)
        if dist2 <= self.gate_pixels * self.gate_pixels:
            self.pending_cx = -1
            self.pending_cy = -1
            self.pending_count = 0
            self.last_reason = "snap"
            return True, self.last_reason

        if self.pending_cx >= 0 and self.pending_cy >= 0:
            pdx = meas_cx - self.pending_cx
            pdy = meas_cy - self.pending_cy
            if pdx * pdx + pdy * pdy <= self.confirm_pixels * self.confirm_pixels:
                self.pending_count += 1
            else:
                self.pending_count = 1
        else:
            self.pending_count = 1

        self.pending_cx = meas_cx
        self.pending_cy = meas_cy

        if self.pending_count >= self.confirm_frames:
            self.pending_count = 0
            self.last_reason = "confirm"
            return True, self.last_reason

        self.last_reason = "reject_far"
        return False, self.last_reason

    def debug_values(self):
        return self.last_reason, self.last_dist, self.pending_count
