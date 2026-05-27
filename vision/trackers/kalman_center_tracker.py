class KalmanCenterTracker:
    def __init__(self, process_var=18.0, measure_var=160.0,
                 max_speed=55.0, max_predict_frames=6,
                 frame_width=-1, frame_height=-1, miss_velocity_decay=0.35):
        self.process_var = float(process_var)
        self.measure_var = float(measure_var)
        self.max_speed = float(max_speed)
        self.max_predict_frames = int(max_predict_frames)
        self.frame_width = int(frame_width)
        self.frame_height = int(frame_height)
        self.miss_velocity_decay = float(miss_velocity_decay)
        self.reset()

    def reset(self):
        self.ready = False
        self.x = [0.0, 0.0, 0.0, 0.0]
        self.p = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.miss_frames = 0
        self.last_pred_cx = -1
        self.last_pred_cy = -1
        self.last_meas_cx = -1
        self.last_meas_cy = -1
        self.last_out_cx = -1
        self.last_out_cy = -1
        self.last_innov_x = 0.0
        self.last_innov_y = 0.0
        self.last_source = "reset"

    def init(self, cx, cy):
        self.ready = True
        self.x = [float(cx), float(cy), 0.0, 0.0]
        self.p = [
            [80.0, 0.0, 0.0, 0.0],
            [0.0, 80.0, 0.0, 0.0],
            [0.0, 0.0, 120.0, 0.0],
            [0.0, 0.0, 0.0, 120.0],
        ]
        self.miss_frames = 0
        self.last_pred_cx = int(cx)
        self.last_pred_cy = int(cy)
        self.last_meas_cx = int(cx)
        self.last_meas_cy = int(cy)
        self.last_out_cx = int(cx)
        self.last_out_cy = int(cy)
        self.last_innov_x = 0.0
        self.last_innov_y = 0.0
        self.last_source = "init"

    def _limit_velocity(self):
        if self.x[2] > self.max_speed:
            self.x[2] = self.max_speed
        elif self.x[2] < -self.max_speed:
            self.x[2] = -self.max_speed
        if self.x[3] > self.max_speed:
            self.x[3] = self.max_speed
        elif self.x[3] < -self.max_speed:
            self.x[3] = -self.max_speed

    def _clamp_position(self):
        if self.frame_width > 0:
            if self.x[0] < 0:
                self.x[0] = 0.0
                self.x[2] = 0.0
            elif self.x[0] > self.frame_width - 1:
                self.x[0] = float(self.frame_width - 1)
                self.x[2] = 0.0
        if self.frame_height > 0:
            if self.x[1] < 0:
                self.x[1] = 0.0
                self.x[3] = 0.0
            elif self.x[1] > self.frame_height - 1:
                self.x[1] = float(self.frame_height - 1)
                self.x[3] = 0.0

    def _freeze_to_last_measurement(self):
        if self.last_meas_cx >= 0 and self.last_meas_cy >= 0:
            self.x[0] = float(self.last_meas_cx)
            self.x[1] = float(self.last_meas_cy)
        self.x[2] = 0.0
        self.x[3] = 0.0
        self._clamp_position()
        self.last_pred_cx = int(self.x[0])
        self.last_pred_cy = int(self.x[1])
        self.last_out_cx = self.last_pred_cx
        self.last_out_cy = self.last_pred_cy

    def predict(self):
        if not self.ready:
            return None

        if self.miss_frames > self.max_predict_frames:
            self._freeze_to_last_measurement()
            self.last_source = "drop"
            return int(self.x[0]), int(self.x[1])

        if self.miss_frames > 0:
            self.x[2] *= self.miss_velocity_decay
            self.x[3] *= self.miss_velocity_decay

        x0 = self.x[0] + self.x[2]
        x1 = self.x[1] + self.x[3]
        x2 = self.x[2]
        x3 = self.x[3]
        self.x = [x0, x1, x2, x3]

        p = self.p
        q = self.process_var
        self.p = [
            [p[0][0] + p[2][0] + p[0][2] + p[2][2] + q, p[0][1] + p[2][1] + p[0][3] + p[2][3], p[0][2] + p[2][2], p[0][3] + p[2][3]],
            [p[1][0] + p[3][0] + p[1][2] + p[3][2], p[1][1] + p[3][1] + p[1][3] + p[3][3] + q, p[1][2] + p[3][2], p[1][3] + p[3][3]],
            [p[2][0] + p[2][2], p[2][1] + p[2][3], p[2][2] + q, p[2][3]],
            [p[3][0] + p[3][2], p[3][1] + p[3][3], p[3][2], p[3][3] + q],
        ]
        self._limit_velocity()
        self._clamp_position()
        self.last_pred_cx = int(self.x[0])
        self.last_pred_cy = int(self.x[1])
        self.last_out_cx = self.last_pred_cx
        self.last_out_cy = self.last_pred_cy
        self.last_source = "predict"
        return int(self.x[0]), int(self.x[1])

    def update(self, cx, cy):
        if not self.ready:
            self.init(cx, cy)
            return int(cx), int(cy)

        z0 = float(cx)
        z1 = float(cy)
        r = self.measure_var
        p = self.p
        y0 = z0 - self.x[0]
        y1 = z1 - self.x[1]
        self.last_meas_cx = int(cx)
        self.last_meas_cy = int(cy)
        self.last_innov_x = y0
        self.last_innov_y = y1
        s00 = p[0][0] + r
        s01 = p[0][1]
        s10 = p[1][0]
        s11 = p[1][1] + r
        det = s00 * s11 - s01 * s10
        if det == 0.0:
            self.miss_frames = 0
            return int(self.x[0]), int(self.x[1])

        inv00 = s11 / det
        inv01 = -s01 / det
        inv10 = -s10 / det
        inv11 = s00 / det
        k = []
        for row in range(4):
            k0 = p[row][0] * inv00 + p[row][1] * inv10
            k1 = p[row][0] * inv01 + p[row][1] * inv11
            k.append((k0, k1))

        for row in range(4):
            self.x[row] += k[row][0] * y0 + k[row][1] * y1

        old_p = self.p
        new_p = []
        for row in range(4):
            new_row = []
            for col in range(4):
                new_row.append(old_p[row][col] - k[row][0] * old_p[0][col] - k[row][1] * old_p[1][col])
            new_p.append(new_row)
        self.p = new_p
        self._limit_velocity()
        self._clamp_position()
        self.miss_frames = 0
        self.last_out_cx = int(self.x[0])
        self.last_out_cy = int(self.x[1])
        self.last_source = "update"
        return int(self.x[0]), int(self.x[1])

    def miss(self):
        if not self.ready:
            return None
        self.miss_frames += 1
        if self.miss_frames > self.max_predict_frames:
            self._freeze_to_last_measurement()
            self.last_source = "drop"
            return None
        self.last_source = "miss"
        return int(self.x[0]), int(self.x[1])

    def debug_values(self):
        return (
            1 if self.ready else 0,
            self.last_source,
            self.last_pred_cx,
            self.last_pred_cy,
            self.last_meas_cx,
            self.last_meas_cy,
            self.last_out_cx,
            self.last_out_cy,
            self.x[2],
            self.x[3],
            self.last_innov_x,
            self.last_innov_y,
            self.miss_frames,
        )
