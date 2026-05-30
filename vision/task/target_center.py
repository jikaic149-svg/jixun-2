import math


class CircleKalmanTracker:
    def __init__(self, process_var=160.0, radius_process_var=70.0,
                 measure_var=0.9, radius_measure_var=0.7,
                 max_speed=120.0, max_radius_speed=28.0,
                 max_predict_frames=4, miss_velocity_decay=0.25):
        self.process_var = float(process_var)
        self.radius_process_var = float(radius_process_var)
        self.measure_var = float(measure_var)
        self.radius_measure_var = float(radius_measure_var)
        self.max_speed = float(max_speed)
        self.max_radius_speed = float(max_radius_speed)
        self.max_predict_frames = int(max_predict_frames)
        self.miss_velocity_decay = float(miss_velocity_decay)
        self.reset()

    def reset(self):
        self.ready = False
        self.x = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.p = [
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ]
        self.miss_frames = 0
        self.last_source = "reset"
        self.last_innov = (0.0, 0.0, 0.0)

    def init(self, cx, cy, radius, roi=None):
        self.ready = True
        self.x = [float(cx), float(cy), float(radius), 0.0, 0.0, 0.0]
        self.p = [
            [90.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 90.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 45.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 140.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 140.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 60.0],
        ]
        self.miss_frames = 0
        self.last_source = "init"
        self.last_innov = (0.0, 0.0, 0.0)
        self._limit_state()
        self._clamp(roi)
        return self.output()

    def _limit_state(self):
        if self.x[3] > self.max_speed:
            self.x[3] = self.max_speed
        elif self.x[3] < -self.max_speed:
            self.x[3] = -self.max_speed
        if self.x[4] > self.max_speed:
            self.x[4] = self.max_speed
        elif self.x[4] < -self.max_speed:
            self.x[4] = -self.max_speed
        if self.x[5] > self.max_radius_speed:
            self.x[5] = self.max_radius_speed
        elif self.x[5] < -self.max_radius_speed:
            self.x[5] = -self.max_radius_speed
        if self.x[2] < 4.0:
            self.x[2] = 4.0
            self.x[5] = 0.0

    def _clamp(self, roi):
        if roi is None:
            return
        min_x = float(roi[0])
        max_x = float(roi[0] + roi[2] - 1)
        min_y = float(roi[1])
        max_y = float(roi[1] + roi[3] - 1)
        if self.x[0] < min_x:
            self.x[0] = min_x
            self.x[3] = 0.0
        elif self.x[0] > max_x:
            self.x[0] = max_x
            self.x[3] = 0.0
        if self.x[1] < min_y:
            self.x[1] = min_y
            self.x[4] = 0.0
        elif self.x[1] > max_y:
            self.x[1] = max_y
            self.x[4] = 0.0
        max_r = max(5.0, float(min(roi[2], roi[3])) * 0.75)
        if self.x[2] > max_r:
            self.x[2] = max_r
            self.x[5] = 0.0

    def output(self):
        return int(self.x[0] + 0.5), int(self.x[1] + 0.5), int(self.x[2] + 0.5)

    def predict(self, roi=None):
        if not self.ready:
            return None
        self.x[0] += self.x[3]
        self.x[1] += self.x[4]
        self.x[2] += self.x[5]

        p = self.p
        fp = []
        for row in range(6):
            out_row = []
            for col in range(6):
                if row < 3:
                    out_row.append(p[row][col] + p[row + 3][col])
                else:
                    out_row.append(p[row][col])
            fp.append(out_row)

        new_p = []
        for row in range(6):
            out_row = []
            for col in range(6):
                if col < 3:
                    value = fp[row][col] + fp[row][col + 3]
                else:
                    value = fp[row][col]
                out_row.append(value)
            new_p.append(out_row)
        new_p[0][0] += self.process_var
        new_p[1][1] += self.process_var
        new_p[2][2] += self.radius_process_var
        new_p[3][3] += self.process_var
        new_p[4][4] += self.process_var
        new_p[5][5] += self.radius_process_var
        self.p = new_p

        self._limit_state()
        self._clamp(roi)
        self.last_source = "predict"
        return self.output()

    def _inv3(self, m):
        a = m[0][0]
        b = m[0][1]
        c = m[0][2]
        d = m[1][0]
        e = m[1][1]
        f = m[1][2]
        g = m[2][0]
        h = m[2][1]
        i = m[2][2]
        det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
        if abs(det) < 1e-6:
            return None
        inv_det = 1.0 / det
        return [
            [(e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det],
            [(f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det],
            [(d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det],
        ]

    def update(self, cx, cy, radius, measure_scale=1.0, roi=None):
        if not self.ready:
            return self.init(cx, cy, radius, roi)

        z = [float(cx), float(cy), float(radius)]
        y = [z[0] - self.x[0], z[1] - self.x[1], z[2] - self.x[2]]
        self.last_innov = (y[0], y[1], y[2])

        p = self.p
        r0 = self.measure_var * float(measure_scale)
        r2 = self.radius_measure_var * float(measure_scale)
        s = [
            [p[0][0] + r0, p[0][1], p[0][2]],
            [p[1][0], p[1][1] + r0, p[1][2]],
            [p[2][0], p[2][1], p[2][2] + r2],
        ]
        inv = self._inv3(s)
        if inv is None:
            self.miss_frames = 0
            self.last_source = "update singular"
            return self.output()

        k = []
        for row in range(6):
            k_row = []
            for col in range(3):
                value = p[row][0] * inv[0][col] + p[row][1] * inv[1][col] + p[row][2] * inv[2][col]
                k_row.append(value)
            k.append(k_row)

        for row in range(6):
            self.x[row] += k[row][0] * y[0] + k[row][1] * y[1] + k[row][2] * y[2]

        new_p = []
        for row in range(6):
            out_row = []
            for col in range(6):
                value = p[row][col] - k[row][0] * p[0][col] - k[row][1] * p[1][col] - k[row][2] * p[2][col]
                out_row.append(value)
            new_p.append(out_row)
        self.p = new_p
        self.miss_frames = 0
        self.last_source = "update"
        self._limit_state()
        self._clamp(roi)
        return self.output()

    def miss(self, roi=None):
        if not self.ready:
            return None
        self.miss_frames += 1
        if self.miss_frames > self.max_predict_frames:
            self.ready = False
            self.last_source = "lost"
            return None
        if self.miss_frames > 1:
            self.x[3] *= self.miss_velocity_decay
            self.x[4] *= self.miss_velocity_decay
            self.x[5] *= self.miss_velocity_decay
        return self.predict(roi)

    def reject(self, roi=None):
        if not self.ready:
            return None
        self.miss_frames += 1
        if self.miss_frames > self.max_predict_frames:
            self.ready = False
            self.last_source = "lost"
            return None
        self.x[3] *= self.miss_velocity_decay
        self.x[4] *= self.miss_velocity_decay
        self.x[5] *= self.miss_velocity_decay
        self._limit_state()
        self._clamp(roi)
        self.last_source = "reject"
        return self.output()

    def debug_values(self):
        return (
            1 if self.ready else 0,
            self.last_source,
            int(self.x[0] + 0.5),
            int(self.x[1] + 0.5),
            int(self.x[2] + 0.5),
            self.x[3],
            self.x[4],
            self.x[5],
            self.last_innov[0],
            self.last_innov[1],
            self.last_innov[2],
            self.miss_frames,
        )


class TargetCenterResult:
    def __init__(self, valid=False, cx=-1, cy=-1, roi=None, reason="missing", radius=-1, width=-1, height=-1):
        self.valid = valid
        self.cx = int(cx)
        self.cy = int(cy)
        self.roi = roi
        self.reason = reason
        self.radius = int(radius)
        self.width = int(width)
        self.height = int(height)
        self.kalman = None
        self.meas_cx = -1
        self.meas_cy = -1
        self.meas_r = -1
        self.meas_w = -1
        self.meas_h = -1
        self.meas_reason = ""


class TargetCenterFinder:
    def __init__(self, thresholds):
        self.thresholds = thresholds
        self.last = TargetCenterResult()
        self.reset()

    def reset(self):
        self.last = TargetCenterResult()
        self.last_circle_cx = -1
        self.last_circle_cy = -1
        self.last_circle_r = -1
        self.last_center_w = -1
        self.last_center_h = -1
        self.lost_circle_frames = 0
        self.circle_kalman = CircleKalmanTracker()

    def _obj_center(self, obj):
        return obj.x + obj.w // 2, obj.y + obj.h // 2

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

    def _circle_value(self, circle, name, index, default=0):
        attr = getattr(circle, name, None)
        if attr is not None:
            try:
                if callable(attr):
                    return int(attr())
                return int(attr)
            except Exception:
                pass
        try:
            return int(circle[index])
        except Exception:
            return int(default)

    def _circle_center(self, circle, roi):
        cx = self._circle_value(circle, "x", 0, -1)
        cy = self._circle_value(circle, "y", 1, -1)
        radius = self._circle_value(circle, "r", 2, -1)
        if radius <= 0:
            radius = self._circle_value(circle, "radius", 2, -1)
        if cx < 0 or cy < 0 or radius <= 0:
            return -1, -1, -1
        if not (roi[0] <= cx < roi[0] + roi[2] and roi[1] <= cy < roi[1] + roi[3]):
            if 0 <= cx < roi[2] and 0 <= cy < roi[3]:
                cx += roi[0]
                cy += roi[1]
        return cx, cy, radius

    def _dist2(self, ax, ay, bx, by):
        dx = ax - bx
        dy = ay - by
        return dx * dx + dy * dy

    def _circle_center_limit(self, obj):
        return max(18, int(max(obj.w, obj.h) * 0.45))

    def _arc_center_limit(self, obj):
        return max(22, int(max(obj.w, obj.h) * 0.50))

    def _circle_only_detection_ok(self, detection, obj):
        if detection is None:
            return False
        cx, cy, radius, width, height, source = detection
        obj_cx, obj_cy = self._obj_center(obj)
        limit = self._arc_center_limit(obj)
        if self._dist2(cx, cy, obj_cx, obj_cy) > limit * limit:
            return False
        if source.startswith("ellipse"):
            if width < obj.w * 0.55 or height < obj.h * 0.55:
                return False
        return True

    def _last_circle_in_roi(self, roi):
        return (
            self.last_circle_cx >= roi[0] and
            self.last_circle_cx < roi[0] + roi[2] and
            self.last_circle_cy >= roi[1] and
            self.last_circle_cy < roi[1] + roi[3]
        )

    def _solve_3x3(self, matrix, vector):
        a = [
            [float(matrix[0][0]), float(matrix[0][1]), float(matrix[0][2]), float(vector[0])],
            [float(matrix[1][0]), float(matrix[1][1]), float(matrix[1][2]), float(vector[1])],
            [float(matrix[2][0]), float(matrix[2][1]), float(matrix[2][2]), float(vector[2])],
        ]
        for col in range(3):
            pivot = col
            for row in range(col + 1, 3):
                if abs(a[row][col]) > abs(a[pivot][col]):
                    pivot = row
            if abs(a[pivot][col]) < 1e-6:
                return None
            if pivot != col:
                a[col], a[pivot] = a[pivot], a[col]
            div = a[col][col]
            for item in range(col, 4):
                a[col][item] /= div
            for row in range(3):
                if row == col:
                    continue
                scale = a[row][col]
                if scale == 0:
                    continue
                for item in range(col, 4):
                    a[row][item] -= scale * a[col][item]
        return a[0][3], a[1][3], a[2][3]

    def _fit_circle_from_points(self, points, origin_x, origin_y):
        if len(points) < 4:
            return None
        m00 = 0.0
        m01 = 0.0
        m02 = 0.0
        m11 = 0.0
        m12 = 0.0
        m22 = 0.0
        b0 = 0.0
        b1 = 0.0
        b2 = 0.0
        for px, py, weight in points:
            w = float(weight)
            x = float(px - origin_x)
            y = float(py - origin_y)
            z = x * x + y * y
            m00 += w * x * x
            m01 += w * x * y
            m02 += w * x
            m11 += w * y * y
            m12 += w * y
            m22 += w
            b0 -= w * x * z
            b1 -= w * y * z
            b2 -= w * z
        solved = self._solve_3x3(
            ((m00, m01, m02), (m01, m11, m12), (m02, m12, m22)),
            (b0, b1, b2),
        )
        if solved is None:
            return None
        a, b, c = solved
        local_cx = -a * 0.5
        local_cy = -b * 0.5
        radius2 = local_cx * local_cx + local_cy * local_cy - c
        if radius2 <= 0:
            return None
        return int(origin_x + local_cx + 0.5), int(origin_y + local_cy + 0.5), int(math.sqrt(radius2) + 0.5)

    def _add_arc_point(self, points, x, y, weight):
        points.append((int(x), int(y), int(weight)))

    def _black_arc_points(self, img, roi, obj):
        try:
            blobs = img.find_blobs(
                self.thresholds,
                roi=roi,
                pixels_threshold=6,
                area_threshold=6,
                merge=False,
                margin=1,
            )
        except Exception:
            return None, None, "blob err"

        if len(blobs) == 0:
            return None, None, "no black"

        obj_cx, obj_cy = self._obj_center(obj)
        center_limit = self._arc_center_limit(obj)
        center_limit2 = center_limit * center_limit
        min_x = roi[0] + roi[2]
        min_y = roi[1] + roi[3]
        max_x = roi[0]
        max_y = roi[1]
        points = []
        accepted = 0
        pixels = 0
        min_side = min(obj.w, obj.h)

        for blob in blobs:
            bx = int(blob[0])
            by = int(blob[1])
            bw = int(blob[2])
            bh = int(blob[3])
            bp = int(blob[4])
            if bp < 5 or bw < 2 or bh < 2:
                continue
            bc_x = bx + bw // 2
            bc_y = by + bh // 2
            if self._dist2(bc_x, bc_y, obj_cx, obj_cy) > center_limit2:
                continue
            if bw > obj.w * 1.55 or bh > obj.h * 1.55:
                continue

            accepted += 1
            pixels += bp
            min_x = min(min_x, bx)
            min_y = min(min_y, by)
            max_x = max(max_x, bx + bw)
            max_y = max(max_y, by + bh)

            weight = min(24, max(2, bp // 3))
            large_component = bw >= min_side * 0.35 or bh >= min_side * 0.35
            if not large_component:
                self._add_arc_point(points, bc_x, bc_y, weight)
            if bw >= 4:
                self._add_arc_point(points, bx, bc_y, max(1, weight // 3))
                self._add_arc_point(points, bx + bw - 1, bc_y, max(1, weight // 3))
            if bh >= 4:
                self._add_arc_point(points, bc_x, by, max(1, weight // 3))
                self._add_arc_point(points, bc_x, by + bh - 1, max(1, weight // 3))

        if accepted == 0 or pixels < 14 or len(points) < 3:
            return None, None, "weak black"
        return points, (min_x, min_y, max_x, max_y, pixels, accepted), "arc pts"

    def _median_radius(self, points, cx, cy):
        distances = []
        for px, py, weight in points:
            distances.append(int(math.sqrt(self._dist2(px, py, cx, cy)) + 0.5))
        if len(distances) == 0:
            return -1
        distances.sort()
        return distances[len(distances) // 2]

    def _score_arc_circle(self, candidate, points, roi, obj, source):
        if candidate is None:
            return None
        cx, cy, radius = candidate
        min_side = min(obj.w, obj.h)
        r_min = max(5, int(min_side * 0.16))
        r_max = max(r_min + 3, int(min(roi[2], roi[3]) * 0.72))
        if radius < r_min or radius > r_max:
            return None
        if cx < roi[0] or cx >= roi[0] + roi[2] or cy < roi[1] or cy >= roi[1] + roi[3]:
            return None

        obj_cx, obj_cy = self._obj_center(obj)
        center_limit = self._arc_center_limit(obj)
        center_dist2 = self._dist2(cx, cy, obj_cx, obj_cy)
        if center_dist2 > center_limit * center_limit:
            return None

        tolerance = max(4.0, float(radius) * 0.24)
        support_weight = 0
        support_error = 0.0
        bins = [False] * 12
        for px, py, weight in points:
            dist = math.sqrt(self._dist2(px, py, cx, cy))
            err = abs(dist - radius)
            if err > tolerance:
                continue
            support_weight += weight
            support_error += err * weight
            angle = math.atan2(py - cy, px - cx)
            if angle < 0:
                angle += math.pi * 2.0
            bins[int(angle / (math.pi * 2.0) * 12.0) % 12] = True

        support_bins = 0
        for ok in bins:
            if ok:
                support_bins += 1
        last_ok = self.lost_circle_frames <= 4 and self._last_circle_in_roi(roi)
        min_bins = 4 if last_ok else 5
        min_support = 30 if last_ok else 45
        if support_bins < min_bins or support_weight < min_support:
            return None

        mean_error = support_error / float(max(1, support_weight))
        center_dist = math.sqrt(center_dist2)
        radius_ref = self.last_circle_r if last_ok and self.last_circle_r > 0 else max(5, min_side // 3)
        if last_ok and abs(radius - self.last_circle_r) > max(7, int(self.last_circle_r * 0.35)):
            if support_bins < 7 or support_weight < 80:
                return None
        score = mean_error * 8.0 + center_dist * 0.30 + abs(radius - radius_ref) * 0.20
        score -= support_bins * 3.5 + support_weight * 0.03
        if last_ok:
            score += math.sqrt(self._dist2(cx, cy, self.last_circle_cx, self.last_circle_cy)) * 0.55
            score += abs(radius - self.last_circle_r) * 0.35
        return score, support_bins, support_weight

    def _find_arc_circle(self, img, roi, obj):
        points, bounds, reason = self._black_arc_points(img, roi, obj)
        if points is None or bounds is None:
            return None

        obj_cx, obj_cy = self._obj_center(obj)
        min_x, min_y, max_x, max_y, _, _ = bounds
        width = max_x - min_x
        height = max_y - min_y
        if width < 4 or height < 4:
            return None

        candidates = []
        fitted = self._fit_circle_from_points(points, obj_cx, obj_cy)
        if fitted is not None:
            candidates.append((fitted[0], fitted[1], fitted[2], "arc fit"))

        bbox_radius = int((width + height) * 0.25 + 0.5)
        candidates.append(((min_x + max_x) // 2, (min_y + max_y) // 2, bbox_radius, "arc bbox"))

        anchor_radius = self._median_radius(points, obj_cx, obj_cy)
        if anchor_radius > 0:
            candidates.append((obj_cx, obj_cy, anchor_radius, "arc anchor"))

        if self.lost_circle_frames <= 4 and self._last_circle_in_roi(roi) and self.last_circle_r > 0:
            candidates.append((self.last_circle_cx, self.last_circle_cy, self.last_circle_r, "arc last"))

        best = None
        best_score = None
        best_reason = "arc"
        for cx, cy, radius, source in candidates:
            scored = self._score_arc_circle((cx, cy, radius), points, roi, obj, source)
            if scored is None:
                continue
            score, support_bins, support_weight = scored
            if best_score is None or score < best_score:
                best_score = score
                best = (cx, cy, radius)
                best_reason = "%s %d/%d" % (source, support_bins, support_weight)
        if best is None:
            return None
        cx, cy, radius = best
        diameter = radius * 2
        return (cx, cy, radius, diameter, diameter, best_reason)

    def _find_black_ellipse(self, img, roi, obj, panel_w, panel_h):
        try:
            blobs = img.find_blobs(
                self.thresholds,
                roi=roi,
                pixels_threshold=12,
                area_threshold=12,
                merge=True,
                margin=5,
            )
        except Exception:
            return None, "blob err"

        if len(blobs) == 0:
            return None, "no black"

        obj_cx, obj_cy = self._obj_center(obj)
        center_limit = max(22, int(max(obj.w, obj.h) * 0.60))
        center_limit2 = center_limit * center_limit
        min_x = panel_w
        min_y = panel_h
        max_x = 0
        max_y = 0
        pixels = 0
        weighted_x = 0
        weighted_y = 0
        for blob in blobs:
            bx = blob[0]
            by = blob[1]
            bw = blob[2]
            bh = blob[3]
            bp = blob[4]
            if bw < 3 or bh < 3 or bp < 8:
                continue
            bc_x = bx + bw // 2
            bc_y = by + bh // 2
            if self._dist2(bc_x, bc_y, obj_cx, obj_cy) > center_limit2:
                continue
            min_x = min(min_x, bx)
            min_y = min(min_y, by)
            max_x = max(max_x, bx + bw)
            max_y = max(max_y, by + bh)
            pixels += bp
            weighted_x += bc_x * bp
            weighted_y += bc_y * bp

        if pixels < 24 or max_x <= min_x or max_y <= min_y:
            return None, "weak black"

        w = max_x - min_x
        h = max_y - min_y
        min_size = min(obj.w, obj.h)
        if w < min_size * 0.20 or h < min_size * 0.20:
            return None, "small ellipse"
        if w > obj.w * 1.35 or h > obj.h * 1.35:
            return None, "large ellipse"
        ratio = float(w) / float(h)
        if ratio < 0.35 or ratio > 2.85:
            return None, "bad ellipse"

        bbox_cx = (min_x + max_x) // 2
        bbox_cy = (min_y + max_y) // 2
        blob_cx = weighted_x // pixels
        blob_cy = weighted_y // pixels
        cx = int(bbox_cx * 0.70 + blob_cx * 0.30)
        cy = int(bbox_cy * 0.70 + blob_cy * 0.30)
        radius = max(4, (w + h) // 4)
        return (cx, cy, radius, w, h, "ellipse"), "ellipse"

    def _find_best_circle(self, img, roi, obj):
        min_side = min(obj.w, obj.h)
        r_min = max(5, min_side // 5)
        r_max = max(r_min + 2, min(roi[2], roi[3]) // 2)
        try:
            circles = img.find_circles(
                roi=roi,
                x_stride=2,
                y_stride=2,
                threshold=1800,
                x_margin=8,
                y_margin=8,
                r_margin=6,
                r_min=r_min,
                r_max=r_max,
                r_step=2,
            )
        except Exception:
            return None

        best = None
        best_score = None
        obj_cx, obj_cy = self._obj_center(obj)
        center_limit = self._circle_center_limit(obj)
        center_limit2 = center_limit * center_limit
        last_ok = self.lost_circle_frames <= 4 and self._last_circle_in_roi(roi)
        radius_ref = self.last_circle_r if last_ok and self.last_circle_r > 0 else min_side // 3
        for circle in circles:
            cx, cy, radius = self._circle_center(circle, roi)
            if cx < roi[0] or cx >= roi[0] + roi[2]:
                continue
            if cy < roi[1] or cy >= roi[1] + roi[3]:
                continue
            obj_dist2 = self._dist2(cx, cy, obj_cx, obj_cy)
            if obj_dist2 > center_limit2:
                continue

            score = obj_dist2 * 6 - radius * radius * 2 + abs(radius - radius_ref) * 18
            if last_ok:
                last_dist2 = self._dist2(cx, cy, self.last_circle_cx, self.last_circle_cy)
                score += last_dist2 * 3
                if last_dist2 > center_limit2 and obj_dist2 >= self._dist2(
                        self.last_circle_cx,
                        self.last_circle_cy,
                        obj_cx,
                        obj_cy):
                    continue
            if best_score is not None and score >= best_score:
                continue
            diameter = radius * 2
            best = (cx, cy, radius, diameter, diameter, "circle")
            best_score = score
        return best

    def _detection_measure_scale(self, source):
        if source.startswith("arc fit"):
            return 0.12
        if source.startswith("circle"):
            return 0.16
        if source.startswith("arc bbox") or source.startswith("arc anchor"):
            return 0.22
        if source.startswith("ellipse"):
            return 0.18
        return 0.30

    def _with_shape_size(self, detection, ellipse):
        if detection is None or ellipse is None:
            return detection
        cx, cy, radius, width, height, source = detection
        _, _, _, ell_w, ell_h, _ = ellipse
        if ell_w > 0 and ell_h > 0:
            return (cx, cy, radius, ell_w, ell_h, source)
        return detection

    def _make_result(self, valid=False, cx=-1, cy=-1, roi=None, reason="missing",
                     radius=-1, width=-1, height=-1, measurement=None):
        result = TargetCenterResult(valid, cx, cy, roi, reason, radius, width, height)
        result.kalman = self.circle_kalman.debug_values()
        if measurement is not None:
            result.meas_cx = int(measurement[0])
            result.meas_cy = int(measurement[1])
            result.meas_r = int(measurement[2])
            result.meas_w = int(measurement[3])
            result.meas_h = int(measurement[4])
            result.meas_reason = str(measurement[5])
        return result

    def _stabilize_center(self, detection, roi, obj):
        if detection is None:
            predicted = self.circle_kalman.miss(roi)
            if predicted is not None and self._last_circle_in_roi(roi):
                cx, cy, radius = predicted
                self.lost_circle_frames += 1
                self.last_circle_cx = int(cx)
                self.last_circle_cy = int(cy)
                self.last_circle_r = int(radius)
                return self.last_circle_cx, self.last_circle_cy, self.last_circle_r, self.last_center_w, self.last_center_h, "kalman predict", None
            self.circle_kalman.reset()
            return None

        cx, cy, radius, width, height, source = detection
        if self.circle_kalman.ready:
            self.circle_kalman.predict(roi)
            pred_cx, pred_cy, pred_r = self.circle_kalman.output()
            jump_limit = max(self._circle_center_limit(obj) * 3, int(max(10, pred_r) * 3.0))
            radius_limit = max(18, int(max(6, pred_r) * 1.20))
            if self._dist2(cx, cy, pred_cx, pred_cy) > jump_limit * jump_limit:
                if source.startswith("circle") or source.startswith("arc fit"):
                    self.circle_kalman.init(cx, cy, radius, roi)
                else:
                    predicted = self.circle_kalman.reject(roi)
                    if predicted is None:
                        return None
                    kx, ky, kr = predicted
                    return int(kx), int(ky), int(kr), self.last_center_w, self.last_center_h, "kalman reject", None
            if abs(radius - pred_r) > radius_limit:
                strong_shape = source.startswith("arc fit") or source.startswith("circle") or source.startswith("ellipse")
                if not strong_shape:
                    radius = pred_r

        filtered = self.circle_kalman.update(cx, cy, radius, self._detection_measure_scale(source), roi)
        if filtered is None:
            self.circle_kalman.reset()
            return None
        kx, ky, kr = filtered
        if self.last_circle_r > 0:
            kr = int(self.last_circle_r * 0.70 + int(kr) * 0.30 + 0.5)
        if source.startswith("ellipse"):
            out_w = int(width)
            out_h = int(height)
        elif self.last_center_w > 0 and self.last_center_h > 0:
            out_w = int(self.last_center_w * 0.35 + width * 0.65)
            out_h = int(self.last_center_h * 0.35 + height * 0.65)
        else:
            out_w = int(width)
            out_h = int(height)
        reason = source + " kf"

        self.last_circle_cx = int(kx)
        self.last_circle_cy = int(ky)
        self.last_circle_r = int(kr)
        self.last_center_w = int(out_w)
        self.last_center_h = int(out_h)
        self.lost_circle_frames = 0
        return self.last_circle_cx, self.last_circle_cy, self.last_circle_r, self.last_center_w, self.last_center_h, reason, detection

    def update(self, img, obj, panel_w, panel_h, circle_only=False):
        if obj is None:
            self.last = self._make_result(reason="no target")
            self.lost_circle_frames += 1
            return self.last

        roi = self._expanded_roi(obj, panel_w, panel_h)
        if roi is None:
            self.last = self._make_result(reason="bad roi")
            self.lost_circle_frames += 1
            return self.last

        ellipse, ellipse_reason = self._find_black_ellipse(img, roi, obj, panel_w, panel_h)
        if circle_only:
            arc_circle = self._find_arc_circle(img, roi, obj)
            if self._circle_only_detection_ok(arc_circle, obj):
                arc_circle = self._with_shape_size(arc_circle, ellipse)
                center = self._stabilize_center(arc_circle, roi, obj)
                if center is not None:
                    cx, cy, radius, width, height, reason, measurement = center
                    self.last = self._make_result(True, cx, cy, roi, reason, radius, width, height, measurement)
                    return self.last

        best_circle = self._find_best_circle(img, roi, obj)
        if best_circle is not None and (not circle_only or self._circle_only_detection_ok(best_circle, obj)):
            best_circle = self._with_shape_size(best_circle, ellipse)
            center = self._stabilize_center(best_circle, roi, obj)
            if center is not None:
                cx, cy, radius, width, height, reason, measurement = center
                self.last = self._make_result(True, cx, cy, roi, reason, radius, width, height, measurement)
                return self.last

        if circle_only:
            if self._circle_only_detection_ok(ellipse, obj):
                center = self._stabilize_center(ellipse, roi, obj)
                if center is not None:
                    cx, cy, radius, width, height, reason, measurement = center
                    self.last = self._make_result(True, cx, cy, roi, reason, radius, width, height, measurement)
                    return self.last
            center = self._stabilize_center(None, roi, obj)
            if center is not None:
                cx, cy, radius, width, height, reason, measurement = center
                self.last = self._make_result(True, cx, cy, roi, reason, radius, width, height, measurement)
                return self.last
            self.lost_circle_frames += 1
            self.last = self._make_result(roi=roi, reason=ellipse_reason)
            return self.last

        if ellipse is not None:
            center = self._stabilize_center(ellipse, roi, obj)
            if center is not None:
                cx, cy, radius, width, height, reason, measurement = center
                self.last = self._make_result(True, cx, cy, roi, reason, radius, width, height, measurement)
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
            self.lost_circle_frames += 1
            self.last = self._make_result(roi=roi, reason="blob err")
            return self.last

        if len(blobs) == 0:
            self.lost_circle_frames += 1
            self.last = self._make_result(roi=roi, reason="no black")
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
            self.lost_circle_frames += 1
            self.last = self._make_result(roi=roi, reason="weak black")
            return self.last

        w = max_x - min_x
        h = max_y - min_y
        if w < obj.w * 0.35 or h < obj.h * 0.35:
            self.lost_circle_frames += 1
            self.last = self._make_result(roi=roi, reason="small ring")
            return self.last
        ratio = float(w) / float(h)
        if ratio < 0.65 or ratio > 1.55:
            self.lost_circle_frames += 1
            self.last = self._make_result(roi=roi, reason="bad ratio")
            return self.last

        cx = (min_x + max_x) // 2
        cy = (min_y + max_y) // 2
        self.last = self._make_result(True, cx, cy, roi, "blob", (w + h) // 4, w, h,
                                      (cx, cy, (w + h) // 4, w, h, "blob"))
        return self.last
