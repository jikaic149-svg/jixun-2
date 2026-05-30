import math

from .basic_center import BasicCenterTask
from .base import is_locked
from .constants import (
    CENTER_STABLE_MS,
    CIRCLE_CENTER_STABLE_MS,
    CIRCLE_CLOSE_STABLE_MS,
    CIRCLE_LIMIT_MS,
    CIRCLE_TRACE_MS,
    CIRCLE_RADIUS_SCALE,
    CIRCLE_MIN_RADIUS,
    CIRCLE_ENTRY_RAMP_MS,
    CIRCLE_TRACE_CENTER_ALPHA,
    CIRCLE_TRACE_RADIUS_ALPHA,
    CIRCLE_TRACE_CENTER_MAX_STEP,
    CIRCLE_TRACE_RADIUS_MAX_STEP,
    CIRCLE_PHASE_FULL_SPEED_ERR,
    CIRCLE_PHASE_SPEED_ALPHA,
    CIRCLE_PHASE_STOP_ERR,
)


class ExtCircleTask(BasicCenterTask):
    timeout_ms = CIRCLE_LIMIT_MS

    def _slew(self, current, target, alpha, max_step):
        value = current + (target - current) * alpha
        delta = value - current
        if delta > max_step:
            value = current + max_step
        elif delta < -max_step:
            value = current - max_step
        return value

    def _trace_geometry(self, runtime, control_obj):
        if runtime.circle_center_cx < 0 or runtime.circle_center_cy < 0:
            return None
        base_w = max(1, getattr(runtime, "circle_target_w", control_obj.w))
        base_h = max(1, getattr(runtime, "circle_target_h", control_obj.h))
        box_radius = max(base_w, base_h) * 0.5
        target_r = getattr(runtime, "circle_target_r", -1)
        if target_r > 0 and target_r >= box_radius * 0.65:
            source_radius = float(target_r)
        else:
            source_radius = box_radius
        trace_radius = max(source_radius * CIRCLE_RADIUS_SCALE, float(CIRCLE_MIN_RADIUS))
        target_rx = trace_radius
        target_ry = trace_radius

        if runtime.circle_trace_cx < 0 or runtime.circle_trace_cy < 0:
            runtime.circle_trace_cx = float(runtime.circle_center_cx)
            runtime.circle_trace_cy = float(runtime.circle_center_cy)
            runtime.circle_trace_rx = target_rx
            runtime.circle_trace_ry = target_ry
        else:
            runtime.circle_trace_cx = self._slew(
                runtime.circle_trace_cx,
                float(runtime.circle_center_cx),
                CIRCLE_TRACE_CENTER_ALPHA,
                CIRCLE_TRACE_CENTER_MAX_STEP,
            )
            runtime.circle_trace_cy = self._slew(
                runtime.circle_trace_cy,
                float(runtime.circle_center_cy),
                CIRCLE_TRACE_CENTER_ALPHA,
                CIRCLE_TRACE_CENTER_MAX_STEP,
            )
            runtime.circle_trace_rx = self._slew(
                runtime.circle_trace_rx,
                target_rx,
                CIRCLE_TRACE_RADIUS_ALPHA,
                CIRCLE_TRACE_RADIUS_MAX_STEP,
            )
            runtime.circle_trace_ry = self._slew(
                runtime.circle_trace_ry,
                target_ry,
                CIRCLE_TRACE_RADIUS_ALPHA,
                CIRCLE_TRACE_RADIUS_MAX_STEP,
            )
        return (
            runtime.circle_trace_cx,
            runtime.circle_trace_cy,
            runtime.circle_trace_rx,
            runtime.circle_trace_ry,
        )

    def update_lock(self, runtime, control_obj, output_valid, tx_offset, now_ms):
        if self.check_timeout(runtime, now_ms):
            return
        if runtime.circle_start_ms >= 0:
            if runtime.circle_phase >= 2.0 * math.pi:
                runtime.status = "CLOSE"
            else:
                runtime.status = "TRACE"
            return
        if is_locked(output_valid, tx_offset):
            if runtime.hold_start_ms < 0:
                runtime.hold_start_ms = now_ms
            lock_ms = CIRCLE_CENTER_STABLE_MS
            if lock_ms <= 0:
                lock_ms = CENTER_STABLE_MS
            if now_ms - runtime.hold_start_ms >= lock_ms:
                runtime.circle_start_ms = now_ms
                runtime.circle_last_phase_ms = now_ms
                runtime.circle_phase = 0.0
                runtime.status = "TRACE"
        else:
            runtime.hold_start_ms = -1
            runtime.status = "LOCKING"

    def _point_at_phase(self, geometry, phase, elapsed):
        cx, cy, radius_x, radius_y = geometry
        ramp = 1.0
        if CIRCLE_ENTRY_RAMP_MS > 0 and elapsed < CIRCLE_ENTRY_RAMP_MS:
            ramp = float(elapsed) / float(CIRCLE_ENTRY_RAMP_MS)
        radius_x *= ramp
        radius_y *= ramp
        px = int(cx + math.cos(phase) * radius_x + 0.5)
        py = int(cy + math.sin(phase) * radius_y + 0.5)
        return px, py

    def _phase_speed_scale(self, runtime, output_valid, tx_offset):
        target_scale = 0.0
        if output_valid and tx_offset is not None:
            dx, dy = tx_offset
            err = max(abs(dx), abs(dy))
            if err <= CIRCLE_PHASE_FULL_SPEED_ERR:
                target_scale = 1.0
            elif err < CIRCLE_PHASE_STOP_ERR:
                span = max(1.0, float(CIRCLE_PHASE_STOP_ERR - CIRCLE_PHASE_FULL_SPEED_ERR))
                target_scale = float(CIRCLE_PHASE_STOP_ERR - err) / span
        runtime.circle_phase_speed_scale += (
            target_scale - runtime.circle_phase_speed_scale
        ) * CIRCLE_PHASE_SPEED_ALPHA
        if runtime.circle_phase_speed_scale < 0.02:
            runtime.circle_phase_speed_scale = 0.0
        elif runtime.circle_phase_speed_scale > 1.0:
            runtime.circle_phase_speed_scale = 1.0
        return runtime.circle_phase_speed_scale

    def override_control_obj(self, runtime, control_obj, output_valid, box_factory, now_ms, tx_offset=None, offset_fn=None):
        if control_obj is None or not output_valid:
            return control_obj
        if runtime.circle_start_ms < 0 or runtime.done:
            return control_obj
        elapsed = now_ms - runtime.circle_start_ms

        geometry = self._trace_geometry(runtime, control_obj)
        if geometry is None:
            return control_obj

        if runtime.circle_last_phase_ms < 0:
            runtime.circle_last_phase_ms = now_ms
        dt_ms = max(0, now_ms - runtime.circle_last_phase_ms)
        runtime.circle_last_phase_ms = now_ms

        phase_tx_offset = tx_offset
        if offset_fn is not None:
            hold_px, hold_py = self._point_at_phase(geometry, runtime.circle_phase, elapsed)
            hold_obj = box_factory(hold_px, hold_py, 8, 8, control_obj.class_id, control_obj.score)
            phase_tx_offset = offset_fn(hold_obj)

        speed_scale = self._phase_speed_scale(runtime, output_valid, phase_tx_offset)
        runtime.circle_phase += (
            float(dt_ms) / float(CIRCLE_TRACE_MS)
        ) * 2.0 * math.pi * speed_scale
        phase = runtime.circle_phase
        if runtime.circle_phase >= 2.0 * math.pi:
            phase = 2.0 * math.pi
            runtime.status = "CLOSE"
        elif speed_scale <= 0.0:
            runtime.status = "TRACE HOLD"
        elif speed_scale < 0.95:
            runtime.status = "TRACE %.2f" % speed_scale

        px, py = self._point_at_phase(geometry, phase, elapsed)
        out_obj = box_factory(px, py, 8, 8, control_obj.class_id, control_obj.score)
        if phase >= 2.0 * math.pi and offset_fn is not None:
            close_offset = offset_fn(out_obj)
            if is_locked(output_valid, close_offset):
                if runtime.circle_close_start_ms < 0:
                    runtime.circle_close_start_ms = now_ms
                if now_ms - runtime.circle_close_start_ms >= CIRCLE_CLOSE_STABLE_MS:
                    runtime.done = True
                    runtime.status = "DONE"
            else:
                runtime.circle_close_start_ms = -1
        return out_obj
