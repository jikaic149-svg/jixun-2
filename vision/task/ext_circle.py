import math

from .basic_center import BasicCenterTask
from .base import obj_center, is_locked
from .constants import CENTER_STABLE_MS, CIRCLE_LIMIT_MS, CIRCLE_TRACE_MS


class ExtCircleTask(BasicCenterTask):
    timeout_ms = CIRCLE_LIMIT_MS

    def update_lock(self, runtime, control_obj, output_valid, tx_offset, now_ms):
        if self.check_timeout(runtime, now_ms):
            return
        if runtime.circle_start_ms >= 0:
            if now_ms - runtime.circle_start_ms >= CIRCLE_TRACE_MS:
                runtime.done = True
                runtime.status = "DONE"
            else:
                runtime.status = "TRACE"
            return
        if is_locked(output_valid, tx_offset):
            if runtime.hold_start_ms < 0:
                runtime.hold_start_ms = now_ms
            if now_ms - runtime.hold_start_ms >= CENTER_STABLE_MS:
                runtime.circle_start_ms = now_ms
                runtime.status = "TRACE"
        else:
            runtime.hold_start_ms = -1
            runtime.status = "LOCKING"

    def override_control_obj(self, runtime, control_obj, output_valid, box_factory, now_ms):
        if control_obj is None or not output_valid:
            return control_obj
        if runtime.circle_start_ms < 0 or runtime.done:
            return control_obj
        elapsed = now_ms - runtime.circle_start_ms
        if elapsed >= CIRCLE_TRACE_MS:
            runtime.done = True
            runtime.status = "DONE"
            return control_obj

        progress = float(elapsed) / float(CIRCLE_TRACE_MS)
        angle = progress * 2.0 * math.pi
        cx, cy = obj_center(control_obj)
        radius = max(control_obj.w, control_obj.h) * 0.55
        if radius < 18:
            radius = 18.0
        px = int(cx + math.cos(angle) * radius + 0.5)
        py = int(cy + math.sin(angle) * radius + 0.5)
        return box_factory(px, py, 8, 8, control_obj.class_id, control_obj.score)
