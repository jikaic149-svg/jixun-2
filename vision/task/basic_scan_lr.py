from .base import BaseTask, best_obj_for_class, is_locked, visible_objects_left_to_right
from .constants import BASIC_SCAN_LIMIT_MS, TARGET_HOLD_MS


class BasicScanLeftRightTask(BaseTask):
    timeout_ms = BASIC_SCAN_LIMIT_MS

    def _ensure_sequence(self, runtime, detections):
        if len(runtime.sequence) > 0:
            return
        objs = visible_objects_left_to_right(detections, self.target_class_ids)
        runtime.sequence = [obj.class_id for obj in objs]
        if len(runtime.sequence) > 0:
            runtime.status = "READY"

    def pick_target(self, runtime, detections, selected_class_id, now_ms):
        if self.check_timeout(runtime, now_ms) or runtime.done:
            return None
        self._ensure_sequence(runtime, detections)
        if len(runtime.sequence) == 0:
            runtime.status = "NO OBJ"
            return None
        if runtime.index >= len(runtime.sequence):
            runtime.done = True
            runtime.status = "DONE"
            return None

        class_id = runtime.sequence[runtime.index]
        if selected_class_id == class_id:
            return None

        obj = best_obj_for_class(detections, class_id)
        if obj is None:
            runtime.status = "WAIT OBJ"
            return None
        runtime.status = "SELECT"
        runtime.hold_start_ms = -1
        return obj

    def update_lock(self, runtime, control_obj, output_valid, tx_offset, now_ms):
        if self.check_timeout(runtime, now_ms):
            return
        if runtime.done or runtime.index >= len(runtime.sequence):
            return
        if is_locked(output_valid, tx_offset):
            if runtime.hold_start_ms < 0:
                runtime.hold_start_ms = now_ms
            elapsed = now_ms - runtime.hold_start_ms
            if elapsed >= TARGET_HOLD_MS:
                runtime.index += 1
                runtime.hold_start_ms = -1
                runtime.advance_requested = True
                if runtime.index >= len(runtime.sequence):
                    runtime.done = True
                    runtime.status = "DONE"
                else:
                    runtime.status = "NEXT"
            else:
                runtime.status = "HOLD"
        else:
            runtime.hold_start_ms = -1
            runtime.status = "LOCKING"

    def status_text(self, runtime, name, label_fn=None):
        if runtime.timeout:
            return "%s TIMEOUT" % name
        total = len(runtime.sequence)
        current = runtime.index + 1 if runtime.index < total else total
        label = ""
        if total > 0 and runtime.index < total:
            class_id = runtime.sequence[runtime.index]
            label = self.labels[class_id]
            if label_fn is not None:
                label = label_fn(class_id)
        return "%s %d/%d %s %s" % (name, current, total, label, runtime.status)
