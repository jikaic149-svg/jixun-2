from .base import best_obj_for_class
from .basic_scan_lr import BasicScanLeftRightTask
from .constants import MEMORY_SCAN_LIMIT_MS


class ExtMemoryScanTask(BasicScanLeftRightTask):
    timeout_ms = MEMORY_SCAN_LIMIT_MS

    def _ensure_sequence(self, runtime, detections):
        if len(runtime.sequence) > 0:
            return
        if hasattr(self, "memory_order") and len(self.memory_order) > 0:
            runtime.sequence = list(self.memory_order)
            runtime.status = "READY"
            return
        runtime.status = "NO MEMORY"

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
