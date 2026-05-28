from .base import BaseTask, best_target_obj, is_locked


class BasicCenterTask(BaseTask):
    def pick_target(self, runtime, detections, selected_class_id, now_ms):
        if self.check_timeout(runtime, now_ms) or runtime.done:
            return None
        obj = best_target_obj(detections, self.target_class_ids)
        if obj is None:
            runtime.status = "NO TARGET"
            return None
        if selected_class_id != obj.class_id:
            runtime.status = "SELECT"
            return obj
        return None

    def update_lock(self, runtime, control_obj, output_valid, tx_offset, now_ms):
        if self.check_timeout(runtime, now_ms):
            return
        if is_locked(output_valid, tx_offset):
            if runtime.hold_start_ms < 0:
                runtime.hold_start_ms = now_ms
            runtime.status = "LOCK"
        else:
            runtime.hold_start_ms = -1
            runtime.status = "LOCKING"
