from .constants import LOCK_DX, LOCK_DY, TARGET_LABELS


class TaskRuntime:
    def __init__(self):
        self.sequence = []
        self.index = 0
        self.hold_start_ms = -1
        self.task_start_ms = -1
        self.circle_start_ms = -1
        self.done = False
        self.timeout = False
        self.status = "SEARCH"
        self.advance_requested = False

    def reset(self, now_ms):
        self.sequence = []
        self.index = 0
        self.hold_start_ms = -1
        self.task_start_ms = now_ms
        self.circle_start_ms = -1
        self.done = False
        self.timeout = False
        self.status = "SEARCH"
        self.advance_requested = False


def resolve_target_ids(labels):
    ids = []
    for i, label in enumerate(labels):
        if label in TARGET_LABELS:
            ids.append(i)
    return ids


def obj_center(obj):
    return obj.x + obj.w // 2, obj.y + obj.h // 2


def is_locked(output_valid, tx_offset):
    if not output_valid or tx_offset is None:
        return False
    dx, dy = tx_offset
    return abs(dx) <= LOCK_DX and abs(dy) <= LOCK_DY


def best_target_obj(detections, target_class_ids):
    best = None
    for obj in detections:
        if obj.class_id not in target_class_ids:
            continue
        if best is None or obj.score > best.score:
            best = obj
    return best


def visible_objects_left_to_right(detections, target_class_ids):
    objs = []
    for obj in detections:
        objs.append(obj)
    objs.sort(key=lambda obj: obj_center(obj)[0])
    return objs


def best_obj_for_class(detections, class_id):
    best = None
    for obj in detections:
        if obj.class_id != class_id:
            continue
        if best is None or obj.score > best.score:
            best = obj
    return best


class BaseTask:
    timeout_ms = 0

    def __init__(self, labels, target_class_ids):
        self.labels = labels
        self.target_class_ids = target_class_ids

    def reset(self, runtime, now_ms):
        runtime.reset(now_ms)

    def check_timeout(self, runtime, now_ms):
        if self.timeout_ms <= 0 or runtime.task_start_ms < 0:
            return False
        if now_ms - runtime.task_start_ms > self.timeout_ms:
            runtime.timeout = True
            runtime.done = True
            runtime.status = "TIMEOUT"
            return True
        return False

    def pick_target(self, runtime, detections, selected_class_id, now_ms):
        return None

    def update_lock(self, runtime, control_obj, output_valid, tx_offset, now_ms):
        pass

    def override_control_obj(self, runtime, control_obj, output_valid, box_factory, now_ms):
        return control_obj

    def status_text(self, runtime, name, label_fn=None):
        if runtime.timeout:
            return "%s TIMEOUT" % name
        return "%s %s" % (name, runtime.status)
