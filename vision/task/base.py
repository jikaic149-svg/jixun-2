from .constants import LOCK_DX, LOCK_DY, SCAN_LABELS, SCAN_LOCK_DX, SCAN_LOCK_DY, TARGET_LABELS


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
        self.lock_cx = -1
        self.lock_cy = -1
        self.lock_w = -1
        self.lock_h = -1
        self.lock_class_id = -1
        self.lock_score = 0.0
        self.circle_target_w = -1
        self.circle_target_h = -1
        self.circle_target_r = -1
        self.circle_center_cx = -1
        self.circle_center_cy = -1
        self.circle_trace_cx = -1
        self.circle_trace_cy = -1
        self.circle_trace_rx = -1.0
        self.circle_trace_ry = -1.0
        self.circle_phase = 0.0
        self.circle_phase_speed_scale = 0.0
        self.circle_last_phase_ms = -1
        self.circle_close_start_ms = -1
        self.scan_votes = []

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
        self.lock_cx = -1
        self.lock_cy = -1
        self.lock_w = -1
        self.lock_h = -1
        self.lock_class_id = -1
        self.lock_score = 0.0
        self.circle_target_w = -1
        self.circle_target_h = -1
        self.circle_target_r = -1
        self.circle_center_cx = -1
        self.circle_center_cy = -1
        self.circle_trace_cx = -1
        self.circle_trace_cy = -1
        self.circle_trace_rx = -1.0
        self.circle_trace_ry = -1.0
        self.circle_phase = 0.0
        self.circle_phase_speed_scale = 0.0
        self.circle_last_phase_ms = -1
        self.circle_close_start_ms = -1
        self.scan_votes = []


def resolve_target_ids(labels):
    ids = []
    for i, label in enumerate(labels):
        if label in TARGET_LABELS:
            ids.append(i)
    return ids


def resolve_center_target_ids(labels):
    ids = []
    for i, label in enumerate(labels):
        if label in ("target", "tgt"):
            ids.append(i)
    if len(ids) > 0:
        return ids
    return resolve_target_ids(labels)


def resolve_scan_target_ids(labels):
    ids = []
    for i, label in enumerate(labels):
        if label in SCAN_LABELS:
            ids.append(i)
    return ids


def obj_center(obj):
    return obj.x + obj.w // 2, obj.y + obj.h // 2


def is_locked(output_valid, tx_offset):
    if not output_valid or tx_offset is None:
        return False
    dx, dy = tx_offset
    return abs(dx) <= LOCK_DX and abs(dy) <= LOCK_DY


def is_scan_locked(output_valid, tx_offset):
    if not output_valid or tx_offset is None:
        return False
    dx, dy = tx_offset
    return abs(dx) <= SCAN_LOCK_DX and abs(dy) <= SCAN_LOCK_DY


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
        if obj.class_id in target_class_ids:
            objs.append(obj)
    objs.sort(key=lambda obj: obj_center(obj)[0])
    return objs


def make_scan_item(obj, order_index, rank_in_class):
    cx, cy = obj_center(obj)
    return {
        "class_id": obj.class_id,
        "order_index": order_index,
        "rank_in_class": rank_in_class,
        "cx": cx,
        "cy": cy,
        "w": obj.w,
        "h": obj.h,
    }


def scan_item_class_id(item):
    if isinstance(item, dict):
        return item.get("class_id", -1)
    return item


def scan_item_matches_selection(item, selected_class_id, selected_cx=-1, selected_cy=-1):
    class_id = scan_item_class_id(item)
    return selected_class_id == class_id


def best_obj_for_scan_item(detections, item, target_class_ids):
    class_id = scan_item_class_id(item)
    objs = []
    for obj in visible_objects_left_to_right(detections, target_class_ids):
        if obj.class_id == class_id:
            objs.append(obj)
    if len(objs) == 0:
        return None
    if isinstance(item, dict):
        rank = item.get("rank_in_class", 0)
        if rank < len(objs):
            return objs[rank]
    best = None
    for obj in objs:
        if best is None or obj.score > best.score:
            best = obj
    return best


def best_obj_for_class(detections, class_id):
    best = None
    for obj in detections:
        if obj.class_id != class_id:
            continue
        if best is None or obj.score > best.score:
            best = obj
    return best


def set_lock_point(runtime, obj):
    if obj is None:
        return
    cx, cy = obj_center(obj)
    runtime.lock_cx = cx
    runtime.lock_cy = cy
    runtime.lock_w = obj.w
    runtime.lock_h = obj.h
    runtime.lock_class_id = obj.class_id
    runtime.lock_score = obj.score


def clear_lock_point(runtime):
    runtime.lock_cx = -1
    runtime.lock_cy = -1
    runtime.lock_w = -1
    runtime.lock_h = -1
    runtime.lock_class_id = -1
    runtime.lock_score = 0.0


def has_lock_point(runtime):
    return runtime.lock_cx >= 0 and runtime.lock_cy >= 0 and runtime.lock_class_id >= 0


def locked_control_obj(runtime, box_factory):
    if not has_lock_point(runtime):
        return None
    return box_factory(
        runtime.lock_cx,
        runtime.lock_cy,
        max(1, runtime.lock_w),
        max(1, runtime.lock_h),
        runtime.lock_class_id,
        runtime.lock_score,
    )


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

    def override_control_obj(self, runtime, control_obj, output_valid, box_factory, now_ms, tx_offset=None, offset_fn=None):
        return control_obj

    def status_text(self, runtime, name, label_fn=None):
        if runtime.timeout:
            return "%s TIMEOUT" % name
        return "%s %s" % (name, runtime.status)
