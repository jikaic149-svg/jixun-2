from .base import TaskRuntime, resolve_target_ids
from .basic_center import BasicCenterTask
from .basic_scan_lr import BasicScanLeftRightTask
from .basic_reaim_center import BasicReaimCenterTask
from .ext_circle import ExtCircleTask
from .ext_memory_scan import ExtMemoryScanTask
from .constants import (
    MCU_STATE_RELAX,
    MCU_READY_STATES,
    MCU_SELECTABLE_STATES,
    MCU_RUN_TRIGGER_STATES,
    TASK_BASIC_CENTER,
    TASK_BASIC_REAIM_CENTER,
    TASK_BASIC_SCAN_LR,
    TASK_EXT_CIRCLE,
    TASK_EXT_MEMORY_SCAN,
    TASK_CENTER_REFINEMENT_TASKS,
    TASK_NAMES,
    TASK_ORDER,
)


class BTaskController:
    def __init__(self, labels):
        self.labels = labels
        self.target_class_ids = resolve_target_ids(labels)
        self.memory_order = []
        self.task_id = TASK_BASIC_CENTER
        self.selected = False
        self.running = False
        self.last_mcu_state = MCU_STATE_RELAX
        self.runtime = TaskRuntime()
        self.tasks = {
            TASK_BASIC_CENTER: BasicCenterTask(labels, self.target_class_ids),
            TASK_BASIC_SCAN_LR: BasicScanLeftRightTask(labels, self.target_class_ids),
            TASK_BASIC_REAIM_CENTER: BasicReaimCenterTask(labels, self.target_class_ids),
            TASK_EXT_CIRCLE: ExtCircleTask(labels, self.target_class_ids),
            TASK_EXT_MEMORY_SCAN: ExtMemoryScanTask(labels, self.target_class_ids),
        }
        self.runtime.reset(-1)

    def available_tasks(self):
        return TASK_ORDER

    def can_select(self, mcu_state):
        return mcu_state in MCU_SELECTABLE_STATES

    def is_ready_state(self, mcu_state):
        return mcu_state in MCU_READY_STATES

    def current_task(self):
        return self.tasks[self.task_id]

    def task_name(self):
        return TASK_NAMES.get(self.task_id, "Task")

    def needs_target_center_refinement(self):
        return self.selected and self.running and self.task_id in TASK_CENTER_REFINEMENT_TASKS

    def select_task(self, task_id, now_ms, mcu_state):
        if not self.can_select(mcu_state):
            self.runtime.status = "MCU NOT READY"
            return False
        self.task_id = task_id
        self.selected = True
        self.running = False
        self.runtime.reset(-1)
        if self.task_id == TASK_EXT_MEMORY_SCAN and len(self.memory_order) == 0:
            self.runtime.status = "NO MEMORY"
        else:
            self.runtime.status = "WAIT USER"
        return True

    def sync_mcu_state(self, mcu_state, now_ms):
        self.last_mcu_state = mcu_state
        if not self.selected:
            self.runtime.status = "SELECT TASK"
            return False
        if self.runtime.done:
            self.running = False
            return False
        if self.running:
            if self.is_ready_state(mcu_state):
                self.running = False
                self.runtime.status = "STOPPED"
            return False
        if mcu_state in MCU_RUN_TRIGGER_STATES:
            if self.task_id == TASK_EXT_MEMORY_SCAN and len(self.memory_order) == 0:
                self.runtime.status = "NO MEMORY"
                return False
            self.running = True
            self.runtime.reset(now_ms)
            self.runtime.status = "RUN"
            return True
        if self.is_ready_state(mcu_state):
            self.runtime.status = "WAIT USER"
        else:
            self.runtime.status = "WAIT STAB"
        return False

    def pick_target(self, detections, selected_class_id, now_ms):
        if not self.selected or not self.running:
            return None
        if self.runtime.done:
            self.running = False
            return None
        obj = self.current_task().pick_target(self.runtime, detections, selected_class_id, now_ms)
        if self.task_id == TASK_BASIC_SCAN_LR and len(self.runtime.sequence) > 0:
            self.memory_order = list(self.runtime.sequence)
            self.tasks[TASK_EXT_MEMORY_SCAN].memory_order = list(self.memory_order)
        return obj

    def update_lock(self, control_obj, output_valid, tx_offset, now_ms):
        if not self.selected or not self.running:
            return False
        if self.runtime.done:
            self.running = False
            return False
        task = self.current_task()
        task.update_lock(self.runtime, control_obj, output_valid, tx_offset, now_ms)
        if self.task_id == TASK_BASIC_SCAN_LR and self.runtime.done:
            self.memory_order = list(self.runtime.sequence)
            self.tasks[TASK_EXT_MEMORY_SCAN].memory_order = list(self.memory_order)
        advance_requested = self.runtime.advance_requested
        self.runtime.advance_requested = False
        if self.runtime.done:
            self.running = False
        return advance_requested

    def override_control_obj(self, control_obj, output_valid, box_factory, now_ms):
        if self.runtime.done:
            self.running = False
            return None
        if not self.selected or not self.running:
            return control_obj
        return self.current_task().override_control_obj(
            self.runtime,
            control_obj,
            output_valid,
            box_factory,
            now_ms,
        )

    def status_text(self, label_fn=None):
        if not self.selected:
            if self.runtime.status != "SELECT TASK":
                return self.runtime.status
            return "Select task"
        return self.current_task().status_text(self.runtime, self.task_name(), label_fn)
