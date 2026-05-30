
from maix import camera, display, image, nn, app, comm, touchscreen, time, uart
import struct, os
try:
    import _thread
except Exception:
    _thread = None
from trackers.roi_color_tracker import DynamicColorModel
from trackers.kalman_center_tracker import KalmanCenterTracker
from trackers.snap_gate import StrongSnapGate
from comm.udp_debug import UdpDebugSender
from task import BTaskController, TASK_NAMES
from task.constants import MCU_READY_STATES, MCU_STATE_CALIBRATING, TASK_BASIC_REAIM_CENTER, TASK_EXT_CIRCLE
from task.target_center import TargetCenterFinder

try:
    from maix import pinmap
except Exception:
    pinmap = None

report_on = True
APP_CMD_DETECT_RES = 0x02
CAMERA_WIDTH = 960
CAMERA_HEIGHT = 720
DIGITAL_ZOOM_SCALE = 1.60
DIGITAL_ZOOM_CENTER_X = 0.50
DIGITAL_ZOOM_CENTER_Y = 0.50
UI_SCALE = 1
PANEL_TEXT_SCALE = 2
UART_DEV = "/dev/ttyS2"
UART_BAUD = 115200
TARGET_SEND_PERIOD_MS = 15
DONE_SEND_REPEAT = 4
STAT_LOG_PERIOD_MS = 1000
UDP_DEBUG_ENABLE = True
UDP_DEBUG_HOST = "255.255.255.255"
UDP_DEBUG_PORT = 5005
UDP_DEBUG_PERIOD_MS = 30
CONSOLE_DEBUG_ENABLE = False
CONSOLE_DEBUG_PERIOD_MS = 500
TASK_DEBUG_ENABLE = True
TASK_DEBUG_CONSOLE_ENABLE = False
TASK_DEBUG_UDP_ENABLE = True
TASK_DEBUG_PERIOD_MS = 300
SAFE_TX_MAX_STEP_X = 75
SAFE_TX_MAX_STEP_Y = 62
SAFE_TX_MAX_ABS_X = 160
SAFE_TX_MAX_ABS_Y = 130
MISSING_TARGET_HOLD_MS = 300
TARGET_OUTPUT_SCALE_X = 0.70
TARGET_OUTPUT_SCALE_Y = 0.70
TARGET_OUTPUT_POLARITY_X = 1
TARGET_OUTPUT_POLARITY_Y = 1
TARGET_OUTPUT_DEADBAND_X = 2
TARGET_OUTPUT_DEADBAND_Y = 2
CIRCLE_TX_MAX_STEP_X = 72
CIRCLE_TX_MAX_STEP_Y = 62
CIRCLE_OUTPUT_SCALE_X = 0.95
CIRCLE_OUTPUT_SCALE_Y = 0.90
CIRCLE_OUTPUT_POLARITY_X = TARGET_OUTPUT_POLARITY_X
CIRCLE_OUTPUT_POLARITY_Y = TARGET_OUTPUT_POLARITY_Y
CIRCLE_OUTPUT_DEADBAND_X = 0.4
CIRCLE_OUTPUT_DEADBAND_Y = 0.4
CIRCLE_TX_INTERP_MAX_MS = 45
CIRCLE_TX_STALE_MS = 220
CIRCLE_TX_KF_PROCESS_VAR = 120.0
CIRCLE_TX_KF_MEASURE_VAR = 9.0
CIRCLE_TX_KF_MAX_SPEED = 260.0
SAFE_TX_JUMP_REJECT_PIXELS = 70
SAFE_TX_JUMP_CONFIRM_FRAMES = 5
SAFE_TX_JUMP_CONFIRM_TOLERANCE_PIXELS = 28
SAFE_TX_REVERSE_MIN_PIXELS = 18
SAFE_TX_REVERSE_CONFIRM_FRAMES = 7
CENTER_REFINE_START_GATE_MS = 350
CENTER_REFINE_CONFIRM_FRAMES = 2
CENTER_REFINE_MAX_JUMP_PIXELS = 72
CENTER_REFINE_MAX_JUMP_RATIO = 0.38
CENTER_REFINE_ERROR_GROW_PIXELS = 42
CENTER_REFINE_CENTER_ESCAPE_PIXELS = 30
PREAIM_VALID_MS = 5000
CONF_TH = 0.25
IOU_TH = 0.45
YOLO_TRACK_INTERVAL_FRAMES = 1
MIN_BOX_WIDTH = 21
MIN_BOX_HEIGHT = 21
MIN_SCORE_TO_DRAW = 0.25
BANANA_MIN_SCORE_TO_DRAW = 0.18
MAX_OBJS = 8
TRACK_MIN_SCORE = 0.25
BANANA_TRACK_MIN_SCORE = 0.16
LOST_HOLD_FRAMES = 15
TRACK_ACCEPT_MIN_SCORE = 0.32
TRACK_ACCEPT_MAX_CENTER_JUMP = 150
TRACK_ACCEPT_MAX_SIZE_DELTA_RATIO = 0.65
LASER_MASK_ENABLE = False
LASER_MASK_RADIUS = 34
LASER_MASK_MAX_BOX_SIZE = 42
LASER_INPUT_MASK_ENABLE = False
LASER_INPUT_MASK_RADIUS = 7
LASER_CIRCLE_MASK_RADIUS = 12
LASER_INPUT_SAMPLE_PAD = 5
TOUCH_DEBOUNCE_MS = 220
SMOOTH_ALPHA = 0.15
OUTPUT_CENTER_DEADBAND_PIXELS = 3
RECT_JITTER_DEADBAND_PIXELS = 3
RECT_SIZE_JITTER_DEADBAND_PIXELS = 3
MAX_MATCH_DIST = 80
LOST_MATCH_GROW = 6
LOCK_REACQUIRE_AFTER_FRAMES = 12
REACQUIRE_MAX_DIST = 180
ANCHOR_STABLE_FRAMES = 8
ANCHOR_STABLE_EPS = 4
SIZE_COST_WEIGHT = 0.8
SCORE_COST_WEIGHT = 0.6
VELOCITY_ALPHA = 0.50
CAM_MOTION_COMP_ENABLE = False
CAM_MOTION_COMP_YAW_PX_PER_DEG = 18.0
CAM_MOTION_COMP_PITCH_PX_PER_DEG = 18.0
CAM_MOTION_COMP_YAW_SIGN = -1.0
CAM_MOTION_COMP_PITCH_SIGN = -1.0
CAM_MOTION_COMP_MAX_SHIFT_X = 180
CAM_MOTION_COMP_MAX_SHIFT_Y = 140
ENABLE_COLOR_REFINE = True
ROI_MARGIN = 12
ROI_TRACK_ENABLE = False
ROI_TRACK_SCALE = 2.20
ROI_TRACK_MIN_PAD = 24
ROI_TRACK_MIN_CENTER_DIST = 38
ROI_TRACK_MAX_CENTER_DIST_SCALE = 1.20
ROI_TRACK_MAX_SIZE_DELTA_RATIO = 0.90
ROI_TRACK_MIN_AREA_RATIO = 0.12
ROI_TRACK_MAX_AREA_RATIO = 3.20
ROI_TRACK_MAX_ASPECT_DELTA_RATIO = 1.25
ROI_TRACK_EDGE_REJECT_MARGIN = 2
ROI_TRACK_AMBIGUOUS_COST_RATIO = 1.55
ROI_TRACK_AMBIGUOUS_COST_MARGIN = 900
COLOR_PIXELS_THRESHOLD = 56
COLOR_AREA_THRESHOLD = 56
COLOR_MERGE_MARGIN = 3
MIN_COLOR_BOX_W = 9
MIN_COLOR_BOX_H = 9
COLOR_MIN_BLOB_INSIDE = 0.45
COLOR_REFINE_PAD = 5
COLOR_MAX_CENTER_DIST_SCALE = 0.85
COLOR_LEARN_ROI_SCALE = 0.45
COLOR_LEARN_MIN_SIZE = 12
COLOR_L_MARGIN = 24
COLOR_A_MARGIN = 18
COLOR_B_MARGIN = 18
RECOVER_ROI_GROW = 90
RECOVER_MAX_CENTER_DIST = 130
SUSPECT_HOLD_FRAMES = 4
SUSPECT_WAIT_FRAMES = 45
YOLO_MISSING_HOLD_FRAMES = 20
E1_YOLO_MISSING_HOLD_FRAMES = 28
ROI_FAIL_TO_LOST_FRAMES = 90
ROI_AMBIGUOUS_TO_LOST_FRAMES = 2
YOLO_VERIFY_MAX_MISS_FRAMES = 5
YOLO_VERIFY_MAX_CENTER_DIST = 90
YOLO_VERIFY_MAX_SIZE_DELTA_RATIO = 1.00
YOLO_TRACK_FALLBACK_MAX_DIST = 90
YOLO_TRACK_FALLBACK_MAX_SIZE_DELTA_RATIO = 0.90
BANANA_YOLO_TRACK_MAX_DIST = 145
BANANA_YOLO_TRACK_MAX_SIZE_DELTA_RATIO = 1.45
BANANA_YOLO_MISSING_HOLD_FRAMES = 120
BANANA_TARGET_SWITCH_MAX_DIST = 130
BANANA_CV_TRACK_SCORE = 0.56
BANANA_CV_ROI_SCALE = 3.10
BANANA_CV_ROI_MIN_PAD = 36
BANANA_CV_YOLO_GATE_SCALE = 1.45
BANANA_CV_YOLO_GATE_MIN_PAD = 18
BANANA_CV_YOLO_GATE_HOLD_FRAMES = 90
BANANA_CV_HOLD_FRAMES = 140
BANANA_CV_SHORT_MISSING_HOLD_FRAMES = 6
BANANA_CV_YOLO_CENTER_RATIO = 0.70
BANANA_CV_YOLO_MIN_CENTER_DIST = 26
BANANA_CV_YOLO_MIN_OVERLAP_RATIO = 0.12
BANANA_CV_MAX_CENTER_STEP = 72
BANANA_CV_MAX_CENTER_STEP_SCALE = 0.85
BANANA_CV_MAX_SIZE_DELTA_RATIO = 0.85
BANANA_CV_MIN_AREA_RATIO = 0.04
BANANA_CV_MAX_AREA_RATIO = 2.60
BANANA_CV_MAX_ASPECT_DELTA_RATIO = 1.30
BANANA_CV_AMBIGUOUS_COST_RATIO = 1.70
BANANA_CV_AMBIGUOUS_COST_MARGIN = 1200
BANANA_CV_PIXELS_THRESHOLD = 14
BANANA_CV_AREA_THRESHOLD = 14
BANANA_CV_MERGE_MARGIN = 8
YOLO_OBSERVE_MAX_DIST = 220
BANANA_YOLO_OBSERVE_MAX_DIST = 260
YOLO_LAST_LOCK_MAX_JUMP = 80
YOLO_GATE_MIN_DIST = 110
YOLO_GATE_SCALE = 2.40
YOLO_REACQUIRE_CONFIRM_FRAMES = 3
YOLO_REACQUIRE_MAX_PENDING_DIST = 45
TARGET_SWITCH_MAX_DIST = 75
TARGET_SWITCH_CONFIRM_FRAMES = 6
TARGET_SWITCH_CONFIRM_PIXELS = 24
KALMAN_PROCESS_VAR = 180.0
KALMAN_MEASURE_VAR = 25.0
KALMAN_MAX_SPEED = 70.0
KALMAN_MAX_PREDICT_FRAMES = 12
KALMAN_CONTROL_MAX_MISS_FRAMES = 12
BANANA_KALMAN_CONTROL_MAX_MISS_FRAMES = 28
KALMAN_MISS_VELOCITY_DECAY = 0.30
KALMAN_MEASUREMENT_GATE_PIXELS = 70
KALMAN_GATE_CONFIRM_FRAMES = 4
KALMAN_GATE_CONFIRM_PIXELS = 28
E1_YOLO_FAST_TRACK = True
E1_YOLO_TRACK_MAX_DIST = 125
E1_YOLO_TRACK_MAX_SIZE_DELTA_RATIO = 0.90
E1_YOLO_LAST_LOCK_MAX_JUMP = 110
E1_YOLO_REACQUIRE_CONFIRM_FRAMES = 3
PANEL_X = 645
PANEL_W = CAMERA_WIDTH - PANEL_X
PANEL_PAD = 12
ITEM_TOP = 128
ITEM_H = 72
BOX_THICKNESS = 3
TEXT_SCALE = 2
AIM_CENTER_X = PANEL_X // 2
AIM_CENTER_Y = CAMERA_HEIGHT // 2
STATE_NAMES = ("RELAX", "ENC_LOCK", "CALIB", "IMU_STAB", "GRAV_CAL", "IDLE", "ANGLE_TEST", "TORQUE_TEST", "VISION_STAB")

COLOR_BLACK = image.COLOR_BLACK
COLOR_WHITE = image.COLOR_WHITE
COLOR_RED = image.COLOR_RED
COLOR_YELLOW = image.COLOR_YELLOW
COLOR_PANEL_BG = image.Color.from_rgb(18, 18, 18)
COLOR_BUTTON_BG = image.Color.from_rgb(45, 45, 45)
COLOR_BUTTON_BORDER = image.Color.from_rgb(220, 220, 220)
COLOR_STATUS = image.Color.from_rgb(170, 220, 255)
COLOR_ITEM_BG = image.Color.from_rgb(36, 36, 36)
COLOR_ITEM_SELECTED_BG = image.Color.from_rgb(35, 72, 35)
COLOR_ITEM_BORDER = image.Color.from_rgb(120, 120, 120)
COLOR_DETAIL = image.Color.from_rgb(200, 200, 200)
COLOR_MUTED = image.Color.from_rgb(180, 180, 180)
COLOR_LOST = image.Color.from_rgb(255, 80, 80)
COLOR_BLUE = image.Color.from_rgb(0, 80, 255)
COLOR_BLUE_BRIGHT = image.Color.from_rgb(0, 220, 255)
zoom_warned = False

RED_THRESHOLDS = [[0, 90, 25, 80, 0, 80]]
YELLOW_THRESHOLDS = [[35, 100, -25, 35, 25, 90]]
GREEN_THRESHOLDS = [[0, 90, -120, -10, 0, 80]]
BLACK_THRESHOLDS = [[0, 45, -25, 25, -25, 25]]
COLOR_THRESHOLDS_BY_LABEL = {
    "apple": RED_THRESHOLDS,
    "strawberry": RED_THRESHOLDS,
    "tgt": BLACK_THRESHOLDS,
    "target": BLACK_THRESHOLDS,
    "triangle": RED_THRESHOLDS,
    "red_triangle": RED_THRESHOLDS,
    "square": GREEN_THRESHOLDS,
    "green_square": GREEN_THRESHOLDS,
    "trapezoid": YELLOW_THRESHOLDS,
    "yellow_trapezoid": YELLOW_THRESHOLDS,
    "banana": YELLOW_THRESHOLDS,
    "watermalon": GREEN_THRESHOLDS,
    "watermelon": GREEN_THRESHOLDS,
}
DISPLAY_LABEL_BY_LABEL = {
    "triangle": "red_triangle",
    "square": "green_square",
    "tgt": "target",
    "trapezoid": "yellow_trapezoid",
    "watermalon": "watermelon",
}
MODEL_NAME = "jixun2_final_manual25_banana_laser_20260529"
MODEL_FILE = "jixun2_final_manual25_banana_laser_20260529.mud"
EXPECTED_LABELS = (
    "apple",
    "banana",
    "strawberry",
    "watermelon",
    "target",
    "green_square",
    "red_triangle",
    "yellow_trapezoid",
)

CANDIDATE_MAX_LOST_FRAMES = 12
CANDIDATE_MATCH_DIST = 95
SELECT_CLEAR_CODE = -1
CALIBRATE_CODE = -2
CALIBRATE_TOUCH_CODE = -3
TASK_SELECT_CODE = -4
TASK_PAGE_CODE = -5
CALIB_PAGE_CODE = -6
UI_PAGE_TASK = 0
UI_PAGE_CALIB = 1
VSTATE_SEARCH = 0
VSTATE_TRACK = 1
VSTATE_SUSPECT = 2
VSTATE_LOST = 3
VSTATE_REACQUIRE = 4
VSTATE_NAMES = ("SEARCH", "TRACK", "SUSPECT", "LOST", "REACQUIRE")

selected_class_id = -1
selected_cx = -1
selected_cy = -1
selected_w = -1
selected_h = -1
selected_slot_index = -1
vision_track_state = VSTATE_SEARCH
last_good_cx = -1
last_good_cy = -1
last_good_w = -1
last_good_h = -1
last_yolo_cx = -1
last_yolo_cy = -1
last_yolo_w = -1
last_yolo_h = -1
last_yolo_seen_frames = 0
last_cv_cx = -1
last_cv_cy = -1
last_cv_w = -1
last_cv_h = -1
last_cv_seen_frames = 0
roi_fail_frames = 0
roi_ambiguous_frames = 0
suspect_frames = 0
reacquire_confirm_frames = 0
pending_reacquire_cx = -1
pending_reacquire_cy = -1
pending_reacquire_w = -1
pending_reacquire_h = -1
anchor_frozen = False
anchor_stable_frames = 0
aim_ref_x = AIM_CENTER_X
aim_ref_y = AIM_CENTER_Y
aim_ref_pitch = 0.0
aim_ref_yaw = 0.0
aim_calibrated = False
track_ref_pitch = 0.0
track_ref_yaw = 0.0
track_vx = 0
track_vy = 0
touch_last_pressed = False
touch_x = 0
touch_y = 0
last_touch_action_ms = -TOUCH_DEBOUNCE_MS
lost_frames = 0
smooth_x = -1
smooth_y = -1
smooth_w = -1
smooth_h = -1
smooth_score = 0
dynamic_color_model = DynamicColorModel(
    COLOR_LEARN_ROI_SCALE,
    COLOR_LEARN_MIN_SIZE,
    COLOR_L_MARGIN,
    COLOR_A_MARGIN,
    COLOR_B_MARGIN,
)
kalman_tracker = KalmanCenterTracker(
    KALMAN_PROCESS_VAR,
    KALMAN_MEASURE_VAR,
    KALMAN_MAX_SPEED,
    KALMAN_MAX_PREDICT_FRAMES,
    PANEL_X,
    CAMERA_HEIGHT,
    KALMAN_MISS_VELOCITY_DECAY,
)
strong_snap_gate = StrongSnapGate(
    KALMAN_MEASUREMENT_GATE_PIXELS,
    KALMAN_GATE_CONFIRM_FRAMES,
    KALMAN_GATE_CONFIRM_PIXELS,
)
target_switch_gate = StrongSnapGate(
    TARGET_SWITCH_MAX_DIST,
    TARGET_SWITCH_CONFIRM_FRAMES,
    TARGET_SWITCH_CONFIRM_PIXELS,
)
candidate_slots = []
serial = None
line_buf = bytearray()
boot_ms = time.ticks_ms()
last_target_tx_ms = 0
last_stat_log_ms = 0
last_tx_packet = None
last_valid_tx_ms = -1
last_valid_tx_flag = 1
safe_tx_dx = 0
safe_tx_dy = 0
safe_tx_jump_count = 0
safe_tx_pending_dx = 0
safe_tx_pending_dy = 0
circle_tx_residual_x = 0.0
circle_tx_residual_y = 0.0
uart_lock = _thread.allocate_lock() if _thread is not None else None
circle_tx_lock = _thread.allocate_lock() if _thread is not None else None
circle_tx_thread_started = False
circle_tx_enabled = False
circle_tx_last_send_ms = 0
done_signal_pending = 0
task_done_latched = False
yolo_frame_div = 0
udp_debug = UdpDebugSender(UDP_DEBUG_ENABLE, UDP_DEBUG_HOST, UDP_DEBUG_PORT)
last_udp_debug_ms = 0
last_console_debug_ms = 0
last_task_debug_ms = 0
last_loop_tick_ms = -1
loop_dt_ema_ms = 0.0
app_fps = 0.0
last_yolo_tick_ms = -1
yolo_dt_ema_ms = 0.0
yolo_fps = 0.0
last_yolo_infer_ms = 0
task_controller = None
ui_page = UI_PAGE_TASK
target_center_finder = None
target_center_result = None
preaim_valid = False
preaim_dx = 0
preaim_dy = 0
preaim_cx = -1
preaim_cy = -1
preaim_dir = "CC"
preaim_updated_ms = -1
center_refine_gate_until_ms = -1
center_refine_confirm_frames = 0
center_refine_gate_reason = "idle"

stat = {
    "time_ms": 0,
    "pitch_abs": 0.0,
    "yaw_abs": 0.0,
    "pitch_enc": 0.0,
    "pitch_current": 0.0,
    "yaw_current": 0.0,
    "dx": 0,
    "dy": 0,
    "state": 0,
    "key": 1,
    "key_raw": 1,
    "key_hold_ms": 0,
    "vision_active": 0,
    "vision_pitch_offset": 0.0,
    "vision_yaw_offset": 0.0,
    "vision_input_dx": 0.0,
    "vision_input_dy": 0.0,
    "vision_predict_dx": 0.0,
    "vision_predict_dy": 0.0,
    "vision_predict_ms": 0,
    "rx": 0,
    "tx": 0,
}

class TrackedObj:
    def __init__(self, x, y, w, h, class_id, score):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.class_id = class_id
        self.score = score

def ticks():
    return int(time.ticks_ms() - boot_ms)

def log(msg):
    print("[T:%06d] %s" % (ticks(), msg))

def state_name():
    state = stat["state"]
    if state in MCU_READY_STATES:
        return "READY"
    if 0 <= state < len(STATE_NAMES):
        return STATE_NAMES[state]
    return "UNKNOWN"

def vision_state_name():
    if 0 <= vision_track_state < len(VSTATE_NAMES):
        return VSTATE_NAMES[vision_track_state]
    return "UNKNOWN"

def set_vision_state(new_state, reason = ""):
    global vision_track_state
    if vision_track_state != new_state:
        if reason:
            log("vision %s -> %s: %s" % (vision_state_name(), VSTATE_NAMES[new_state], reason))
        else:
            log("vision %s -> %s" % (vision_state_name(), VSTATE_NAMES[new_state]))
    vision_track_state = new_state

def reset_reacquire_pending():
    global reacquire_confirm_frames
    global pending_reacquire_cx, pending_reacquire_cy, pending_reacquire_w, pending_reacquire_h
    reacquire_confirm_frames = 0
    pending_reacquire_cx = -1
    pending_reacquire_cy = -1
    pending_reacquire_w = -1
    pending_reacquire_h = -1

def reset_track_identity_gates():
    strong_snap_gate.reset()
    target_switch_gate.reset()

def is_e1_fast_tracking():
    return (
        E1_YOLO_FAST_TRACK and
        task_controller is not None and
        task_controller.task_id == TASK_EXT_CIRCLE and
        task_controller.running)

def update_loop_fps(now_ms):
    global last_loop_tick_ms, loop_dt_ema_ms, app_fps
    if last_loop_tick_ms >= 0:
        dt = max(1, now_ms - last_loop_tick_ms)
        if loop_dt_ema_ms <= 0.0:
            loop_dt_ema_ms = float(dt)
        else:
            loop_dt_ema_ms = loop_dt_ema_ms * 0.80 + float(dt) * 0.20
        app_fps = 1000.0 / loop_dt_ema_ms
    last_loop_tick_ms = now_ms

def mark_yolo_timing(start_ms, end_ms):
    global last_yolo_tick_ms, yolo_dt_ema_ms, yolo_fps, last_yolo_infer_ms
    infer_ms = max(0, end_ms - start_ms)
    last_yolo_infer_ms = infer_ms
    if last_yolo_tick_ms >= 0:
        dt = max(1, end_ms - last_yolo_tick_ms)
        if yolo_dt_ema_ms <= 0.0:
            yolo_dt_ema_ms = float(dt)
        else:
            yolo_dt_ema_ms = yolo_dt_ema_ms * 0.80 + float(dt) * 0.20
        yolo_fps = 1000.0 / yolo_dt_ema_ms
    last_yolo_tick_ms = end_ms

def circle_kalman_params():
    if target_center_finder is None:
        return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0
    tracker = getattr(target_center_finder, "circle_kalman", None)
    if tracker is None:
        return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0
    return (
        tracker.process_var,
        tracker.radius_process_var,
        tracker.measure_var,
        tracker.radius_measure_var,
        tracker.max_speed,
        tracker.max_radius_speed,
        tracker.max_predict_frames,
        tracker.miss_velocity_decay,
    )

def angle_delta_deg(current, ref):
    delta = current - ref
    if delta > 180.0:
        delta -= 360.0
    elif delta < -180.0:
        delta += 360.0
    return delta

def camera_motion_shift_from_ref():
    if not CAM_MOTION_COMP_ENABLE:
        return 0, 0

    dyaw = angle_delta_deg(stat["yaw_abs"], track_ref_yaw)
    dpitch = stat["pitch_abs"] - track_ref_pitch
    shift_x = int(dyaw * CAM_MOTION_COMP_YAW_PX_PER_DEG * CAM_MOTION_COMP_YAW_SIGN)
    shift_y = int(dpitch * CAM_MOTION_COMP_PITCH_PX_PER_DEG * CAM_MOTION_COMP_PITCH_SIGN)
    shift_x = clamp_abs(shift_x, CAM_MOTION_COMP_MAX_SHIFT_X)
    shift_y = clamp_abs(shift_y, CAM_MOTION_COMP_MAX_SHIFT_Y)
    return shift_x, shift_y

def motion_compensated_track_ref():
    if selected_cx < 0 or selected_cy < 0:
        return selected_cx, selected_cy

    shift_x, shift_y = camera_motion_shift_from_ref()
    return (clamp(selected_cx + shift_x, 0, PANEL_X - 1),
            clamp(selected_cy + shift_y, 0, CAMERA_HEIGHT - 1))

def init_uart():
    global serial
    if pinmap:
        try:
            pinmap.set_pin_function("B0", "UART2_TX")
            pinmap.set_pin_function("B1", "UART2_RX")
        except Exception:
            pass
    try:
        serial = uart.UART(UART_DEV, UART_BAUD)
        log("uart %s %d" % (UART_DEV, UART_BAUD))
    except Exception as e:
        serial = None
        log("uart init failed: %s" % e)

def uart_write(text):
    if serial is not None:
        if uart_lock is not None:
            uart_lock.acquire()
        try:
            serial.write(text.encode("ascii"))
        finally:
            if uart_lock is not None:
                uart_lock.release()
        stat["tx"] += len(text)

def parse_line(line):
    global last_stat_log_ms

    idx = line.find("M,STAT,")
    if idx > 0:
        line = line[idx:]
    next_idx = line.find("M,STAT,", 1)
    if next_idx > 0:
        line = line[:next_idx]
    if line == "U6BOOT" or line == "PONG":
        return

    parts = line.split(",")
    if len(parts) >= 10 and parts[0] == "M" and parts[1] == "STAT":
        try:
            next_stat = dict(stat)
            next_stat["time_ms"] = int(parts[2])
            if len(parts) >= 16:
                next_stat["state"] = int(parts[3])
                next_stat["dx"] = int(parts[4])
                next_stat["dy"] = int(parts[5])
                next_stat["pitch_abs"] = float(parts[9])
                next_stat["yaw_abs"] = float(parts[10])
                next_stat["pitch_current"] = float(parts[11])
                next_stat["yaw_current"] = float(parts[12])
                next_stat["vision_active"] = int(parts[13])
                next_stat["vision_pitch_offset"] = float(parts[14])
                next_stat["vision_yaw_offset"] = float(parts[15])
                if len(parts) >= 21:
                    next_stat["vision_input_dx"] = float(parts[16])
                    next_stat["vision_input_dy"] = float(parts[17])
                    next_stat["vision_predict_dx"] = float(parts[18])
                    next_stat["vision_predict_dy"] = float(parts[19])
                    next_stat["vision_predict_ms"] = int(parts[20])
            else:
                next_stat["pitch_abs"] = float(parts[3])
                next_stat["yaw_abs"] = float(parts[4])
                next_stat["pitch_enc"] = float(parts[5])
                next_stat["pitch_current"] = float(parts[6])
                next_stat["yaw_current"] = float(parts[7])
                next_stat["dx"] = int(parts[8])
                next_stat["dy"] = int(parts[9])
                if len(parts) >= 12:
                    next_stat["state"] = int(parts[10])
                    next_stat["key"] = int(parts[11])
                if len(parts) >= 14:
                    next_stat["key_raw"] = int(parts[12])
                    next_stat["key_hold_ms"] = int(parts[13])
                if len(parts) >= 17:
                    next_stat["vision_pitch_offset"] = float(parts[14])
                    next_stat["vision_yaw_offset"] = float(parts[15])
                    next_stat["vision_active"] = int(parts[16])
        except Exception as e:
            log("bad stat: %s line=%s" % (e, line))
            return
        stat.update(next_stat)

        now = time.ticks_ms()
        if now - last_stat_log_ms >= STAT_LOG_PERIOD_MS:
            last_stat_log_ms = now
            log("t=%lu state=%s(%d) key=%d raw=%d hold=%lu pitch=%.2f enc=%.2f yaw=%.2f ip=%.2f iy=%.2f dx=%d dy=%d vi=%.1f,%.1f pr=%.1f,%.1f,%d va=%d vpo=%.3f vyo=%.3f" %
                (stat["time_ms"], state_name(), stat["state"],
                 stat["key"], stat["key_raw"], stat["key_hold_ms"],
                 stat["pitch_abs"], stat["pitch_enc"],
                 stat["yaw_abs"], stat["pitch_current"], stat["yaw_current"],
                 stat["dx"], stat["dy"],
                 stat["vision_input_dx"], stat["vision_input_dy"],
                 stat["vision_predict_dx"], stat["vision_predict_dy"],
                 stat["vision_predict_ms"],
                 stat["vision_active"], stat["vision_pitch_offset"], stat["vision_yaw_offset"]))

def poll_uart():
    if serial is None:
        return
    if uart_lock is not None:
        uart_lock.acquire()
    try:
        data = serial.read()
    finally:
        if uart_lock is not None:
            uart_lock.release()
    if not data:
        return
    raw = bytes(data)
    stat["rx"] += len(raw)
    for b in raw:
        if b == 10:
            line = bytes(line_buf).decode("ascii", "ignore").strip()
            line_buf.clear()
            if line:
                parse_line(line)
        elif b != 13:
            if len(line_buf) < 160:
                line_buf.append(b)
            else:
                line_buf.clear()

def encode_objs(objs):
    '''
        encode objs info to bytes body for protocol
        2B x(LE) + 2B y(LE) + 2B w(LE) + 2B h(LE) + 2B idx + 4B score(float) ...
    '''
    body = b''
    for obj in objs:
        body += struct.pack("<hhHHHf", obj.x, obj.y, obj.w, obj.h, obj.class_id, obj.score)
    return body

def clamp_u16(value):
    value = int(value)
    if value < 0:
        return 0
    if value > 65535:
        return 65535
    return value

def encode_circle_kalman_debug(result):
    if result is None:
        return b''

    body_objs = []
    valid_score = 0.95 if result.valid else 0.10
    body_objs.append(TrackedObj(
        clamp(result.cx, -32768, 32767),
        clamp(result.cy, -32768, 32767),
        clamp_u16(result.radius),
        clamp_u16(result.width),
        0,
        valid_score,
    ))
    if result.meas_cx >= 0 and result.meas_cy >= 0:
        body_objs.append(TrackedObj(
            clamp(result.meas_cx, -32768, 32767),
            clamp(result.meas_cy, -32768, 32767),
            clamp_u16(result.meas_r),
            clamp_u16(result.meas_w),
            1,
            0.90,
        ))
    if result.kalman is not None:
        ready, source, kx, ky, kr, vx, vy, vr, ix, iy, ir, miss = result.kalman
        body_objs.append(TrackedObj(
            clamp(int(vx * 10.0), -32768, 32767),
            clamp(int(vy * 10.0), -32768, 32767),
            clamp_u16(int(abs(vr) * 10.0)),
            clamp_u16(miss),
            2,
            0.90 if ready else 0.10,
        ))
        body_objs.append(TrackedObj(
            clamp(int(ix * 10.0), -32768, 32767),
            clamp(int(iy * 10.0), -32768, 32767),
            clamp_u16(int(abs(ir) * 10.0)),
            clamp_u16(len(str(source))),
            3,
            0.90 if ready else 0.10,
        ))
    return encode_objs(body_objs)

def target_offset(obj):
    if obj is None:
        return None
    cx, cy = obj_center(obj)
    ax, ay = aim_point()
    return cx - ax, cy - ay

class CircleTxInterpolator:
    def __init__(self):
        self.reset()

    def reset(self):
        self.ready = False
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.px = 1.0
        self.py = 1.0
        self.pvx = 200.0
        self.pvy = 200.0
        self.last_ms = -1

    def _limit_velocity(self):
        self.vx = clamp(self.vx, -CIRCLE_TX_KF_MAX_SPEED, CIRCLE_TX_KF_MAX_SPEED)
        self.vy = clamp(self.vy, -CIRCLE_TX_KF_MAX_SPEED, CIRCLE_TX_KF_MAX_SPEED)

    def _predict_to(self, now_ms):
        if not self.ready or self.last_ms < 0:
            self.last_ms = now_ms
            return
        dt_ms = max(0, min(CIRCLE_TX_INTERP_MAX_MS, now_ms - self.last_ms))
        if dt_ms <= 0:
            return
        dt = float(dt_ms) * 0.001
        self.x += self.vx * dt
        self.y += self.vy * dt
        q = CIRCLE_TX_KF_PROCESS_VAR * dt
        self.px += (self.pvx * dt * dt) + q
        self.py += (self.pvy * dt * dt) + q
        self.pvx += q
        self.pvy += q
        self.last_ms = now_ms

    def update(self, dx, dy, now_ms):
        dx = float(dx)
        dy = float(dy)
        if not self.ready:
            self.ready = True
            self.x = dx
            self.y = dy
            self.vx = 0.0
            self.vy = 0.0
            self.px = CIRCLE_TX_KF_MEASURE_VAR
            self.py = CIRCLE_TX_KF_MEASURE_VAR
            self.pvx = 200.0
            self.pvy = 200.0
            self.last_ms = now_ms
            return
        prev_ms = self.last_ms
        self._predict_to(now_ms)
        innov_x = dx - self.x
        innov_y = dy - self.y
        r = CIRCLE_TX_KF_MEASURE_VAR
        kx = self.px / (self.px + r)
        ky = self.py / (self.py + r)
        dt_ms = max(1, min(CIRCLE_TX_INTERP_MAX_MS, now_ms - prev_ms if prev_ms >= 0 else TARGET_SEND_PERIOD_MS))
        dt = float(dt_ms) * 0.001
        self.x += innov_x * kx
        self.y += innov_y * ky
        self.vx += (innov_x / dt) * kx * 0.28
        self.vy += (innov_y / dt) * ky * 0.28
        self.px *= (1.0 - kx)
        self.py *= (1.0 - ky)
        self._limit_velocity()
        self.last_ms = now_ms

    def predict(self, now_ms):
        if not self.ready or self.last_ms < 0:
            return None
        if now_ms - self.last_ms > CIRCLE_TX_STALE_MS:
            self.reset()
            return None
        self._predict_to(now_ms)
        return self.x, self.y

circle_tx_interp = CircleTxInterpolator()

def direction_hint(dx, dy):
    if dx > TARGET_OUTPUT_DEADBAND_X:
        x_hint = "R"
    elif dx < -TARGET_OUTPUT_DEADBAND_X:
        x_hint = "L"
    else:
        x_hint = "C"
    if dy > TARGET_OUTPUT_DEADBAND_Y:
        y_hint = "D"
    elif dy < -TARGET_OUTPUT_DEADBAND_Y:
        y_hint = "U"
    else:
        y_hint = "C"
    return x_hint + y_hint

def axis_sign(value, deadband):
    if value > deadband:
        return 1
    if value < -deadband:
        return -1
    return 0

def update_preaim_from_obj(obj, now_ms):
    global preaim_valid, preaim_dx, preaim_dy, preaim_cx, preaim_cy, preaim_dir, preaim_updated_ms
    if obj is None:
        return
    offset = target_offset(obj)
    if offset is None:
        return
    preaim_dx, preaim_dy = offset
    preaim_cx, preaim_cy = obj_center(obj)
    preaim_dir = direction_hint(preaim_dx, preaim_dy)
    preaim_updated_ms = now_ms
    preaim_valid = True

def update_preaim_if_waiting(obj, now_ms):
    if task_controller is None or not task_controller.selected or task_controller.running:
        return
    if stat["state"] not in (MCU_STATE_CALIBRATING,) and stat["state"] not in MCU_READY_STATES:
        return
    update_preaim_from_obj(obj, now_ms)

def best_preaim_candidate(detections):
    if task_controller is None:
        return None
    target_ids = getattr(task_controller, "target_class_ids", ())
    best = None
    for obj in detections:
        if obj.class_id not in target_ids:
            continue
        if best is None or obj.score > best.score:
            best = obj
    return best

def begin_center_refine_start_gate(now_ms):
    global center_refine_gate_until_ms, center_refine_confirm_frames, center_refine_gate_reason
    center_refine_gate_until_ms = now_ms + CENTER_REFINE_START_GATE_MS
    center_refine_confirm_frames = 0
    center_refine_gate_reason = "start"

def reset_preaim():
    global preaim_valid, preaim_dx, preaim_dy, preaim_cx, preaim_cy, preaim_dir, preaim_updated_ms
    preaim_valid = False
    preaim_dx = 0
    preaim_dy = 0
    preaim_cx = -1
    preaim_cy = -1
    preaim_dir = "CC"
    preaim_updated_ms = -1

def reset_center_refine_gate():
    global center_refine_gate_until_ms, center_refine_confirm_frames, center_refine_gate_reason
    center_refine_gate_until_ms = -1
    center_refine_confirm_frames = 0
    center_refine_gate_reason = "idle"

def preaim_recent(now_ms):
    return preaim_valid and preaim_updated_ms >= 0 and now_ms - preaim_updated_ms <= PREAIM_VALID_MS

def reject_center_refine(result, reason):
    global center_refine_confirm_frames, center_refine_gate_reason
    center_refine_confirm_frames = 0
    center_refine_gate_reason = reason
    if result is not None:
        result.valid = False
        result.reason = "gate " + reason
    return False

def valid_roi(roi):
    return (
        roi is not None and
        len(roi) >= 4 and
        roi[2] > 0 and
        roi[3] > 0
    )

def point_in_roi(cx, cy, roi):
    if not valid_roi(roi):
        return False
    return (
        cx >= roi[0] and cx < roi[0] + roi[2] and
        cy >= roi[1] and cy < roi[1] + roi[3]
    )

def roi_diagonal_center(roi):
    if not valid_roi(roi):
        return None
    cx = roi[0] + roi[2] // 2
    cy = roi[1] + roi[3] // 2
    return (
        clamp(cx, roi[0], roi[0] + roi[2] - 1),
        clamp(cy, roi[1], roi[1] + roi[3] - 1),
    )

def is_circle_task():
    return task_controller is not None and task_controller.task_id == TASK_EXT_CIRCLE

def is_plain_running_task():
    return (
        task_controller is not None and
        task_controller.running and
        task_controller.task_id != TASK_EXT_CIRCLE
    )

def is_reaim_running_task():
    return (
        task_controller is not None and
        task_controller.running and
        task_controller.task_id == TASK_BASIC_REAIM_CENTER
    )

def task_has_lock_point():
    if task_controller is None:
        return False
    runtime = task_controller.runtime
    return (
        runtime.lock_cx >= 0 and
        runtime.lock_cy >= 0 and
        runtime.lock_class_id >= 0
    )

def task_is_lock_holding():
    if task_controller is None:
        return False
    return task_controller.runtime.hold_start_ms >= 0 or task_has_lock_point()

def should_yaw_search_for_target(control_obj):
    if task_controller is None:
        return False
    runtime = task_controller.runtime
    if not task_controller.selected or not task_controller.running:
        return False
    if runtime.done or control_obj is not None:
        return False
    if task_is_lock_holding():
        return False
    if is_reaim_running_task():
        return True
    if selected_class_id < 0:
        return True
    if vision_track_state in (VSTATE_LOST, VSTATE_REACQUIRE):
        return True
    return (
        vision_track_state == VSTATE_SUSPECT and
        suspect_frames > yolo_missing_hold_frames()
    )

def normalize_center_result_for_task(result):
    if result is None or not result.valid:
        return
    if is_circle_task():
        return
    center = roi_diagonal_center(result.roi)
    if center is not None:
        result.cx = center[0]
        result.cy = center[1]

def accept_center_refine(base_obj, result, now_ms):
    global center_refine_confirm_frames, center_refine_gate_reason
    if result is None or not result.valid:
        center_refine_confirm_frames = 0
        center_refine_gate_reason = "none"
        return False
    if base_obj is None:
        return reject_center_refine(result, "no base")
    if result.roi is not None:
        if not valid_roi(result.roi):
            return reject_center_refine(result, "roi")
        if not point_in_roi(result.cx, result.cy, result.roi):
            return reject_center_refine(result, "roi")

    base_cx, base_cy = obj_center(base_obj)
    jump_x = result.cx - base_cx
    jump_y = result.cy - base_cy
    max_jump = max(CENTER_REFINE_MAX_JUMP_PIXELS,
                   int(max(base_obj.w, base_obj.h) * CENTER_REFINE_MAX_JUMP_RATIO))
    if jump_x * jump_x + jump_y * jump_y > max_jump * max_jump:
        return reject_center_refine(result, "jump")
    if abs(jump_x) > base_obj.w // 2 + CENTER_REFINE_CENTER_ESCAPE_PIXELS:
        return reject_center_refine(result, "escape x")
    if abs(jump_y) > base_obj.h // 2 + CENTER_REFINE_CENTER_ESCAPE_PIXELS:
        return reject_center_refine(result, "escape y")

    in_start_gate = now_ms <= center_refine_gate_until_ms
    if in_start_gate and preaim_recent(now_ms):
        base_offset = target_offset(base_obj)
        if base_offset is not None:
            base_dx, base_dy = base_offset
            cand_dx, cand_dy = result.cx - aim_point()[0], result.cy - aim_point()[1]
            pre_sx = axis_sign(preaim_dx, TARGET_OUTPUT_DEADBAND_X)
            pre_sy = axis_sign(preaim_dy, TARGET_OUTPUT_DEADBAND_Y)
            cand_sx = axis_sign(cand_dx, TARGET_OUTPUT_DEADBAND_X)
            cand_sy = axis_sign(cand_dy, TARGET_OUTPUT_DEADBAND_Y)
            if pre_sx != 0 and cand_sx != 0 and pre_sx != cand_sx:
                return reject_center_refine(result, "rev x")
            if pre_sy != 0 and cand_sy != 0 and pre_sy != cand_sy:
                return reject_center_refine(result, "rev y")
            if cand_sx != 0 and cand_sx == axis_sign(base_dx, TARGET_OUTPUT_DEADBAND_X):
                if abs(cand_dx) > abs(base_dx) + CENTER_REFINE_ERROR_GROW_PIXELS:
                    return reject_center_refine(result, "grow x")
            if cand_sy != 0 and cand_sy == axis_sign(base_dy, TARGET_OUTPUT_DEADBAND_Y):
                if abs(cand_dy) > abs(base_dy) + CENTER_REFINE_ERROR_GROW_PIXELS:
                    return reject_center_refine(result, "grow y")

    if in_start_gate:
        center_refine_confirm_frames += 1
        if center_refine_confirm_frames < CENTER_REFINE_CONFIRM_FRAMES:
            center_refine_gate_reason = "confirm"
            result.valid = False
            result.reason = "gate confirm"
            return False

    center_refine_gate_reason = "ok"
    return True

def normalize_circle_center_result(result):
    if result is None or not result.valid:
        return
    shape_radius = max(result.width, result.height) // 2
    if shape_radius <= 0:
        return
    if result.radius < shape_radius:
        result.radius = shape_radius

def target_center_point(obj):
    if obj is None:
        return None
    if not is_circle_task():
        roi = [obj.x, obj.y, obj.w, obj.h]
        center = roi_diagonal_center(roi)
        if center is not None:
            return center
    cx, cy = obj_center(obj)
    return clamp(cx, 0, PANEL_X - 1), clamp(cy, 0, CAMERA_HEIGHT - 1)

def aim_point():
    if not aim_calibrated:
        return AIM_CENTER_X, AIM_CENTER_Y
    return clamp(aim_ref_x, 0, PANEL_X - 1), clamp(aim_ref_y, 0, CAMERA_HEIGHT - 1)

def limit_step(current, target, max_step):
    delta = target - current
    if delta > max_step:
        return current + max_step
    if delta < -max_step:
        return current - max_step
    return target

def clamp_abs(value, max_abs):
    if value > max_abs:
        return max_abs
    if value < -max_abs:
        return -max_abs
    return value

def apply_output_deadband(value, deadband):
    if value > deadband:
        return value - deadband
    if value < -deadband:
        return value + deadband
    return 0

def is_axis_reversal(current, target, min_pixels):
    return (
        current > min_pixels and target < -min_pixels or
        current < -min_pixels and target > min_pixels
    )

def target_offset_for_mcu(obj):
    offset = target_offset(obj)
    if offset is None:
        return None
    raw_dx, raw_dy = offset
    raw_dx = int(apply_output_deadband(raw_dx, TARGET_OUTPUT_DEADBAND_X) * TARGET_OUTPUT_SCALE_X * TARGET_OUTPUT_POLARITY_X)
    raw_dy = int(apply_output_deadband(raw_dy, TARGET_OUTPUT_DEADBAND_Y) * TARGET_OUTPUT_SCALE_Y * TARGET_OUTPUT_POLARITY_Y)
    raw_dx = clamp_abs(raw_dx, SAFE_TX_MAX_ABS_X)
    raw_dy = clamp_abs(raw_dy, SAFE_TX_MAX_ABS_Y)
    return raw_dx, raw_dy

def target_offset_for_circle(obj):
    offset = target_offset(obj)
    if offset is None:
        return None
    raw_dx, raw_dy = offset
    raw_dx = apply_output_deadband(raw_dx, CIRCLE_OUTPUT_DEADBAND_X) * CIRCLE_OUTPUT_SCALE_X * CIRCLE_OUTPUT_POLARITY_X
    raw_dy = apply_output_deadband(raw_dy, CIRCLE_OUTPUT_DEADBAND_Y) * CIRCLE_OUTPUT_SCALE_Y * CIRCLE_OUTPUT_POLARITY_Y
    raw_dx = clamp_abs(raw_dx, SAFE_TX_MAX_ABS_X)
    raw_dy = clamp_abs(raw_dy, SAFE_TX_MAX_ABS_Y)
    return raw_dx, raw_dy

def quantize_circle_offset(raw_dx, raw_dy):
    global circle_tx_residual_x, circle_tx_residual_y
    value_x = raw_dx + circle_tx_residual_x
    value_y = raw_dy + circle_tx_residual_y
    out_x = int(value_x)
    out_y = int(value_y)
    circle_tx_residual_x = clamp(value_x - out_x, -1.0, 1.0)
    circle_tx_residual_y = clamp(value_y - out_y, -1.0, 1.0)
    return out_x, out_y

def accept_tx_jump(raw_dx, raw_dy):
    global safe_tx_jump_count, safe_tx_pending_dx, safe_tx_pending_dy
    jump_dx = raw_dx - safe_tx_dx
    jump_dy = raw_dy - safe_tx_dy
    reverse_jump = (
        is_axis_reversal(safe_tx_dx, raw_dx, SAFE_TX_REVERSE_MIN_PIXELS) or
        is_axis_reversal(safe_tx_dy, raw_dy, SAFE_TX_REVERSE_MIN_PIXELS)
    )
    toward_zero = (
        is_axis_toward_zero(safe_tx_dx, raw_dx) and
        is_axis_toward_zero(safe_tx_dy, raw_dy)
    )
    large_jump = (
        jump_dx * jump_dx + jump_dy * jump_dy >
        SAFE_TX_JUMP_REJECT_PIXELS * SAFE_TX_JUMP_REJECT_PIXELS
    )
    if reverse_jump:
        safe_tx_pending_dx = 0
        safe_tx_pending_dy = 0
        safe_tx_jump_count = 1
        return False
    if large_jump and not toward_zero:
        pending_dx = raw_dx - safe_tx_pending_dx
        pending_dy = raw_dy - safe_tx_pending_dy
        if (safe_tx_jump_count <= 0 or
                pending_dx * pending_dx + pending_dy * pending_dy >
                SAFE_TX_JUMP_CONFIRM_TOLERANCE_PIXELS * SAFE_TX_JUMP_CONFIRM_TOLERANCE_PIXELS):
            safe_tx_pending_dx = raw_dx
            safe_tx_pending_dy = raw_dy
            safe_tx_jump_count = 1
        else:
            safe_tx_jump_count += 1
        return safe_tx_jump_count >= SAFE_TX_JUMP_CONFIRM_FRAMES
    safe_tx_jump_count = 0
    safe_tx_pending_dx = raw_dx
    safe_tx_pending_dy = raw_dy
    return True

def is_axis_toward_zero(current, target):
    if current == 0 or target == 0:
        return True
    if current > 0 and target > 0 and target <= current:
        return True
    if current < 0 and target < 0 and target >= current:
        return True
    return False

def prevent_tx_reverse(raw_dx, raw_dy):
    if safe_tx_dx > 0 and raw_dx < 0:
        raw_dx = 0
    elif safe_tx_dx < 0 and raw_dx > 0:
        raw_dx = 0
    if safe_tx_dy > 0 and raw_dy < 0:
        raw_dy = 0
    elif safe_tx_dy < 0 and raw_dy > 0:
        raw_dy = 0
    return raw_dx, raw_dy

def send_zero_target(flag):
    global safe_tx_dx, safe_tx_dy, last_tx_packet
    safe_tx_dx = 0
    safe_tx_dy = 0
    last_tx_packet = None
    uart_write("0,0,%d\n" % flag)

def circle_tx_set_measurement(raw_dx, raw_dy, now_ms):
    global circle_tx_enabled
    if circle_tx_lock is not None:
        circle_tx_lock.acquire()
    try:
        circle_tx_interp.update(raw_dx, raw_dy, now_ms)
        circle_tx_enabled = True
    finally:
        if circle_tx_lock is not None:
            circle_tx_lock.release()

def circle_tx_disable(send_idle=True):
    global circle_tx_enabled
    if circle_tx_lock is not None:
        circle_tx_lock.acquire()
    try:
        circle_tx_enabled = False
        circle_tx_interp.reset()
    finally:
        if circle_tx_lock is not None:
            circle_tx_lock.release()
    if send_idle:
        send_zero_target(0)

def send_done_signal():
    global done_signal_pending, last_target_tx_ms, last_valid_tx_ms
    now = time.ticks_ms()
    if done_signal_pending <= 0:
        return False
    if now - last_target_tx_ms < TARGET_SEND_PERIOD_MS:
        return True
    last_target_tx_ms = now
    done_signal_pending -= 1
    reset_tx_filter()
    last_valid_tx_ms = -1
    send_zero_target(4)
    return True

def circle_tx_send_from_prediction(now_ms):
    global last_tx_packet, safe_tx_dx, safe_tx_dy, circle_tx_last_send_ms
    global last_valid_tx_ms, last_valid_tx_flag
    if now_ms - circle_tx_last_send_ms < TARGET_SEND_PERIOD_MS:
        return
    circle_tx_last_send_ms = now_ms
    if circle_tx_lock is not None:
        circle_tx_lock.acquire()
    try:
        enabled = circle_tx_enabled
        pred = circle_tx_interp.predict(now_ms)
    finally:
        if circle_tx_lock is not None:
            circle_tx_lock.release()
    if not enabled or pred is None:
        return
    raw_dx, raw_dy = pred[0], pred[1]
    if not accept_tx_jump(raw_dx, raw_dy):
        last_tx_packet = (safe_tx_dx, safe_tx_dy)
        last_valid_tx_ms = now_ms
        last_valid_tx_flag = 6
        uart_write("%d,%d,6\n" % (safe_tx_dx, safe_tx_dy))
        return
    raw_dx, raw_dy = quantize_circle_offset(raw_dx, raw_dy)
    safe_tx_dx = limit_step(safe_tx_dx, raw_dx, CIRCLE_TX_MAX_STEP_X)
    safe_tx_dy = limit_step(safe_tx_dy, raw_dy, CIRCLE_TX_MAX_STEP_Y)
    last_tx_packet = (safe_tx_dx, safe_tx_dy)
    last_valid_tx_ms = now_ms
    last_valid_tx_flag = 6
    uart_write("%d,%d,6\n" % (safe_tx_dx, safe_tx_dy))

def circle_tx_thread():
    while not app.need_exit():
        circle_tx_send_from_prediction(time.ticks_ms())
        try:
            time.sleep_ms(2)
        except Exception:
            time.sleep(0.002)

def ensure_circle_tx_thread():
    global circle_tx_thread_started
    if _thread is None or circle_tx_thread_started:
        return
    circle_tx_thread_started = True
    _thread.start_new_thread(circle_tx_thread, ())

def send_target(obj, search_request = False, circle_mode = False, prediction_mode = False):
    global last_target_tx_ms, last_tx_packet, safe_tx_dx, safe_tx_dy, safe_tx_jump_count
    global safe_tx_pending_dx, safe_tx_pending_dy
    global last_valid_tx_ms, last_valid_tx_flag
    now = time.ticks_ms()
    if now - last_target_tx_ms < TARGET_SEND_PERIOD_MS:
        return
    last_target_tx_ms = now
    offset = target_offset_for_circle(obj) if circle_mode else target_offset_for_mcu(obj)
    if offset is None:
        circle_tx_disable(False)
        if last_tx_packet is not None and last_valid_tx_ms >= 0 and now - last_valid_tx_ms <= MISSING_TARGET_HOLD_MS:
            uart_write("%d,%d,%d\n" % (safe_tx_dx, safe_tx_dy, last_valid_tx_flag))
            return
        reset_tx_filter()
        last_valid_tx_ms = -1
        if search_request:
            send_zero_target(2)
        else:
            send_zero_target(0)
        return

    raw_dx, raw_dy = offset
    raw_dx, raw_dy = prevent_tx_reverse(raw_dx, raw_dy)
    if circle_mode:
        if not accept_tx_jump(raw_dx, raw_dy):
            last_tx_packet = (safe_tx_dx, safe_tx_dy)
            last_valid_tx_ms = now
            last_valid_tx_flag = 3 if not prediction_mode else 6
            uart_write("%d,%d,%d\n" % (safe_tx_dx, safe_tx_dy, last_valid_tx_flag))
            return
        circle_tx_set_measurement(raw_dx, raw_dy, now)
        if _thread is not None:
            ensure_circle_tx_thread()
            return
        raw_dx, raw_dy = quantize_circle_offset(raw_dx, raw_dy)
    else:
        circle_tx_disable(False)
        if not accept_tx_jump(raw_dx, raw_dy):
            last_tx_packet = (safe_tx_dx, safe_tx_dy)
            last_valid_tx_ms = now
            last_valid_tx_flag = 1 if not prediction_mode else 5
            uart_write("%d,%d,%d\n" % (safe_tx_dx, safe_tx_dy, last_valid_tx_flag))
            return

    step_x = CIRCLE_TX_MAX_STEP_X if circle_mode else SAFE_TX_MAX_STEP_X
    step_y = CIRCLE_TX_MAX_STEP_Y if circle_mode else SAFE_TX_MAX_STEP_Y
    safe_tx_dx = limit_step(safe_tx_dx, raw_dx, step_x)
    safe_tx_dy = limit_step(safe_tx_dy, raw_dy, step_y)
    if raw_dx == 0:
        safe_tx_dx = limit_step(safe_tx_dx, 0, step_x)
    if raw_dy == 0:
        safe_tx_dy = limit_step(safe_tx_dy, 0, step_y)
    last_tx_packet = (safe_tx_dx, safe_tx_dy)
    if prediction_mode:
        valid_flag = 6 if circle_mode else 5
    else:
        valid_flag = 3 if circle_mode else 1
    last_valid_tx_ms = now
    last_valid_tx_flag = valid_flag
    uart_write("%d,%d,%d\n" % (safe_tx_dx, safe_tx_dy, valid_flag))

def send_debug_status(target_obj, output_valid, yolo_obj, yolo_reason):
    global last_udp_debug_ms, last_console_debug_ms
    now = time.ticks_ms()
    need_udp = UDP_DEBUG_ENABLE and now - last_udp_debug_ms >= UDP_DEBUG_PERIOD_MS
    need_console = CONSOLE_DEBUG_ENABLE and now - last_console_debug_ms >= CONSOLE_DEBUG_PERIOD_MS
    if not need_udp and not need_console:
        return

    tx_dx = 0
    tx_dy = 0
    tx_valid = 0
    raw_dx = 0
    raw_dy = 0
    dir_hint = "CC"
    if target_obj is not None and output_valid:
        raw_offset = target_offset(target_obj)
        if raw_offset is not None:
            raw_dx, raw_dy = raw_offset
            dir_hint = direction_hint(raw_dx, raw_dy)
        off = target_offset_for_mcu(target_obj)
        if off is not None:
            tx_dx = off[0]
            tx_dy = off[1]
            tx_valid = 1

    tcx = -1
    tcy = -1
    if target_obj is not None:
        tcx, tcy = obj_center(target_obj)

    ycx = -1
    ycy = -1
    yscore = 0.0
    if yolo_obj is not None:
        ycx, ycy = obj_center(yolo_obj)
        yscore = yolo_obj.score

    gate_reason, gate_dist, gate_count = strong_snap_gate.debug_values()
    line = (
        "V,t=%lu,vs=%s,valid=%d,dir=%s,raw=%d,%d,tx=%d,%d,tgt=%d,%d,yolo=%d,%d,ys=%.2f,yr=%s,gate=%s,gd=%d,gc=%d,stm=%d,sdx=%d,sdy=%d,va=%d,vpo=%.3f,vyo=%.3f" %
        (ticks(), vision_state_name(), tx_valid, dir_hint, raw_dx, raw_dy, tx_dx, tx_dy,
         tcx, tcy, ycx, ycy, yscore, yolo_reason,
         gate_reason, gate_dist, gate_count,
         stat["state"], stat["dx"], stat["dy"],
         stat["vision_active"], stat["vision_pitch_offset"], stat["vision_yaw_offset"]))
    if need_udp:
        last_udp_debug_ms = now
        udp_debug.send(line)
    if need_console:
        last_console_debug_ms = now
        print(line)

def send_task_debug(control_obj, output_valid, task_tx_offset):
    global last_task_debug_ms
    if not TASK_DEBUG_ENABLE or task_controller is None:
        return
    now = time.ticks_ms()
    if now - last_task_debug_ms < TASK_DEBUG_PERIOD_MS:
        return
    last_task_debug_ms = now

    runtime = task_controller.runtime
    tx_dx = 0
    tx_dy = 0
    tx_valid = 0
    raw_dx = 0
    raw_dy = 0
    dir_hint = "CC"
    if output_valid and task_tx_offset is not None:
        tx_dx, tx_dy = int(task_tx_offset[0]), int(task_tx_offset[1])
        tx_valid = 1

    cx = -1
    cy = -1
    if control_obj is not None:
        cx, cy = obj_center(control_obj)
        raw_offset = target_offset(control_obj)
        if raw_offset is not None:
            raw_dx, raw_dy = raw_offset
            dir_hint = direction_hint(raw_dx, raw_dy)

    center_valid = 0
    center_x = -1
    center_y = -1
    center_r = -1
    center_w = -1
    center_h = -1
    trace_cx = -1
    trace_cy = -1
    trace_rx = -1
    trace_ry = -1
    center_reason = "none"
    meas_x = -1
    meas_y = -1
    meas_r = -1
    kf_ready = 0
    kf_source = "none"
    kf_x = -1
    kf_y = -1
    kf_r = -1
    kf_vx = 0.0
    kf_vy = 0.0
    kf_vr = 0.0
    kf_ix = 0.0
    kf_iy = 0.0
    kf_ir = 0.0
    kf_miss = 0
    if target_center_result is not None:
        center_valid = 1 if target_center_result.valid else 0
        center_x = target_center_result.cx
        center_y = target_center_result.cy
        center_r = target_center_result.radius
        center_w = target_center_result.width
        center_h = target_center_result.height
        center_reason = target_center_result.reason
        meas_x = target_center_result.meas_cx
        meas_y = target_center_result.meas_cy
        meas_r = target_center_result.meas_r
        if target_center_result.kalman is not None:
            (kf_ready, kf_source, kf_x, kf_y, kf_r,
             kf_vx, kf_vy, kf_vr, kf_ix, kf_iy, kf_ir, kf_miss) = target_center_result.kalman
    if runtime.circle_trace_cx >= 0 and runtime.circle_trace_cy >= 0:
        trace_cx = int(runtime.circle_trace_cx + 0.5)
        trace_cy = int(runtime.circle_trace_cy + 0.5)
        trace_rx = int(runtime.circle_trace_rx + 0.5)
        trace_ry = int(runtime.circle_trace_ry + 0.5)
    y_ready, y_source, y_pred_x, y_pred_y, y_meas_x, y_meas_y, y_out_x, y_out_y, y_vx, y_vy, y_inn_x, y_inn_y, y_miss = kalman_tracker.debug_values()
    c_proc, c_rproc, c_meas, c_rmeas, c_max_v, c_max_rv, c_max_miss, c_decay = circle_kalman_params()

    hold_ms = 0
    if runtime.hold_start_ms >= 0:
        hold_ms = now - runtime.hold_start_ms

    line = (
        "TDBG,t=%lu,fps=%.1f,yfps=%.1f,yms=%d,task=%s,run=%d,sel=%d,state=%s,idx=%d,total=%d,done=%d,timeout=%d,hold=%lu,dir=%s,raw=%d,%d,tx=%d,%d,valid=%d,ctrl=%d,%d,pre=%d,%s,%d,%d,cg=%s,%d,ykf=%d,%s,p=%d,%d,m=%d,%d,o=%d,%d,yv=%.1f,%.1f,yinn=%.1f,%.1f,ymiss=%d,yp=%.1f,%.1f,%.1f,%d,%.2f,center=%d,%d,%d,r=%d,wh=%d,%d,tr=%d,%d,%d,%d,meas=%d,%d,%d,kf=%d,%s,%d,%d,%d,vel=%.1f,%.1f,%.1f,inn=%.1f,%.1f,%.1f,km=%d,cp=%.1f,%.1f,%.2f,%.2f,%.1f,%.1f,%d,%.2f,%s,stm=%s,va=%d,ph=%.3f" %
        (ticks(),
         app_fps,
         yolo_fps,
         last_yolo_infer_ms,
         task_controller.task_name(),
         1 if task_controller.running else 0,
         1 if task_controller.selected else 0,
         runtime.status,
         runtime.index,
         len(runtime.sequence),
         1 if runtime.done else 0,
         1 if runtime.timeout else 0,
         hold_ms,
         dir_hint,
         raw_dx,
         raw_dy,
         tx_dx,
         tx_dy,
         tx_valid,
         cx,
         cy,
         1 if preaim_recent(now) else 0,
         preaim_dir,
         preaim_dx,
         preaim_dy,
         center_refine_gate_reason,
         center_refine_confirm_frames,
         y_ready,
         y_source,
         y_pred_x,
         y_pred_y,
         y_meas_x,
         y_meas_y,
         y_out_x,
         y_out_y,
         y_vx,
         y_vy,
         y_inn_x,
         y_inn_y,
         y_miss,
         kalman_tracker.process_var,
         kalman_tracker.measure_var,
         kalman_tracker.max_speed,
         kalman_tracker.max_predict_frames,
         kalman_tracker.miss_velocity_decay,
         center_valid,
         center_x,
         center_y,
         center_r,
         center_w,
         center_h,
         trace_cx,
         trace_cy,
         trace_rx,
         trace_ry,
         meas_x,
         meas_y,
         meas_r,
         kf_ready,
         kf_source,
         kf_x,
         kf_y,
         kf_r,
         kf_vx,
         kf_vy,
         kf_vr,
         kf_ix,
         kf_iy,
         kf_ir,
         kf_miss,
         c_proc,
         c_rproc,
         c_meas,
         c_rmeas,
         c_max_v,
         c_max_rv,
         c_max_miss,
         c_decay,
         center_reason,
         state_name(),
         stat["vision_active"],
         runtime.circle_phase))
    if TASK_DEBUG_CONSOLE_ENABLE:
        print(line)
    if TASK_DEBUG_UDP_ENABLE and UDP_DEBUG_ENABLE:
        udp_debug.send(line)

def filter_objs(objs):
    filtered = []
    for obj in objs:
        if obj.x >= PANEL_X:
            continue
        if is_laser_hit_obj(obj):
            continue
        if obj.w < MIN_BOX_WIDTH or obj.h < MIN_BOX_HEIGHT:
            continue
        min_score = BANANA_MIN_SCORE_TO_DRAW if is_cv_anchor_class(obj.class_id) else MIN_SCORE_TO_DRAW
        if obj.score < min_score:
            continue
        filtered.append(obj)
    return filtered[:MAX_OBJS]

def init_candidate_slots():
    global candidate_slots
    if len(candidate_slots) == 0:
        for class_id in range(len(detector.labels)):
            candidate_slots.append({"class_id": class_id, "obj": None, "lost": CANDIDATE_MAX_LOST_FRAMES + 1})

def display_slot_indices():
    indices = []
    for i in range(len(candidate_slots)):
        indices.append(i)
    indices.sort(key = lambda i: display_label(candidate_slots[i]["class_id"]))
    return indices

def display_label(class_id):
    label = detector.labels[class_id]
    return DISPLAY_LABEL_BY_LABEL.get(label, label)

def class_label(class_id):
    if class_id < 0 or class_id >= len(detector.labels):
        return ""
    return detector.labels[class_id]

def is_banana_class(class_id):
    return class_label(class_id) == "banana"

def is_watermelon_class(class_id):
    label = class_label(class_id)
    return label == "watermelon" or label == "watermalon"

def is_cv_anchor_class(class_id):
    return is_banana_class(class_id) or is_watermelon_class(class_id)

def is_selected_banana():
    return is_banana_class(selected_class_id)

def is_selected_cv_anchor():
    return is_cv_anchor_class(selected_class_id)

def update_candidate_slots(detections):
    init_candidate_slots()
    best_by_class = {}
    for obj in detections:
        old = best_by_class.get(obj.class_id)
        if old is None or obj.score > old.score:
            best_by_class[obj.class_id] = obj

    for slot in candidate_slots:
        obj = best_by_class.get(slot["class_id"])
        if obj is not None:
            slot["obj"] = obj
            slot["lost"] = 0
        else:
            slot["lost"] += 1

    visible = []
    for slot in candidate_slots:
        if slot["obj"] is not None and slot["lost"] == 0:
            visible.append(slot["obj"])
    return visible

def filter_track_objs(objs):
    filtered = []
    for obj in objs:
        if obj.x >= PANEL_X:
            continue
        if is_laser_hit_obj(obj):
            continue
        if obj.w < MIN_BOX_WIDTH or obj.h < MIN_BOX_HEIGHT:
            continue
        min_score = BANANA_TRACK_MIN_SCORE if is_cv_anchor_class(obj.class_id) else TRACK_MIN_SCORE
        if obj.score < min_score:
            continue
        filtered.append(obj)
    return filtered

def obj_center(obj):
    return obj.x + obj.w // 2, obj.y + obj.h // 2

def is_laser_hit_obj(obj):
    if not LASER_MASK_ENABLE or not aim_calibrated:
        return False
    if obj.w > LASER_MASK_MAX_BOX_SIZE or obj.h > LASER_MASK_MAX_BOX_SIZE:
        return False

    cx, cy = obj_center(obj)
    ax, ay = aim_point()
    dx = cx - ax
    dy = cy - ay
    return dx * dx + dy * dy <= LASER_MASK_RADIUS * LASER_MASK_RADIUS

def pixel_rgb(pixel):
    def unpack(value):
        value = int(value)
        if value <= 0xff:
            return value, value, value
        return (value >> 16) & 0xff, (value >> 8) & 0xff, value & 0xff

    if isinstance(pixel, tuple) or isinstance(pixel, list):
        if len(pixel) >= 3:
            r = clamp(int(pixel[0]), 0, 255)
            g = clamp(int(pixel[1]), 0, 255)
            b = clamp(int(pixel[2]), 0, 255)
            return r, g, b
        if len(pixel) == 1:
            return unpack(pixel[0])
        return None
    return unpack(pixel)

def laser_patch_color(img, ax, ay, r):
    sample_r = r + LASER_INPUT_SAMPLE_PAD
    points = (
        (ax - sample_r, ay),
        (ax + sample_r, ay),
        (ax, ay - sample_r),
        (ax, ay + sample_r),
        (ax - sample_r, ay - sample_r),
        (ax + sample_r, ay - sample_r),
        (ax - sample_r, ay + sample_r),
        (ax + sample_r, ay + sample_r),
    )
    count = 0
    sum_r = 0
    sum_g = 0
    sum_b = 0
    for px, py in points:
        if px < 0 or px >= PANEL_X or py < 0 or py >= CAMERA_HEIGHT:
            continue
        try:
            rgb = pixel_rgb(img.get_pixel(px, py))
        except Exception:
            rgb = None
        if rgb is None:
            continue
        sum_r += rgb[0]
        sum_g += rgb[1]
        sum_b += rgb[2]
        count += 1
    if count <= 0:
        return COLOR_WHITE
    return image.Color.from_rgb(sum_r // count, sum_g // count, sum_b // count)

def mask_laser_for_detector(img, radius = LASER_INPUT_MASK_RADIUS, copy_image = True):
    if not LASER_INPUT_MASK_ENABLE or not aim_calibrated:
        return img

    ax, ay = aim_point()
    r = radius
    x1 = clamp(ax - r, 0, PANEL_X - 1)
    y1 = clamp(ay - r, 0, CAMERA_HEIGHT - 1)
    x2 = clamp(ax + r + 1, 0, PANEL_X)
    y2 = clamp(ay + r + 1, 0, CAMERA_HEIGHT)
    if x2 <= x1 or y2 <= y1:
        return img

    patch_color = laser_patch_color(img, ax, ay, r)
    if copy_image:
        try:
            masked = img.copy()
        except Exception:
            masked = img
    else:
        masked = img
    masked.draw_rect(x1, y1, x2 - x1, y2 - y1, color = patch_color, thickness = -1)
    return masked

def clamp(value, min_value, max_value):
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value

def apply_digital_zoom(img):
    global zoom_warned
    scale = DIGITAL_ZOOM_SCALE
    if scale <= 1.01:
        return img

    src_w = img.width()
    src_h = img.height()
    crop_w = clamp(int(src_w / scale), 32, src_w)
    crop_h = clamp(int(src_h / scale), 32, src_h)
    center_x = int(src_w * clamp(DIGITAL_ZOOM_CENTER_X, 0.0, 1.0))
    center_y = int(src_h * clamp(DIGITAL_ZOOM_CENTER_Y, 0.0, 1.0))
    crop_x = clamp(center_x - crop_w // 2, 0, src_w - crop_w)
    crop_y = clamp(center_y - crop_h // 2, 0, src_h - crop_h)

    try:
        return img.crop(crop_x, crop_y, crop_w, crop_h).resize(src_w, src_h)
    except Exception as e:
        if not zoom_warned:
            log("digital zoom disabled: %s" % e)
            zoom_warned = True
        return img

def expanded_roi(obj):
    x = clamp(obj.x - ROI_MARGIN, 0, PANEL_X - 1)
    y = clamp(obj.y - ROI_MARGIN, 0, CAMERA_HEIGHT - 1)
    x2 = clamp(obj.x + obj.w + ROI_MARGIN, 0, PANEL_X)
    y2 = clamp(obj.y + obj.h + ROI_MARGIN, 0, CAMERA_HEIGHT)
    return [x, y, x2 - x, y2 - y]

def overlap_area(a_x, a_y, a_w, a_h, b_x, b_y, b_w, b_h):
    left = max(a_x, b_x)
    top = max(a_y, b_y)
    right = min(a_x + a_w, b_x + b_w)
    bottom = min(a_y + a_h, b_y + b_h)
    if right <= left or bottom <= top:
        return 0
    return (right - left) * (bottom - top)

def refine_obj_by_color(img, obj):
    if not ENABLE_COLOR_REFINE:
        return obj

    label = detector.labels[obj.class_id]
    if label not in COLOR_THRESHOLDS_BY_LABEL:
        return obj

    roi = expanded_roi(obj)
    blobs = img.find_blobs(COLOR_THRESHOLDS_BY_LABEL[label], roi = roi, pixels_threshold = COLOR_PIXELS_THRESHOLD, area_threshold = COLOR_AREA_THRESHOLD, merge = True, margin = COLOR_MERGE_MARGIN)
    if len(blobs) == 0:
        return obj

    obj_cx, obj_cy = obj_center(obj)
    max_center_dist = max(obj.w, obj.h) * COLOR_MAX_CENTER_DIST_SCALE
    merged_x1 = PANEL_X
    merged_y1 = CAMERA_HEIGHT
    merged_x2 = 0
    merged_y2 = 0
    merged_pixels = 0
    for blob in blobs:
        blob_x = blob[0]
        blob_y = blob[1]
        blob_w = blob[2]
        blob_h = blob[3]
        blob_pixels = blob[4]
        if blob_w < MIN_COLOR_BOX_W or blob_h < MIN_COLOR_BOX_H:
            continue
        overlap = overlap_area(blob_x, blob_y, blob_w, blob_h, obj.x, obj.y, obj.w, obj.h)
        blob_area = blob_w * blob_h
        if blob_area <= 0 or overlap < blob_area * COLOR_MIN_BLOB_INSIDE:
            continue
        cx = blob_x + blob_w // 2
        cy = blob_y + blob_h // 2
        dx = cx - obj_cx
        dy = cy - obj_cy
        if dx * dx + dy * dy > max_center_dist * max_center_dist:
            continue
        merged_x1 = min(merged_x1, blob_x)
        merged_y1 = min(merged_y1, blob_y)
        merged_x2 = max(merged_x2, blob_x + blob_w)
        merged_y2 = max(merged_y2, blob_y + blob_h)
        merged_pixels += blob_pixels

    if merged_pixels <= 0:
        return obj

    merged_x1 = clamp(merged_x1 - COLOR_REFINE_PAD, 0, PANEL_X - 1)
    merged_y1 = clamp(merged_y1 - COLOR_REFINE_PAD, 0, CAMERA_HEIGHT - 1)
    merged_x2 = clamp(merged_x2 + COLOR_REFINE_PAD, 0, PANEL_X)
    merged_y2 = clamp(merged_y2 + COLOR_REFINE_PAD, 0, CAMERA_HEIGHT)
    return TrackedObj(merged_x1, merged_y1, merged_x2 - merged_x1, merged_y2 - merged_y1, obj.class_id, obj.score)

def recover_selected_by_color(img):
    if selected_class_id < 0 or selected_cx < 0 or selected_cy < 0:
        return None

    label = detector.labels[selected_class_id]
    if label not in COLOR_THRESHOLDS_BY_LABEL:
        return None

    base_w = max(selected_w, MIN_BOX_WIDTH)
    base_h = max(selected_h, MIN_BOX_HEIGHT)
    roi_w = base_w + RECOVER_ROI_GROW * 2
    roi_h = base_h + RECOVER_ROI_GROW * 2
    x = clamp(selected_cx - roi_w // 2, 0, PANEL_X - 1)
    y = clamp(selected_cy - roi_h // 2, 0, CAMERA_HEIGHT - 1)
    x2 = clamp(selected_cx + roi_w // 2, 0, PANEL_X)
    y2 = clamp(selected_cy + roi_h // 2, 0, CAMERA_HEIGHT)
    roi = [x, y, x2 - x, y2 - y]

    blobs = img.find_blobs(COLOR_THRESHOLDS_BY_LABEL[label], roi = roi, pixels_threshold = COLOR_PIXELS_THRESHOLD, area_threshold = COLOR_AREA_THRESHOLD, merge = True, margin = COLOR_MERGE_MARGIN)
    best_blob = None
    best_cost = 0
    for blob in blobs:
        blob_x = blob[0]
        blob_y = blob[1]
        blob_w = blob[2]
        blob_h = blob[3]
        blob_pixels = blob[4]
        if blob_w < MIN_COLOR_BOX_W or blob_h < MIN_COLOR_BOX_H:
            continue
        cx = blob_x + blob_w // 2
        cy = blob_y + blob_h // 2
        dx = cx - selected_cx
        dy = cy - selected_cy
        dist2 = dx * dx + dy * dy
        if dist2 > RECOVER_MAX_CENTER_DIST * RECOVER_MAX_CENTER_DIST:
            continue
        size_cost = abs(blob_w - base_w) + abs(blob_h - base_h)
        cost = dist2 + size_cost * size_cost - blob_pixels
        if best_blob is None or cost < best_cost:
            best_blob = blob
            best_cost = cost

    if best_blob is None:
        return None

    x = clamp(best_blob[0] - COLOR_REFINE_PAD, 0, PANEL_X - 1)
    y = clamp(best_blob[1] - COLOR_REFINE_PAD, 0, CAMERA_HEIGHT - 1)
    x2 = clamp(best_blob[0] + best_blob[2] + COLOR_REFINE_PAD, 0, PANEL_X)
    y2 = clamp(best_blob[1] + best_blob[3] + COLOR_REFINE_PAD, 0, CAMERA_HEIGHT)
    return TrackedObj(x, y, x2 - x, y2 - y, selected_class_id, 0.50)

def use_color_recover():
    return False

def base_track_box():
    if last_good_cx >= 0 and last_good_cy >= 0:
        return last_good_cx, last_good_cy, max(last_good_w, MIN_BOX_WIDTH), max(last_good_h, MIN_BOX_HEIGHT)
    if selected_cx >= 0 and selected_cy >= 0:
        return selected_cx, selected_cy, max(selected_w, MIN_BOX_WIDTH), max(selected_h, MIN_BOX_HEIGHT)
    return -1, -1, -1, -1

def predicted_track_center():
    base_cx, base_cy, base_w, base_h = base_track_box()
    if base_cx < 0 or base_cy < 0:
        return -1, -1
    shift_x, shift_y = camera_motion_shift_from_ref()
    pred_cx = clamp(base_cx + track_vx + shift_x, 0, PANEL_X - 1)
    pred_cy = clamp(base_cy + track_vy + shift_y, 0, CAMERA_HEIGHT - 1)
    return pred_cx, pred_cy

def build_track_roi():
    pred_cx, pred_cy = predicted_track_center()
    base_cx, base_cy, base_w, base_h = base_track_box()
    if pred_cx < 0 or pred_cy < 0 or base_w <= 0 or base_h <= 0:
        return None

    roi_w = int(base_w * ROI_TRACK_SCALE) + ROI_TRACK_MIN_PAD * 2
    roi_h = int(base_h * ROI_TRACK_SCALE) + ROI_TRACK_MIN_PAD * 2
    roi_w = clamp(roi_w, base_w + ROI_TRACK_MIN_PAD * 2, PANEL_X)
    roi_h = clamp(roi_h, base_h + ROI_TRACK_MIN_PAD * 2, CAMERA_HEIGHT)
    x = clamp(pred_cx - roi_w // 2, 0, PANEL_X - 1)
    y = clamp(pred_cy - roi_h // 2, 0, CAMERA_HEIGHT - 1)
    x2 = clamp(x + roi_w, 0, PANEL_X)
    y2 = clamp(y + roi_h, 0, CAMERA_HEIGHT)
    return [x, y, x2 - x, y2 - y]

def yolo_gate_rect():
    if last_yolo_cx < 0 or last_yolo_cy < 0 or last_yolo_w <= 0 or last_yolo_h <= 0:
        return None
    gate_w = int(max(last_yolo_w * YOLO_GATE_SCALE, last_yolo_w + YOLO_GATE_MIN_DIST))
    gate_h = int(max(last_yolo_h * YOLO_GATE_SCALE, last_yolo_h + YOLO_GATE_MIN_DIST))
    gate_w = clamp(gate_w, last_yolo_w, PANEL_X)
    gate_h = clamp(gate_h, last_yolo_h, CAMERA_HEIGHT)
    x = clamp(last_yolo_cx - gate_w // 2, 0, PANEL_X - 1)
    y = clamp(last_yolo_cy - gate_h // 2, 0, CAMERA_HEIGHT - 1)
    x2 = clamp(x + gate_w, 0, PANEL_X)
    y2 = clamp(y + gate_h, 0, CAMERA_HEIGHT)
    return [x, y, x2 - x, y2 - y]

def point_in_rect(px, py, rect):
    if rect is None:
        return True
    return px >= rect[0] and px < rect[0] + rect[2] and py >= rect[1] and py < rect[1] + rect[3]

def expanded_obj_rect(obj, scale, min_pad):
    if obj is None:
        return None
    cx, cy = obj_center(obj)
    base_w = max(obj.w, MIN_BOX_WIDTH)
    base_h = max(obj.h, MIN_BOX_HEIGHT)
    gate_w = int(max(base_w * scale, base_w + min_pad * 2))
    gate_h = int(max(base_h * scale, base_h + min_pad * 2))
    gate_w = clamp(gate_w, base_w, PANEL_X)
    gate_h = clamp(gate_h, base_h, CAMERA_HEIGHT)
    x = clamp(cx - gate_w // 2, 0, PANEL_X - 1)
    y = clamp(cy - gate_h // 2, 0, CAMERA_HEIGHT - 1)
    x2 = clamp(x + gate_w, 0, PANEL_X)
    y2 = clamp(y + gate_h, 0, CAMERA_HEIGHT)
    return [x, y, x2 - x, y2 - y]

def rect_overlap_ratio(a, b):
    if a is None or b is None or a[2] <= 0 or a[3] <= 0:
        return 0.0
    overlap = overlap_area(a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3])
    return float(overlap) / float(max(a[2] * a[3], 1))

def roi_blob_to_obj(blob, class_id, score):
    cx = blob[0] + blob[2] // 2
    cy = blob[1] + blob[3] // 2
    return box_at_center(cx, cy, selected_w, selected_h, class_id, score)

def roi_candidate_cost(blob, pred_cx, pred_cy, base_w, base_h):
    blob_w = blob[2]
    blob_h = blob[3]
    blob_pixels = blob[4]
    cx = blob[0] + blob_w // 2
    cy = blob[1] + blob_h // 2
    dx = cx - pred_cx
    dy = cy - pred_cy
    size_cost = abs(blob_w - base_w) + abs(blob_h - base_h)
    return dx * dx + dy * dy + size_cost * size_cost * SIZE_COST_WEIGHT - blob_pixels

def is_roi_blob_valid(blob, roi, pred_cx, pred_cy, base_w, base_h, gate_rect):
    blob_x = blob[0]
    blob_y = blob[1]
    blob_w = blob[2]
    blob_h = blob[3]
    if blob_w < MIN_COLOR_BOX_W or blob_h < MIN_COLOR_BOX_H:
        return False
    if ROI_TRACK_EDGE_REJECT_MARGIN > 0:
        if blob_x <= roi[0] + ROI_TRACK_EDGE_REJECT_MARGIN:
            return False
        if blob_y <= roi[1] + ROI_TRACK_EDGE_REJECT_MARGIN:
            return False
        if blob_x + blob_w >= roi[0] + roi[2] - ROI_TRACK_EDGE_REJECT_MARGIN:
            return False
        if blob_y + blob_h >= roi[1] + roi[3] - ROI_TRACK_EDGE_REJECT_MARGIN:
            return False

    cx = blob_x + blob_w // 2
    cy = blob_y + blob_h // 2
    if not point_in_rect(cx, cy, gate_rect):
        return False

    max_center_dist = max(ROI_TRACK_MIN_CENTER_DIST, int(max(base_w, base_h) * ROI_TRACK_MAX_CENTER_DIST_SCALE))
    dx = cx - pred_cx
    dy = cy - pred_cy
    if dx * dx + dy * dy > max_center_dist * max_center_dist:
        return False

    if abs(blob_w - base_w) > base_w * ROI_TRACK_MAX_SIZE_DELTA_RATIO:
        return False
    if abs(blob_h - base_h) > base_h * ROI_TRACK_MAX_SIZE_DELTA_RATIO:
        return False

    base_area = max(base_w * base_h, 1)
    blob_area = max(blob_w * blob_h, 1)
    if blob_area < base_area * ROI_TRACK_MIN_AREA_RATIO:
        return False
    if blob_area > base_area * ROI_TRACK_MAX_AREA_RATIO:
        return False

    base_aspect = base_w / max(base_h, 1)
    blob_aspect = blob_w / max(blob_h, 1)
    if abs(blob_aspect - base_aspect) > base_aspect * ROI_TRACK_MAX_ASPECT_DELTA_RATIO:
        return False

    return True

def choose_unique_roi_candidate(blobs, roi, pred_cx, pred_cy, base_w, base_h, gate_rect):
    candidates = []
    for blob in blobs:
        if is_roi_blob_valid(blob, roi, pred_cx, pred_cy, base_w, base_h, gate_rect):
            cost = roi_candidate_cost(blob, pred_cx, pred_cy, base_w, base_h)
            candidates.append((cost, blob))

    if len(candidates) == 0:
        return None, "missing"

    candidates.sort(key = lambda item: item[0])
    best_cost, best_blob = candidates[0]
    if len(candidates) > 1:
        second_cost = candidates[1][0]
        close_margin = abs(best_cost) * (ROI_TRACK_AMBIGUOUS_COST_RATIO - 1.0) + ROI_TRACK_AMBIGUOUS_COST_MARGIN
        if second_cost - best_cost < close_margin:
            return None, "ambiguous"

    return best_blob, "ok"

def track_selected_by_maix_blobs(img):
    if not ROI_TRACK_ENABLE or selected_class_id < 0:
        return None, "disabled"

    label = detector.labels[selected_class_id]
    thresholds = dynamic_color_model.thresholds
    if thresholds is None:
        thresholds = COLOR_THRESHOLDS_BY_LABEL.get(label)
    if thresholds is None:
        return None, "no color"

    roi = build_track_roi()
    if roi is None or roi[2] <= 0 or roi[3] <= 0:
        return None, "missing roi"

    pred_cx, pred_cy = predicted_track_center()
    base_cx, base_cy, base_w, base_h = base_track_box()
    gate_rect = yolo_gate_rect()
    blobs = img.find_blobs(thresholds, roi = roi, pixels_threshold = COLOR_PIXELS_THRESHOLD, area_threshold = COLOR_AREA_THRESHOLD, merge = True, margin = COLOR_MERGE_MARGIN)
    blob, reason = choose_unique_roi_candidate(blobs, roi, pred_cx, pred_cy, base_w, base_h, gate_rect)
    if blob is None:
        return None, reason

    return roi_blob_to_obj(blob, selected_class_id, 0.58), "ok"

def banana_cv_base_box():
    if smooth_x >= 0 and smooth_y >= 0:
        return (
            smooth_x + smooth_w // 2,
            smooth_y + smooth_h // 2,
            max(smooth_w, MIN_BOX_WIDTH),
            max(smooth_h, MIN_BOX_HEIGHT),
        )
    if last_good_cx >= 0 and last_good_cy >= 0:
        return (
            last_good_cx,
            last_good_cy,
            max(last_good_w, MIN_BOX_WIDTH),
            max(last_good_h, MIN_BOX_HEIGHT),
        )
    if last_yolo_cx >= 0 and last_yolo_cy >= 0:
        return (
            last_yolo_cx,
            last_yolo_cy,
            max(last_yolo_w, MIN_BOX_WIDTH),
            max(last_yolo_h, MIN_BOX_HEIGHT),
        )
    if selected_cx >= 0 and selected_cy >= 0:
        return (
            selected_cx,
            selected_cy,
            max(selected_w, MIN_BOX_WIDTH),
            max(selected_h, MIN_BOX_HEIGHT),
        )
    return -1, -1, -1, -1

def banana_cv_roi_box(yolo_obj = None):
    if yolo_obj is not None:
        yolo_cx, yolo_cy = obj_center(yolo_obj)
        return (
            yolo_cx,
            yolo_cy,
            max(yolo_obj.w, MIN_BOX_WIDTH),
            max(yolo_obj.h, MIN_BOX_HEIGHT),
        )
    if (last_yolo_seen_frames <= BANANA_CV_YOLO_GATE_HOLD_FRAMES and
            last_yolo_cx >= 0 and last_yolo_cy >= 0 and
            last_yolo_w > 0 and last_yolo_h > 0):
        return (
            last_yolo_cx,
            last_yolo_cy,
            max(last_yolo_w, MIN_BOX_WIDTH),
            max(last_yolo_h, MIN_BOX_HEIGHT),
        )
    if (last_cv_seen_frames <= BANANA_CV_HOLD_FRAMES and
            last_cv_cx >= 0 and last_cv_cy >= 0 and
            last_cv_w > 0 and last_cv_h > 0):
        return (
            last_cv_cx,
            last_cv_cy,
            max(last_cv_w, MIN_BOX_WIDTH),
            max(last_cv_h, MIN_BOX_HEIGHT),
        )
    return banana_cv_base_box()

def banana_cv_track_roi(yolo_obj = None):
    roi_cx, roi_cy, base_w, base_h = banana_cv_roi_box(yolo_obj)
    if roi_cx < 0 or roi_cy < 0 or base_w <= 0 or base_h <= 0:
        return None
    roi_w = int(base_w * BANANA_CV_ROI_SCALE) + BANANA_CV_ROI_MIN_PAD * 2
    roi_h = int(base_h * BANANA_CV_ROI_SCALE) + BANANA_CV_ROI_MIN_PAD * 2
    roi_w = clamp(roi_w, base_w + BANANA_CV_ROI_MIN_PAD * 2, PANEL_X)
    roi_h = clamp(roi_h, base_h + BANANA_CV_ROI_MIN_PAD * 2, CAMERA_HEIGHT)
    x = clamp(roi_cx - roi_w // 2, 0, PANEL_X - 1)
    y = clamp(roi_cy - roi_h // 2, 0, CAMERA_HEIGHT - 1)
    x2 = clamp(x + roi_w, 0, PANEL_X)
    y2 = clamp(y + roi_h, 0, CAMERA_HEIGHT)
    return [x, y, x2 - x, y2 - y]

def banana_cv_yolo_gate_rect(yolo_obj = None):
    if yolo_obj is not None:
        return expanded_obj_rect(
            yolo_obj,
            BANANA_CV_YOLO_GATE_SCALE,
            BANANA_CV_YOLO_GATE_MIN_PAD,
        )
    if last_yolo_seen_frames > BANANA_CV_YOLO_GATE_HOLD_FRAMES:
        return None
    if last_yolo_cx < 0 or last_yolo_cy < 0 or last_yolo_w <= 0 or last_yolo_h <= 0:
        return None
    gate_w = int(max(
        last_yolo_w * BANANA_CV_YOLO_GATE_SCALE,
        last_yolo_w + BANANA_CV_YOLO_GATE_MIN_PAD * 2,
    ))
    gate_h = int(max(
        last_yolo_h * BANANA_CV_YOLO_GATE_SCALE,
        last_yolo_h + BANANA_CV_YOLO_GATE_MIN_PAD * 2,
    ))
    gate_w = clamp(gate_w, last_yolo_w, PANEL_X)
    gate_h = clamp(gate_h, last_yolo_h, CAMERA_HEIGHT)
    x = clamp(last_yolo_cx - gate_w // 2, 0, PANEL_X - 1)
    y = clamp(last_yolo_cy - gate_h // 2, 0, CAMERA_HEIGHT - 1)
    x2 = clamp(x + gate_w, 0, PANEL_X)
    y2 = clamp(y + gate_h, 0, CAMERA_HEIGHT)
    return [x, y, x2 - x, y2 - y]

def banana_cv_has_hold_ref():
    return (
        last_cv_seen_frames <= BANANA_CV_HOLD_FRAMES and
        last_cv_cx >= 0 and last_cv_cy >= 0 and
        last_cv_w > 0 and last_cv_h > 0
    )

def banana_cv_candidate_cost(blob, ref_cx, ref_cy, base_w, base_h):
    blob_w = blob[2]
    blob_h = blob[3]
    blob_pixels = blob[4]
    cx = blob[0] + blob_w // 2
    cy = blob[1] + blob_h // 2
    dx = cx - ref_cx
    dy = cy - ref_cy
    size_cost = abs(blob_w - base_w) + abs(blob_h - base_h)
    return dx * dx + dy * dy + size_cost * size_cost * 0.55 - blob_pixels

def banana_cv_yolo_blob_ok(blob, yolo_obj, gate_rect):
    if yolo_obj is None:
        return True
    blob_cx = blob[0] + blob[2] // 2
    blob_cy = blob[1] + blob[3] // 2
    if not point_in_rect(blob_cx, blob_cy, gate_rect):
        return False
    yolo_cx, yolo_cy = obj_center(yolo_obj)
    dx = blob_cx - yolo_cx
    dy = blob_cy - yolo_cy
    max_dist = max(
        BANANA_CV_YOLO_MIN_CENTER_DIST,
        int(max(yolo_obj.w, yolo_obj.h) * BANANA_CV_YOLO_CENTER_RATIO),
    )
    if dx * dx + dy * dy > max_dist * max_dist:
        return False
    blob_rect = [blob[0], blob[1], blob[2], blob[3]]
    yolo_rect = [yolo_obj.x, yolo_obj.y, yolo_obj.w, yolo_obj.h]
    return rect_overlap_ratio(blob_rect, yolo_rect) >= BANANA_CV_YOLO_MIN_OVERLAP_RATIO

def is_banana_cv_blob_valid(blob, roi, ref_cx, ref_cy, base_w, base_h, gate_rect, yolo_obj = None):
    blob_x = blob[0]
    blob_y = blob[1]
    blob_w = blob[2]
    blob_h = blob[3]
    if blob_w < MIN_COLOR_BOX_W or blob_h < MIN_COLOR_BOX_H:
        return False
    cx = blob_x + blob_w // 2
    cy = blob_y + blob_h // 2
    if not point_in_rect(cx, cy, gate_rect):
        return False
    if not banana_cv_yolo_blob_ok(blob, yolo_obj, gate_rect):
        return False
    max_step = max(
        BANANA_CV_MAX_CENTER_STEP,
        int(max(base_w, base_h) * BANANA_CV_MAX_CENTER_STEP_SCALE),
    )
    dx = cx - ref_cx
    dy = cy - ref_cy
    if dx * dx + dy * dy > (max_step * 3) * (max_step * 3):
        return False
    if abs(blob_w - base_w) > base_w * BANANA_CV_MAX_SIZE_DELTA_RATIO:
        return False
    if abs(blob_h - base_h) > base_h * BANANA_CV_MAX_SIZE_DELTA_RATIO:
        return False
    base_area = max(base_w * base_h, 1)
    blob_area = max(blob_w * blob_h, 1)
    if blob_area < base_area * BANANA_CV_MIN_AREA_RATIO:
        return False
    if blob_area > base_area * BANANA_CV_MAX_AREA_RATIO:
        return False
    base_aspect = float(base_w) / float(max(base_h, 1))
    blob_aspect = float(blob_w) / float(max(blob_h, 1))
    if abs(blob_aspect - base_aspect) > base_aspect * BANANA_CV_MAX_ASPECT_DELTA_RATIO:
        return False
    return True

def choose_banana_cv_candidate(blobs, roi, ref_cx, ref_cy, base_w, base_h, gate_rect, yolo_obj = None):
    candidates = []
    for blob in blobs:
        if is_banana_cv_blob_valid(blob, roi, ref_cx, ref_cy, base_w, base_h, gate_rect, yolo_obj):
            cost = banana_cv_candidate_cost(blob, ref_cx, ref_cy, base_w, base_h)
            candidates.append((cost, blob))
    if len(candidates) == 0:
        return None, "missing"
    candidates.sort(key = lambda item: item[0])
    best_cost, best_blob = candidates[0]
    if len(candidates) > 1:
        second_cost = candidates[1][0]
        close_margin = abs(best_cost) * (BANANA_CV_AMBIGUOUS_COST_RATIO - 1.0) + BANANA_CV_AMBIGUOUS_COST_MARGIN
        if second_cost - best_cost < close_margin:
            return None, "ambiguous"
    return best_blob, "ok"

def banana_cv_threshold_sets():
    sets = []
    if dynamic_color_model.thresholds is not None:
        sets.append(("learn", dynamic_color_model.thresholds))
    label = class_label(selected_class_id)
    fallback = COLOR_THRESHOLDS_BY_LABEL.get(label)
    if fallback is not None:
        sets.append((display_label(selected_class_id), fallback))
    elif is_selected_banana():
        sets.append(("yellow", YELLOW_THRESHOLDS))
    return sets

def banana_cv_blob_to_limited_obj(blob, ref_cx, ref_cy, base_w, base_h):
    cand_x = clamp(blob[0] - COLOR_REFINE_PAD, 0, PANEL_X - 1)
    cand_y = clamp(blob[1] - COLOR_REFINE_PAD, 0, CAMERA_HEIGHT - 1)
    cand_x2 = clamp(blob[0] + blob[2] + COLOR_REFINE_PAD, 0, PANEL_X)
    cand_y2 = clamp(blob[1] + blob[3] + COLOR_REFINE_PAD, 0, CAMERA_HEIGHT)
    cand_w = max(cand_x2 - cand_x, MIN_BOX_WIDTH)
    cand_h = max(cand_y2 - cand_y, MIN_BOX_HEIGHT)
    cand_cx = cand_x + cand_w // 2
    cand_cy = cand_y + cand_h // 2
    max_step = max(
        BANANA_CV_MAX_CENTER_STEP,
        int(max(base_w, base_h) * BANANA_CV_MAX_CENTER_STEP_SCALE),
    )
    out_cx = limit_step(ref_cx, cand_cx, max_step)
    out_cy = limit_step(ref_cy, cand_cy, max_step)
    min_w = max(MIN_BOX_WIDTH, int(base_w * (1.0 - BANANA_CV_MAX_SIZE_DELTA_RATIO)))
    max_w = max(min_w, int(base_w * (1.0 + BANANA_CV_MAX_SIZE_DELTA_RATIO)))
    min_h = max(MIN_BOX_HEIGHT, int(base_h * (1.0 - BANANA_CV_MAX_SIZE_DELTA_RATIO)))
    max_h = max(min_h, int(base_h * (1.0 + BANANA_CV_MAX_SIZE_DELTA_RATIO)))
    out_w = clamp(cand_w, min_w, max_w)
    out_h = clamp(cand_h, min_h, max_h)
    return box_at_center(out_cx, out_cy, out_w, out_h, selected_class_id, BANANA_CV_TRACK_SCORE)

def banana_yolo_box_for_output(yolo_obj, score = None):
    if yolo_obj is None:
        return None
    yolo_cx, yolo_cy = obj_center(yolo_obj)
    out_w = selected_w if selected_w > 0 else yolo_obj.w
    out_h = selected_h if selected_h > 0 else yolo_obj.h
    out_score = yolo_obj.score if score is None else score
    return box_at_center(yolo_cx, yolo_cy, out_w, out_h, yolo_obj.class_id, out_score)

def track_selected_banana_by_cv(img, yolo_obj = None):
    if not is_selected_cv_anchor():
        return None, "not cv anchor"
    if (yolo_obj is None and
            last_yolo_seen_frames > BANANA_CV_YOLO_GATE_HOLD_FRAMES and
            not banana_cv_has_hold_ref()):
        return None, "yolo stale"
    roi = banana_cv_track_roi(yolo_obj)
    if roi is None or roi[2] <= 0 or roi[3] <= 0:
        return None, "missing roi"
    if yolo_obj is not None:
        ref_cx, ref_cy = obj_center(yolo_obj)
        base_w = max(yolo_obj.w, MIN_BOX_WIDTH)
        base_h = max(yolo_obj.h, MIN_BOX_HEIGHT)
    else:
        ref_cx, ref_cy, base_w, base_h = banana_cv_base_box()
    gate_rect = banana_cv_yolo_gate_rect(yolo_obj)
    last_reason = "missing"
    for name, thresholds in banana_cv_threshold_sets():
        try:
            blobs = img.find_blobs(
                thresholds,
                roi = roi,
                pixels_threshold = BANANA_CV_PIXELS_THRESHOLD,
                area_threshold = BANANA_CV_AREA_THRESHOLD,
                merge = True,
                margin = BANANA_CV_MERGE_MARGIN,
            )
        except Exception:
            last_reason = name + " blob err"
            continue
        blob, reason = choose_banana_cv_candidate(blobs, roi, ref_cx, ref_cy, base_w, base_h, gate_rect, yolo_obj)
        if blob is not None:
            cv_obj = banana_cv_blob_to_limited_obj(blob, ref_cx, ref_cy, base_w, base_h)
            return cv_obj, "cv " + name
        last_reason = name + " " + reason
    return None, last_reason

def is_in_rect(x, y, rect):
    return x >= rect[0] and x < rect[0] + rect[2] and y >= rect[1] and y < rect[1] + rect[3]

def page_task_rect():
    return [PANEL_X + PANEL_PAD, 10, (PANEL_W - PANEL_PAD * 3) // 2, 36]

def page_calib_rect():
    x = PANEL_X + PANEL_PAD * 2 + (PANEL_W - PANEL_PAD * 3) // 2
    return [x, 10, (PANEL_W - PANEL_PAD * 3) // 2, 36]

def task_item_rect(index):
    return [PANEL_X + PANEL_PAD, 88 + index * 58, PANEL_W - PANEL_PAD * 2, 50]

def candidate_item_rect(index):
    top = 176 if ui_page == UI_PAGE_CALIB else ITEM_TOP
    return [PANEL_X + PANEL_PAD, top + index * ITEM_H, PANEL_W - PANEL_PAD * 2, ITEM_H - 8]

def calib_button_rect():
    return [PANEL_X + PANEL_PAD, 58, PANEL_W - PANEL_PAD * 2, 42]

def clear_button_rect():
    return [PANEL_X + PANEL_PAD, 104, PANEL_W - PANEL_PAD * 2, 42]

def calib_status_y():
    return 152

def select_by_point(x, y, objs):
    if is_in_rect(x, y, page_task_rect()):
        return TASK_PAGE_CODE, -1, -1, -1, -1, -1
    if is_in_rect(x, y, page_calib_rect()):
        return CALIB_PAGE_CODE, -1, -1, -1, -1, -1
    if ui_page == UI_PAGE_TASK:
        if task_controller is not None:
            for i, task_id in enumerate(task_controller.available_tasks()):
                if is_in_rect(x, y, task_item_rect(i)):
                    return TASK_SELECT_CODE, task_id, -1, -1, -1, -1
        return None, None, None, None, None, None

    if is_in_rect(x, y, calib_button_rect()):
        return CALIBRATE_CODE, -1, -1, -1, -1, -1
    if is_in_rect(x, y, clear_button_rect()):
        return SELECT_CLEAR_CODE, -1, -1, -1, -1, -1

    for display_i, slot_i in enumerate(display_slot_indices()):
        slot = candidate_slots[slot_i]
        obj = slot["obj"]
        if obj is None:
            continue
        if is_in_rect(x, y, candidate_item_rect(display_i)):
            cx, cy = obj_center(obj)
            return obj.class_id, cx, cy, obj.w, obj.h, slot_i

    hit_obj = None
    hit_area = 0
    for obj in objs:
        if is_in_rect(x, y, [obj.x, obj.y, obj.w, obj.h]):
            area = obj.w * obj.h
            if hit_obj is None or area < hit_area:
                hit_obj = obj
                hit_area = area

    if hit_obj:
        cx, cy = obj_center(hit_obj)
        slot_index = -1
        for i, slot in enumerate(candidate_slots):
            if slot["class_id"] == hit_obj.class_id:
                slot_index = i
                break
        return hit_obj.class_id, cx, cy, hit_obj.w, hit_obj.h, slot_index

    return None, None, None, None, None, None

def find_selected_obj(objs, class_id, last_cx, last_cy, last_w, last_h, vx, vy, lost_count):
    if class_id < 0:
        return None

    matched = []
    for obj in objs:
        if obj.class_id == class_id:
            matched.append(obj)

    if len(matched) == 0:
        return None

    if last_cx < 0 or last_cy < 0:
        matched.sort(key = lambda obj: obj.score, reverse = True)
        return matched[0]

    pred_cx = last_cx + vx
    pred_cy = last_cy + vy
    max_dist = MAX_MATCH_DIST + lost_count * LOST_MATCH_GROW
    best_obj = None
    best_cost = 0

    for obj in matched:
        cx, cy = obj_center(obj)
        dx = cx - pred_cx
        dy = cy - pred_cy
        dist2 = dx * dx + dy * dy
        if dist2 > max_dist * max_dist:
            continue

        size_cost = 0
        if last_w > 0 and last_h > 0:
            size_cost = abs(obj.w - last_w) + abs(obj.h - last_h)
        score_cost = (1.0 - obj.score) * 100
        cost = dist2 + size_cost * size_cost * SIZE_COST_WEIGHT + score_cost * SCORE_COST_WEIGHT

        if best_obj is None or cost < best_cost:
            best_obj = obj
            best_cost = cost

    return best_obj

def reacquire_selected_obj(objs, class_id, last_cx, last_cy, last_w, last_h):
    if class_id < 0:
        return None

    best_obj = None
    best_cost = 0
    for obj in objs:
        if obj.class_id != class_id:
            continue

        cx, cy = obj_center(obj)
        if last_cx >= 0 and last_cy >= 0:
            dx = cx - last_cx
            dy = cy - last_cy
            dist2 = dx * dx + dy * dy
            if dist2 > REACQUIRE_MAX_DIST * REACQUIRE_MAX_DIST:
                continue
        else:
            dist2 = 0

        size_cost = 0
        if last_w > 0 and last_h > 0:
            size_cost = abs(obj.w - last_w) + abs(obj.h - last_h)
        score_cost = (1.0 - obj.score) * 100
        cost = dist2 + size_cost * size_cost * SIZE_COST_WEIGHT + score_cost * SCORE_COST_WEIGHT

        if best_obj is None or cost < best_cost:
            best_obj = obj
            best_cost = cost

    return best_obj

def yolo_obj_cost(obj, ref_cx, ref_cy, ref_w, ref_h):
    cx, cy = obj_center(obj)
    dx = cx - ref_cx
    dy = cy - ref_cy
    size_cost = 0
    if ref_w > 0 and ref_h > 0:
        size_cost = abs(obj.w - ref_w) + abs(obj.h - ref_h)
    score_cost = (1.0 - obj.score) * 100
    return dx * dx + dy * dy + size_cost * size_cost * SIZE_COST_WEIGHT + score_cost * SCORE_COST_WEIGHT

def choose_unique_yolo_candidate(objs, ref_cx, ref_cy, ref_w, ref_h, max_dist, max_size_delta_ratio, allow_ambiguous = False, selected_only = True):
    if selected_class_id < 0 or ref_cx < 0 or ref_cy < 0:
        return None, "not selected"

    candidates = []
    for obj in objs:
        if selected_only and obj.class_id != selected_class_id:
            continue
        cx, cy = obj_center(obj)
        dx = cx - ref_cx
        dy = cy - ref_cy
        if dx * dx + dy * dy > max_dist * max_dist:
            continue
        if ref_w > 0 and abs(obj.w - ref_w) > ref_w * max_size_delta_ratio:
            continue
        if ref_h > 0 and abs(obj.h - ref_h) > ref_h * max_size_delta_ratio:
            continue
        candidates.append((yolo_obj_cost(obj, ref_cx, ref_cy, ref_w, ref_h), obj))

    if len(candidates) == 0:
        return None, "missing"

    candidates.sort(key = lambda item: item[0])
    best_cost, best_obj = candidates[0]
    if len(candidates) > 1:
        second_cost = candidates[1][0]
        if allow_ambiguous:
            close_margin = abs(best_cost) * 0.45 + 900
        else:
            close_margin = abs(best_cost) * 0.30 + 500
        if second_cost - best_cost < close_margin:
            return None, "ambiguous"

    return best_obj, "ok"

def update_last_yolo_box(obj):
    global last_yolo_cx, last_yolo_cy, last_yolo_w, last_yolo_h, last_yolo_seen_frames
    if obj is None:
        return
    last_yolo_cx, last_yolo_cy = obj_center(obj)
    last_yolo_w = obj.w
    last_yolo_h = obj.h
    last_yolo_seen_frames = 0

def update_last_cv_box(obj):
    global last_cv_cx, last_cv_cy, last_cv_w, last_cv_h, last_cv_seen_frames
    if obj is None:
        return
    last_cv_cx, last_cv_cy = obj_center(obj)
    last_cv_w = obj.w
    last_cv_h = obj.h
    last_cv_seen_frames = 0

def age_last_cv_box():
    global last_cv_seen_frames
    if last_cv_cx >= 0 and last_cv_cy >= 0:
        last_cv_seen_frames += 1

def track_selected_by_observation(track_objs):
    global last_yolo_seen_frames
    if selected_class_id < 0:
        last_yolo_seen_frames += 1
        return None, "not selected"

    ref_cx, ref_cy, ref_w, ref_h = base_track_box()
    if ref_cx < 0 or ref_cy < 0:
        ref_cx = selected_cx
        ref_cy = selected_cy
        ref_w = max(selected_w, MIN_BOX_WIDTH)
        ref_h = max(selected_h, MIN_BOX_HEIGHT)
    if ref_cx < 0 or ref_cy < 0:
        last_yolo_seen_frames += 1
        return None, "no ref"

    max_dist = BANANA_YOLO_OBSERVE_MAX_DIST if is_selected_cv_anchor() else YOLO_OBSERVE_MAX_DIST
    max_size_delta_ratio = BANANA_YOLO_TRACK_MAX_SIZE_DELTA_RATIO if is_selected_cv_anchor() else YOLO_TRACK_FALLBACK_MAX_SIZE_DELTA_RATIO
    obj, reason = choose_unique_yolo_candidate(
        track_objs,
        ref_cx,
        ref_cy,
        ref_w,
        ref_h,
        max_dist,
        max_size_delta_ratio,
        True,
    )
    if obj is None:
        last_yolo_seen_frames += 1
        return None, reason
    return obj, "obs"

def classify_target_measurement(obj, ref_cx, ref_cy, ref_w, ref_h, kready):
    if obj is None:
        return "hold", "missing"
    cx, cy = obj_center(obj)
    if selected_class_id < 0:
        return "switch", "unlocked"
    if obj.class_id != selected_class_id:
        target_switch_gate.miss("class")
        return "hold", "class"
    max_size_delta_ratio = BANANA_YOLO_TRACK_MAX_SIZE_DELTA_RATIO if is_selected_cv_anchor() else YOLO_TRACK_FALLBACK_MAX_SIZE_DELTA_RATIO
    switch_max_dist = BANANA_TARGET_SWITCH_MAX_DIST if is_selected_cv_anchor() else TARGET_SWITCH_MAX_DIST
    if ref_w > 0 and abs(obj.w - ref_w) > ref_w * max_size_delta_ratio:
        target_switch_gate.miss("size")
        return "hold", "size"
    if ref_h > 0 and abs(obj.h - ref_h) > ref_h * max_size_delta_ratio:
        target_switch_gate.miss("size")
        return "hold", "size"
    if not kready or ref_cx < 0 or ref_cy < 0:
        target_switch_gate.reset()
        return "same", "open"

    dx = cx - ref_cx
    dy = cy - ref_cy
    if dx * dx + dy * dy <= switch_max_dist * switch_max_dist:
        target_switch_gate.reset()
        return "same", "same"

    accepted, reason = target_switch_gate.accept(cx, cy, ref_cx, ref_cy, True)
    if accepted and reason == "confirm":
        return "switch", "switch"
    return "hold", reason

def is_yolo_near_last_lock(obj):
    if obj is None:
        return False
    if last_yolo_cx < 0 or last_yolo_cy < 0:
        return True
    cx, cy = obj_center(obj)
    dx = cx - last_yolo_cx
    dy = cy - last_yolo_cy
    max_jump = max(YOLO_LAST_LOCK_MAX_JUMP, int(max(last_yolo_w, last_yolo_h) * 1.45))
    if is_e1_fast_tracking():
        max_jump = max(E1_YOLO_LAST_LOCK_MAX_JUMP, int(max(last_yolo_w, last_yolo_h) * 1.10))
    return dx * dx + dy * dy <= max_jump * max_jump

def verify_track_with_yolo(track_objs, target_obj):
    global last_yolo_seen_frames
    if selected_class_id < 0 or target_obj is None:
        last_yolo_seen_frames += 1
        return False, "missing target"

    ref_cx, ref_cy = obj_center(target_obj)
    ref_w = max(target_obj.w, MIN_BOX_WIDTH)
    ref_h = max(target_obj.h, MIN_BOX_HEIGHT)
    yolo_obj, reason = choose_unique_yolo_candidate(
        track_objs,
        ref_cx,
        ref_cy,
        ref_w,
        ref_h,
        YOLO_VERIFY_MAX_CENTER_DIST,
        YOLO_VERIFY_MAX_SIZE_DELTA_RATIO,
        True,
    )
    if yolo_obj is None:
        last_yolo_seen_frames += 1
        return False, reason

    update_last_yolo_box(yolo_obj)
    return True, "ok"

def track_selected_by_yolo_fallback(track_objs, ref_cx = None, ref_cy = None):
    global last_yolo_seen_frames

    if ref_cx is None or ref_cy is None:
        ref_cx, ref_cy = predicted_track_center()
    _, _, ref_w, ref_h = base_track_box()
    if ref_cx < 0 or ref_cy < 0:
        ref_cx = selected_cx
        ref_cy = selected_cy
        ref_w = max(selected_w, MIN_BOX_WIDTH)
        ref_h = max(selected_h, MIN_BOX_HEIGHT)

    max_dist = YOLO_TRACK_FALLBACK_MAX_DIST
    max_size_delta_ratio = YOLO_TRACK_FALLBACK_MAX_SIZE_DELTA_RATIO
    if is_selected_cv_anchor():
        max_dist = BANANA_YOLO_TRACK_MAX_DIST
        max_size_delta_ratio = BANANA_YOLO_TRACK_MAX_SIZE_DELTA_RATIO
    if is_e1_fast_tracking():
        max_dist = E1_YOLO_TRACK_MAX_DIST
        max_size_delta_ratio = E1_YOLO_TRACK_MAX_SIZE_DELTA_RATIO

    fast_yolo = is_e1_fast_tracking()
    obj, reason = choose_unique_yolo_candidate(
        track_objs,
        ref_cx,
        ref_cy,
        ref_w,
        ref_h,
        max_dist,
        max_size_delta_ratio,
        fast_yolo,
    )
    if obj is None:
        last_yolo_seen_frames += 1
        return None, reason

    return obj, "ok"

def hold_prediction_target():
    if not is_e1_fast_tracking():
        return None
    kalman_pred = kalman_tracker.miss()
    max_miss = BANANA_KALMAN_CONTROL_MAX_MISS_FRAMES if is_selected_cv_anchor() else KALMAN_CONTROL_MAX_MISS_FRAMES
    if kalman_pred is None or suspect_frames > max_miss:
        return None
    pred_cx, pred_cy = kalman_pred
    return box_at_center(pred_cx, pred_cy, selected_w, selected_h, selected_class_id, max(smooth_score, 0.45))

def yolo_missing_hold_frames():
    if is_e1_fast_tracking():
        return E1_YOLO_MISSING_HOLD_FRAMES
    if is_selected_cv_anchor():
        return BANANA_YOLO_MISSING_HOLD_FRAMES
    return YOLO_MISSING_HOLD_FRAMES

def cv_anchor_short_missing_target():
    if not is_selected_cv_anchor() or is_e1_fast_tracking():
        return None
    if suspect_frames > BANANA_CV_SHORT_MISSING_HOLD_FRAMES:
        return None
    if smooth_x < 0 or smooth_y < 0:
        return None
    return smoothed_target_obj()

def held_or_smoothed_prediction():
    target_obj = hold_prediction_target()
    if target_obj is not None:
        return target_obj
    if (is_reaim_running_task() and not is_selected_cv_anchor() and
            smooth_x >= 0 and suspect_frames <= yolo_missing_hold_frames()):
        return smoothed_target_obj()
    return None

def reacquire_with_yolo(track_objs):
    global reacquire_confirm_frames
    global pending_reacquire_cx, pending_reacquire_cy, pending_reacquire_w, pending_reacquire_h

    ref_cx, ref_cy = predicted_track_center()
    _, _, ref_w, ref_h = base_track_box()
    if ref_cx < 0 or ref_cy < 0:
        ref_cx = selected_cx
        ref_cy = selected_cy
        ref_w = max(selected_w, MIN_BOX_WIDTH)
        ref_h = max(selected_h, MIN_BOX_HEIGHT)

    max_dist = REACQUIRE_MAX_DIST
    max_size_delta_ratio = YOLO_VERIFY_MAX_SIZE_DELTA_RATIO
    if is_e1_fast_tracking():
        max_dist = E1_YOLO_TRACK_MAX_DIST
        max_size_delta_ratio = E1_YOLO_TRACK_MAX_SIZE_DELTA_RATIO

    obj, reason = choose_unique_yolo_candidate(
        track_objs,
        ref_cx,
        ref_cy,
        ref_w,
        ref_h,
        max_dist,
        max_size_delta_ratio,
        True,
    )
    if obj is None:
        reacquire_confirm_frames = 0
        return None, reason

    cx, cy = obj_center(obj)
    if pending_reacquire_cx >= 0 and pending_reacquire_cy >= 0:
        dx = cx - pending_reacquire_cx
        dy = cy - pending_reacquire_cy
        if dx * dx + dy * dy > YOLO_REACQUIRE_MAX_PENDING_DIST * YOLO_REACQUIRE_MAX_PENDING_DIST:
            reacquire_confirm_frames = 1
        else:
            reacquire_confirm_frames += 1
    else:
        reacquire_confirm_frames = 1

    pending_reacquire_cx = cx
    pending_reacquire_cy = cy
    pending_reacquire_w = obj.w
    pending_reacquire_h = obj.h

    confirm_frames = YOLO_REACQUIRE_CONFIRM_FRAMES
    if is_e1_fast_tracking():
        confirm_frames = E1_YOLO_REACQUIRE_CONFIRM_FRAMES
    if reacquire_confirm_frames >= confirm_frames:
        return obj, "confirmed"
    return None, "confirming"

def get_slot_obj(slot_index, class_id):
    if slot_index < 0 or slot_index >= len(candidate_slots):
        return None
    if candidate_slots[slot_index]["lost"] != 0:
        return None
    obj = candidate_slots[slot_index]["obj"]
    if obj is None:
        return None
    if obj.class_id != class_id:
        return None
    return obj

def box_at_center(cx, cy, w, h, class_id, score):
    w = clamp(max(w, MIN_BOX_WIDTH), MIN_BOX_WIDTH, PANEL_X)
    h = clamp(max(h, MIN_BOX_HEIGHT), MIN_BOX_HEIGHT, CAMERA_HEIGHT)
    x = clamp(cx - w // 2, 0, PANEL_X - w)
    y = clamp(cy - h // 2, 0, CAMERA_HEIGHT - h)
    return TrackedObj(x, y, w, h, class_id, score)

def should_accept_track_obj(obj, ref_cx, ref_cy):
    if obj is None:
        return False
    if smooth_x < 0 or smooth_y < 0:
        return True
    if obj.score < TRACK_ACCEPT_MIN_SCORE:
        return False

    cx, cy = obj_center(obj)
    if ref_cx >= 0 and ref_cy >= 0:
        dx = cx - ref_cx
        dy = cy - ref_cy
        if dx * dx + dy * dy > TRACK_ACCEPT_MAX_CENTER_JUMP * TRACK_ACCEPT_MAX_CENTER_JUMP:
            return False

    base_w = max(selected_w, MIN_BOX_WIDTH)
    base_h = max(selected_h, MIN_BOX_HEIGHT)
    if abs(obj.w - base_w) > base_w * TRACK_ACCEPT_MAX_SIZE_DELTA_RATIO:
        return False
    if abs(obj.h - base_h) > base_h * TRACK_ACCEPT_MAX_SIZE_DELTA_RATIO:
        return False

    return True

def smooth_box(obj):
    global smooth_x, smooth_y, smooth_w, smooth_h, smooth_score

    if smooth_x < 0:
        smooth_x = obj.x
        smooth_y = obj.y
        smooth_w = obj.w
        smooth_h = obj.h
    else:
        smooth_x = int(smooth_x * SMOOTH_ALPHA + obj.x * (1 - SMOOTH_ALPHA))
        smooth_y = int(smooth_y * SMOOTH_ALPHA + obj.y * (1 - SMOOTH_ALPHA))
        smooth_w = obj.w
        smooth_h = obj.h
    smooth_score = obj.score

def snap_smooth_box(obj):
    global smooth_x, smooth_y, smooth_w, smooth_h, smooth_score
    smooth_x = obj.x
    smooth_y = obj.y
    smooth_w = obj.w
    smooth_h = obj.h
    smooth_score = obj.score

def center_delta_from_smooth(cx, cy):
    if smooth_x < 0 or smooth_y < 0:
        return OUTPUT_CENTER_DEADBAND_PIXELS + 1
    old_cx = smooth_x + smooth_w // 2
    old_cy = smooth_y + smooth_h // 2
    dx = cx - old_cx
    dy = cy - old_cy
    return int((dx * dx + dy * dy) ** 0.5)

def is_small_rect_jitter(obj):
    if obj is None or smooth_x < 0 or smooth_y < 0:
        return False
    return (
        abs(obj.x - smooth_x) <= RECT_JITTER_DEADBAND_PIXELS and
        abs(obj.y - smooth_y) <= RECT_JITTER_DEADBAND_PIXELS and
        abs(obj.w - smooth_w) <= RECT_SIZE_JITTER_DEADBAND_PIXELS and
        abs(obj.h - smooth_h) <= RECT_SIZE_JITTER_DEADBAND_PIXELS
    )

def reset_smooth_box():
    global smooth_x, smooth_y, smooth_w, smooth_h, smooth_score, lost_frames, track_vx, track_vy
    global track_ref_pitch, track_ref_yaw
    global last_good_cx, last_good_cy, last_good_w, last_good_h
    global last_yolo_cx, last_yolo_cy, last_yolo_w, last_yolo_h, last_yolo_seen_frames
    global last_cv_cx, last_cv_cy, last_cv_w, last_cv_h, last_cv_seen_frames
    global roi_fail_frames, roi_ambiguous_frames, suspect_frames, reacquire_confirm_frames
    global pending_reacquire_cx, pending_reacquire_cy, pending_reacquire_w, pending_reacquire_h
    smooth_x = -1
    smooth_y = -1
    smooth_w = -1
    smooth_h = -1
    smooth_score = 0
    lost_frames = 0
    track_vx = 0
    track_vy = 0
    track_ref_pitch = stat["pitch_abs"]
    track_ref_yaw = stat["yaw_abs"]
    last_good_cx = -1
    last_good_cy = -1
    last_good_w = -1
    last_good_h = -1
    last_yolo_cx = -1
    last_yolo_cy = -1
    last_yolo_w = -1
    last_yolo_h = -1
    last_yolo_seen_frames = 0
    last_cv_cx = -1
    last_cv_cy = -1
    last_cv_w = -1
    last_cv_h = -1
    last_cv_seen_frames = 0
    roi_fail_frames = 0
    roi_ambiguous_frames = 0
    suspect_frames = 0
    reset_reacquire_pending()
    kalman_tracker.reset()
    reset_track_identity_gates()
    dynamic_color_model.clear()

def held_target_obj():
    if selected_class_id < 0 or smooth_x < 0 or smooth_y < 0:
        return None
    if lost_frames > LOST_HOLD_FRAMES:
        return None
    cx = smooth_x + smooth_w // 2
    cy = smooth_y + smooth_h // 2
    return box_at_center(cx, cy, smooth_w, smooth_h, selected_class_id, smooth_score)

def smoothed_target_obj():
    if selected_class_id < 0 or smooth_x < 0 or smooth_y < 0:
        return None
    return TrackedObj(smooth_x, smooth_y, smooth_w, smooth_h, selected_class_id, smooth_score)

def reset_anchor_lock():
    global anchor_frozen, anchor_stable_frames
    anchor_frozen = False
    anchor_stable_frames = 0

def reset_calibration():
    global aim_ref_x, aim_ref_y, aim_ref_pitch, aim_ref_yaw, aim_calibrated
    aim_ref_x = AIM_CENTER_X
    aim_ref_y = AIM_CENTER_Y
    aim_ref_pitch = stat["pitch_abs"]
    aim_ref_yaw = stat["yaw_abs"]
    aim_calibrated = False

def reset_tx_filter():
    global last_tx_packet, last_valid_tx_ms, last_valid_tx_flag
    global safe_tx_dx, safe_tx_dy, safe_tx_jump_count, safe_tx_pending_dx, safe_tx_pending_dy
    global circle_tx_residual_x, circle_tx_residual_y
    global circle_tx_enabled
    last_tx_packet = (safe_tx_dx, safe_tx_dy) if (safe_tx_dx != 0 or safe_tx_dy != 0) else None
    last_valid_tx_ms = -1
    last_valid_tx_flag = 1
    safe_tx_jump_count = 0
    safe_tx_pending_dx = safe_tx_dx
    safe_tx_pending_dy = safe_tx_dy
    circle_tx_residual_x = 0.0
    circle_tx_residual_y = 0.0
    if circle_tx_lock is not None:
        circle_tx_lock.acquire()
    try:
        circle_tx_enabled = False
        circle_tx_interp.reset()
    finally:
        if circle_tx_lock is not None:
            circle_tx_lock.release()

def reset_target_control_chain():
    reset_tx_filter()
    reset_reacquire_pending()
    kalman_tracker.reset()
    reset_track_identity_gates()
    if target_center_finder is not None:
        target_center_finder.reset()

def queue_done_signal():
    global done_signal_pending, task_done_latched
    if not task_done_latched:
        done_signal_pending = max(done_signal_pending, DONE_SEND_REPEAT)
        task_done_latched = True

def cancel_done_signal():
    global done_signal_pending, task_done_latched
    done_signal_pending = 0
    task_done_latched = False

def calibrate_visual_anchor():
    global selected_cx, selected_cy, anchor_frozen, anchor_stable_frames
    global aim_ref_x, aim_ref_y, aim_ref_pitch, aim_ref_yaw, aim_calibrated
    global track_ref_pitch, track_ref_yaw
    if smooth_x >= 0 and smooth_y >= 0:
        selected_cx = smooth_x + smooth_w // 2
        selected_cy = smooth_y + smooth_h // 2
        aim_ref_x = selected_cx
        aim_ref_y = selected_cy
        aim_ref_pitch = stat["pitch_abs"]
        aim_ref_yaw = stat["yaw_abs"]
        track_ref_pitch = stat["pitch_abs"]
        track_ref_yaw = stat["yaw_abs"]
        aim_calibrated = True
        anchor_frozen = False
        anchor_stable_frames = 0
    else:
        reset_anchor_lock()
    reset_tx_filter()

def update_anchor_lock(cx, cy):
    global selected_cx, selected_cy, anchor_frozen, anchor_stable_frames
    global track_ref_pitch, track_ref_yaw
    global last_good_cx, last_good_cy, last_good_w, last_good_h
    selected_cx = cx
    selected_cy = cy
    last_good_cx = cx
    last_good_cy = cy
    if selected_w > 0:
        last_good_w = selected_w
    if selected_h > 0:
        last_good_h = selected_h
    track_ref_pitch = stat["pitch_abs"]
    track_ref_yaw = stat["yaw_abs"]
    anchor_frozen = False
    anchor_stable_frames = 0

def clear_selection(reset_direction=True):
    global selected_class_id, selected_cx, selected_cy, selected_w, selected_h, selected_slot_index
    global track_ref_pitch, track_ref_yaw
    selected_class_id = -1
    selected_cx = -1
    selected_cy = -1
    selected_w = -1
    selected_h = -1
    selected_slot_index = -1
    track_ref_pitch = stat["pitch_abs"]
    track_ref_yaw = stat["yaw_abs"]
    reset_smooth_box()
    reset_anchor_lock()
    if reset_direction:
        reset_preaim()
        reset_center_refine_gate()

def slot_index_for_class(class_id):
    for i, slot in enumerate(candidate_slots):
        if slot["class_id"] == class_id:
            return i
    return -1

def select_target_obj(obj, img, reason = "task"):
    global selected_class_id, selected_cx, selected_cy, selected_w, selected_h, selected_slot_index
    global track_ref_pitch, track_ref_yaw
    if obj is None:
        return False

    cx, cy = obj_center(obj)
    target_changed = (
        selected_class_id != obj.class_id or
        selected_cx < 0 or
        selected_cy < 0 or
        (cx - selected_cx) * (cx - selected_cx) + (cy - selected_cy) * (cy - selected_cy) >
        YOLO_REACQUIRE_MAX_PENDING_DIST * YOLO_REACQUIRE_MAX_PENDING_DIST
    )
    if target_changed:
        reset_target_control_chain()
    selected_class_id = obj.class_id
    selected_cx = cx
    selected_cy = cy
    selected_w = obj.w
    selected_h = obj.h
    selected_slot_index = slot_index_for_class(obj.class_id)
    track_ref_pitch = stat["pitch_abs"]
    track_ref_yaw = stat["yaw_abs"]
    reset_smooth_box()
    seed_obj = box_at_center(cx, cy, obj.w, obj.h, obj.class_id, max(obj.score, 0.80))
    kalman_tracker.init(cx, cy)
    reset_track_identity_gates()
    update_last_yolo_box(seed_obj)
    update_anchor_lock(cx, cy)
    smooth_box(seed_obj)
    dynamic_color_model.learn(img, seed_obj, PANEL_X, CAMERA_HEIGHT)
    set_vision_state(VSTATE_TRACK, reason)
    return True

def touch_to_image_pos(x, y, img):
    if x >= dis.width() or y >= dis.height():
        return int(x), int(y)
    mapped = image.resize_map_pos_reverse(img.width(), img.height(), dis.width(), dis.height(), image.Fit.FIT_CONTAIN, x, y)
    return int(mapped[0]), int(mapped[1])

def handle_touch_press(raw_x, raw_y, img, objs):
    global touch_x, touch_y, last_touch_action_ms
    global selected_class_id, selected_cx, selected_cy, selected_w, selected_h, selected_slot_index
    global track_ref_pitch, track_ref_yaw
    global ui_page

    now = time.ticks_ms()
    if now - last_touch_action_ms < TOUCH_DEBOUNCE_MS:
        return

    touch_x, touch_y = touch_to_image_pos(raw_x, raw_y, img)
    class_id, cx, cy, obj_w, obj_h, slot_index = select_by_point(touch_x, touch_y, objs)
    if class_id is None:
        return

    last_touch_action_ms = now
    if class_id == TASK_PAGE_CODE:
        ui_page = UI_PAGE_TASK
        return
    if class_id == CALIB_PAGE_CODE:
        ui_page = UI_PAGE_CALIB
        return
    if class_id == CALIBRATE_CODE:
        calibrate_visual_anchor()
        return
    if class_id == SELECT_CLEAR_CODE:
        clear_selection(True)
        return
    if class_id == TASK_SELECT_CODE:
        if task_controller is not None:
            if task_controller.select_task(cx, time.ticks_ms(), stat["state"]):
                cancel_done_signal()
                clear_selection(True)
                reset_tx_filter()
                log("task selected -> %s" % task_controller.status_text(display_label))
            else:
                log("task select blocked: %s" % task_controller.status_text(display_label))
        return
    panel_select = touch_x >= PANEL_X
    if selected_class_id >= 0 and vision_track_state != VSTATE_SEARCH and not panel_select:
        return
    seed_obj = box_at_center(cx, cy, obj_w, obj_h, class_id, 0.80)
    if select_target_obj(seed_obj, img, "selected"):
        selected_slot_index = slot_index

def draw_label(img, x, y, msg, color):
    text_w = len(msg) * 8 * TEXT_SCALE
    text_h = 10 * TEXT_SCALE
    label_y = y - text_h - 6 * UI_SCALE
    if label_y < 0:
        label_y = y + 2 * UI_SCALE
    img.draw_rect(x, label_y, text_w + 6 * UI_SCALE, text_h + 6 * UI_SCALE, color = COLOR_BLACK, thickness = -1)
    img.draw_string(x + 3 * UI_SCALE, label_y + 3 * UI_SCALE, msg, color = color, scale = TEXT_SCALE)

def draw_index_badge(img, obj, index):
    badge_w = 28 * UI_SCALE
    badge_h = 24 * UI_SCALE
    x = obj.x
    y = obj.y - badge_h - 2 * UI_SCALE
    if y < 0:
        y = obj.y + 2 * UI_SCALE
    img.draw_rect(x, y, badge_w, badge_h, color = COLOR_BLACK, thickness = -1)
    img.draw_rect(x, y, badge_w, badge_h, color = COLOR_YELLOW, thickness = 2 * UI_SCALE)
    img.draw_string(x + 8 * UI_SCALE, y + 3 * UI_SCALE, str(index), color = COLOR_YELLOW, scale = 2 * UI_SCALE)

def draw_candidate_markers(img, objs):
    for display_i, slot_i in enumerate(display_slot_indices()):
        slot = candidate_slots[slot_i]
        obj = slot["obj"]
        if obj is None or slot["lost"] != 0:
            continue
        img.draw_rect(obj.x, obj.y, obj.w, obj.h, color = COLOR_YELLOW, thickness = UI_SCALE)
        draw_index_badge(img, obj, display_i + 1)

def draw_smoothed_target(img, class_id):
    if smooth_x < 0:
        return
    img.draw_rect(smooth_x, smooth_y, smooth_w, smooth_h, color = image.COLOR_RED, thickness = BOX_THICKNESS)
    msg = f'{display_label(class_id)}: {smooth_score:.2f}'
    draw_label(img, smooth_x, smooth_y, msg, image.COLOR_RED)

def draw_target_crosses(img, target_obj):
    ax, ay = aim_point()
    img.draw_cross(ax, ay, color = COLOR_BLACK, size = 22, thickness = 4)
    img.draw_cross(ax, ay, color = COLOR_WHITE, size = 18, thickness = 2)
    point = target_center_point(target_obj)
    if point is not None:
        px, py = point[0], point[1]
        img.draw_cross(px, py, color = COLOR_BLACK, size = 28, thickness = 5)
        img.draw_cross(px, py, color = COLOR_BLUE_BRIGHT, size = 24, thickness = 3)
        img.draw_circle(px, py, 9, color = COLOR_BLACK, thickness = 3)
        img.draw_circle(px, py, 8, color = COLOR_BLUE, thickness = 2)

def draw_target_center_result(img):
    runtime = task_controller.runtime if task_controller is not None else None
    if (runtime is not None and
            task_controller.task_id == TASK_EXT_CIRCLE and
            runtime.circle_trace_cx >= 0 and runtime.circle_trace_cy >= 0 and
            runtime.circle_trace_rx > 0):
        cx = int(runtime.circle_trace_cx + 0.5)
        cy = int(runtime.circle_trace_cy + 0.5)
        radius = int(max(runtime.circle_trace_rx, runtime.circle_trace_ry) + 0.5)
        img.draw_cross(cx, cy, color = COLOR_BLACK, size = 30, thickness = 5)
        img.draw_cross(cx, cy, color = image.COLOR_GREEN, size = 26, thickness = 3)
        img.draw_circle(cx, cy, radius, color = image.COLOR_GREEN, thickness = 2)
        draw_label(img, cx + 8, cy, "circle", image.COLOR_GREEN)
        return
    if target_center_result is None:
        return
    if target_center_result.roi is not None:
        roi = target_center_result.roi
        img.draw_rect(roi[0], roi[1], roi[2], roi[3], color = COLOR_YELLOW, thickness = 1)
    if target_center_result.valid:
        cx = target_center_result.cx
        cy = target_center_result.cy
        if task_controller is not None and task_controller.task_id != TASK_EXT_CIRCLE:
            center = roi_diagonal_center(target_center_result.roi)
            if center is None:
                return
            cx, cy = center[0], center[1]
        elif target_center_result.roi is not None and not point_in_roi(cx, cy, target_center_result.roi):
            return
        img.draw_cross(cx, cy, color = COLOR_BLACK, size = 30, thickness = 5)
        center_color = image.COLOR_GREEN if (task_controller is not None and task_controller.task_id == TASK_EXT_CIRCLE) else COLOR_BLUE_BRIGHT
        img.draw_cross(cx, cy, color = center_color, size = 26, thickness = 3)
        if (target_center_result.radius > 0 and
                task_controller is not None and
                task_controller.task_id == TASK_EXT_CIRCLE):
            img.draw_circle(cx,
                            cy,
                            target_center_result.radius,
                            color = image.COLOR_GREEN,
                            thickness = 2)
        draw_label(img, cx + 8, cy, "center", center_color)
    else:
        img.draw_rect(4 * UI_SCALE, 44 * UI_SCALE, 220 * UI_SCALE, 24 * UI_SCALE, color = COLOR_BLACK, thickness = -1)
        img.draw_string(8 * UI_SCALE, 48 * UI_SCALE, "center:%s" % target_center_result.reason, color = COLOR_LOST, scale = UI_SCALE)

def draw_uart_overlay(img, target_obj):
    if last_tx_packet is None:
        tx_dx = 0
        tx_dy = 0
    else:
        tx_dx, tx_dy = last_tx_packet

    raw_dx = 0
    raw_dy = 0
    dir_hint = "CC"
    if target_obj is not None:
        offset = target_offset(target_obj)
        if offset is not None:
            raw_dx, raw_dy = offset
            dir_hint = direction_hint(raw_dx, raw_dy)

    draw_target_crosses(img, target_obj)
    lines = [
        "S:%s K:%d RX:%d TX:%d" % (state_name(), stat["key"], stat["rx"], stat["tx"]),
        "DIR:%s raw:%+4d,%+4d" % (dir_hint, raw_dx, raw_dy),
        "PRE:%s %+4d,%+4d CG:%s" % (preaim_dir, preaim_dx, preaim_dy, center_refine_gate_reason),
        "TX dx:%+4d dy:%+4d" % (tx_dx, tx_dy),
        "MCU dx:%+4d dy:%+4d" % (stat["dx"], stat["dy"]),
        "P:%6.2f Y:%6.2f" % (stat["pitch_abs"], stat["yaw_abs"]),
    ]
    x = 8
    y = 8
    for line in lines:
        text_w = len(line) * 8 * UI_SCALE
        img.draw_rect(x - 2, y - 2, text_w + 6, 14, color = COLOR_BLACK, thickness = -1)
        img.draw_string(x, y, line, color = COLOR_WHITE, scale = UI_SCALE)
        y += 15

def draw_side_panel(img, objs, target_obj, class_id):
    img.draw_rect(PANEL_X, 0, PANEL_W, CAMERA_HEIGHT, color = COLOR_PANEL_BG, thickness = -1)

    task_tab = page_task_rect()
    calib_tab = page_calib_rect()
    task_bg = COLOR_ITEM_SELECTED_BG if ui_page == UI_PAGE_TASK else COLOR_BUTTON_BG
    calib_bg = COLOR_ITEM_SELECTED_BG if ui_page == UI_PAGE_CALIB else COLOR_BUTTON_BG
    img.draw_rect(task_tab[0], task_tab[1], task_tab[2], task_tab[3], color = task_bg, thickness = -1)
    img.draw_rect(task_tab[0], task_tab[1], task_tab[2], task_tab[3], color = COLOR_BUTTON_BORDER, thickness = 1)
    img.draw_string(task_tab[0] + 12, task_tab[1] + 8, "Task", color = COLOR_WHITE, scale = PANEL_TEXT_SCALE)
    img.draw_rect(calib_tab[0], calib_tab[1], calib_tab[2], calib_tab[3], color = calib_bg, thickness = -1)
    img.draw_rect(calib_tab[0], calib_tab[1], calib_tab[2], calib_tab[3], color = COLOR_BUTTON_BORDER, thickness = 1)
    img.draw_string(calib_tab[0] + 12, calib_tab[1] + 8, "Aim", color = COLOR_WHITE, scale = PANEL_TEXT_SCALE)

    if ui_page == UI_PAGE_TASK:
        status = "MCU:%s  %s" % (state_name(), task_controller.status_text(display_label) if task_controller else "")
        img.draw_string(PANEL_X + PANEL_PAD, 56, status, color = COLOR_STATUS, scale = UI_SCALE)
        if task_controller is not None:
            for i, task_id in enumerate(task_controller.available_tasks()):
                rect = task_item_rect(i)
                selected = task_controller.selected and task_controller.task_id == task_id
                bg = COLOR_ITEM_SELECTED_BG if selected else COLOR_ITEM_BG
                border = image.COLOR_GREEN if selected else COLOR_ITEM_BORDER
                name = TASK_NAMES.get(task_id, "Task")
                hint = "Tap select"
                if selected:
                    hint = "Press USER"
                img.draw_rect(rect[0], rect[1], rect[2], rect[3], color = bg, thickness = -1)
                img.draw_rect(rect[0], rect[1], rect[2], rect[3], color = border, thickness = 2)
                img.draw_string(rect[0] + 10, rect[1] + 7, name, color = COLOR_WHITE, scale = PANEL_TEXT_SCALE)
                img.draw_string(rect[0] + 10, rect[1] + 31, hint, color = COLOR_DETAIL, scale = UI_SCALE)
        return

    calib_rect = calib_button_rect()
    img.draw_rect(calib_rect[0], calib_rect[1], calib_rect[2], calib_rect[3], color = COLOR_BUTTON_BG, thickness = -1)
    img.draw_rect(calib_rect[0], calib_rect[1], calib_rect[2], calib_rect[3], color = COLOR_BUTTON_BORDER, thickness = 2)
    img.draw_string(calib_rect[0] + 12, calib_rect[1] + 10, "Calibrate", color = COLOR_WHITE, scale = PANEL_TEXT_SCALE)

    clear_rect = clear_button_rect()
    clear_text = "Clear Target" if class_id >= 0 else "Pick target"
    img.draw_rect(clear_rect[0], clear_rect[1], clear_rect[2], clear_rect[3], color = COLOR_BUTTON_BG, thickness = -1)
    img.draw_rect(clear_rect[0], clear_rect[1], clear_rect[2], clear_rect[3], color = COLOR_BUTTON_BORDER, thickness = 2)
    img.draw_string(clear_rect[0] + 12, clear_rect[1] + 10, clear_text, color = COLOR_WHITE, scale = PANEL_TEXT_SCALE)

    if task_controller is not None:
        status = task_controller.status_text(display_label)
        if target_center_result is not None:
            status = "%s %s" % (status, target_center_result.reason)
    elif class_id >= 0:
        status = vision_state_name()
    else:
        status = "Candidates"
    img.draw_string(PANEL_X + PANEL_PAD, calib_status_y(), status, color = COLOR_STATUS, scale = PANEL_TEXT_SCALE)

    for display_i, slot_i in enumerate(display_slot_indices()):
        slot = candidate_slots[slot_i]
        obj = slot["obj"]
        rect = candidate_item_rect(display_i)
        label = display_label(slot["class_id"])
        if len(label) > 11:
            label = label[:11]
        title = f'{display_i + 1}. {label}'
        if obj is None:
            img.draw_rect(rect[0], rect[1], rect[2], rect[3], color = COLOR_ITEM_BG, thickness = -1)
            img.draw_rect(rect[0], rect[1], rect[2], rect[3], color = COLOR_ITEM_BORDER, thickness = 1)
            img.draw_string(rect[0] + 10, rect[1] + 8, title, color = COLOR_MUTED, scale = PANEL_TEXT_SCALE)
            img.draw_string(rect[0] + 10, rect[1] + 38, "--", color = COLOR_MUTED, scale = PANEL_TEXT_SCALE)
            continue

        selected = slot_i == selected_slot_index
        bg = COLOR_ITEM_SELECTED_BG if selected else COLOR_ITEM_BG
        border = image.COLOR_GREEN if selected else COLOR_ITEM_BORDER
        detail = f'{obj.score:.2f}  {obj.w}x{obj.h}'

        img.draw_rect(rect[0], rect[1], rect[2], rect[3], color = bg, thickness = -1)
        img.draw_rect(rect[0], rect[1], rect[2], rect[3], color = border, thickness = 2)
        img.draw_string(rect[0] + 10, rect[1] + 8, title, color = COLOR_WHITE, scale = PANEL_TEXT_SCALE)
        img.draw_string(rect[0] + 10, rect[1] + 38, detail, color = COLOR_DETAIL, scale = PANEL_TEXT_SCALE)

def read_mud_model_type(path):
    try:
        with open(path, "r") as f:
            for line in f:
                line = line.strip()
                if line.startswith("model_type"):
                    parts = line.split("=", 1)
                    if len(parts) == 2:
                        return parts[1].strip().lower()
    except Exception:
        pass
    return ""

def create_detector(path):
    model_type = read_mud_model_type(path)
    if model_type == "yolo11":
        if not hasattr(nn, "YOLO11"):
            raise RuntimeError("MaixPy firmware does not support nn.YOLO11")
        return nn.YOLO11(model = path, dual_buff = True)
    return nn.YOLOv5(model = path)

def resolve_model_path():
    candidates = [
        MODEL_FILE,
        "./" + MODEL_FILE,
        "/root/models/%s/%s" % (MODEL_NAME, MODEL_FILE),
        "/root/models/jixun2/%s" % MODEL_FILE,
        "/root/models/maixhub/277595/%s" % MODEL_FILE,
        "jixun2.mud",
        "./jixun2.mud",
        "/root/models/jixun2/jixun2.mud",
        "/root/models/maixhub/277595/jixun2.mud",
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    raise RuntimeError("model not found: %s" % MODEL_FILE)

def validate_detector_labels():
    labels = tuple(detector.labels)
    if labels != EXPECTED_LABELS:
        raise RuntimeError("label mismatch: %s" % ",".join(labels))

def run():
    global detector, cam, dis, ts, p, model_path
    global touch_x, touch_y, touch_last_pressed
    global track_ref_pitch, track_ref_yaw, track_vx, track_vy, lost_frames
    global roi_fail_frames, roi_ambiguous_frames, suspect_frames, reacquire_confirm_frames
    global last_yolo_seen_frames
    global smooth_score
    global yolo_frame_div
    global last_yolo_infer_ms
    global done_signal_pending, task_done_latched
    global last_udp_debug_ms, last_console_debug_ms
    global task_controller, target_center_finder, target_center_result

    model_path = resolve_model_path()
    detector = create_detector(model_path)
    validate_detector_labels()
    log("model %s labels=%d input=%dx%d" % (model_path, len(detector.labels), detector.input_width(), detector.input_height()))
    task_controller = BTaskController(detector.labels)
    target_center_finder = TargetCenterFinder(BLACK_THRESHOLDS)
    log("task %s" % task_controller.status_text(display_label))

    cam = camera.Camera(CAMERA_WIDTH, CAMERA_HEIGHT, detector.input_format())
    dis = display.Display()
    ts = touchscreen.TouchScreen()

    p = comm.CommProtocol(buff_size = 1024)
    init_candidate_slots()
    init_uart()
    if UDP_DEBUG_ENABLE:
        if udp_debug.init():
            log("udp debug %s:%d" % (UDP_DEBUG_HOST, UDP_DEBUG_PORT))
        else:
            log("udp debug disabled: socket init failed")

    objs = []
    raw_objs = []
    track_objs = []
    task_objs = []
    while not app.need_exit():
        loop_now_ms = time.ticks_ms()
        update_loop_fps(loop_now_ms)
        poll_uart()
        if task_controller is not None:
            now_ms = loop_now_ms
            if task_controller.sync_mcu_state(stat["state"], now_ms):
                cancel_done_signal()
                begin_center_refine_start_gate(now_ms)
                clear_selection(False)
                reset_tx_filter()

        img = apply_digital_zoom(cam.read())
        x, y, pressed = ts.read()
        pressed = bool(pressed)
        if pressed and not touch_last_pressed:
            handle_touch_press(x, y, img, objs)
        touch_last_pressed = pressed

        need_yolo = selected_class_id < 0 or vision_track_state in (VSTATE_SEARCH, VSTATE_LOST, VSTATE_REACQUIRE)
        if not need_yolo:
            yolo_frame_div += 1
            if yolo_frame_div >= YOLO_TRACK_INTERVAL_FRAMES:
                yolo_frame_div = 0
                need_yolo = True
        else:
            yolo_frame_div = 0

        yolo_fresh = False
        if need_yolo:
            detect_img = mask_laser_for_detector(img)
            yolo_start_ms = time.ticks_ms()
            raw_objs = detector.detect(detect_img, conf_th = CONF_TH, iou_th = IOU_TH)
            mark_yolo_timing(yolo_start_ms, time.ticks_ms())
            detected_objs = filter_objs(raw_objs)
            objs = update_candidate_slots(detected_objs)
            track_objs = filter_track_objs(raw_objs)
            task_objs = detected_objs
            yolo_fresh = True
        else:
            last_yolo_seen_frames += 1

        if selected_class_id < 0:
            set_vision_state(VSTATE_SEARCH)

        if task_controller is not None and task_controller.selected and not task_controller.running:
            preaim_obj = best_preaim_candidate(task_objs if len(task_objs) > 0 else objs)
            update_preaim_if_waiting(preaim_obj, time.ticks_ms())

        if task_controller is not None:
            task_obj = task_controller.pick_target(task_objs if len(task_objs) > 0 else objs,
                                                  selected_class_id,
                                                  time.ticks_ms(),
                                                  selected_cx,
                                                  selected_cy)
            if task_obj is not None:
                select_target_obj(task_obj, img, "task")

        target_obj = None
        output_valid = False
        update_lock = False
        snap_output = False
        debug_yolo_obj = None
        debug_yolo_reason = "none"
        if is_e1_fast_tracking():
            kalman_predicted = kalman_tracker.predict()
        else:
            kalman_predicted = None

        if selected_class_id >= 0 and vision_track_state in (VSTATE_TRACK, VSTATE_SUSPECT):
            ref_cx, ref_cy = predicted_track_center()
            kready, _, kpx, kpy, _, _, _, _, _, _, _, _, _ = kalman_tracker.debug_values()
            if is_e1_fast_tracking() and kready and kpx >= 0 and kpy >= 0:
                ref_cx, ref_cy = kpx, kpy
            if yolo_fresh:
                if is_e1_fast_tracking():
                    yolo_obj, yolo_reason = track_selected_by_yolo_fallback(track_objs, ref_cx, ref_cy)
                else:
                    yolo_obj, yolo_reason = track_selected_by_observation(track_objs)
            else:
                yolo_obj, yolo_reason = None, "skip"
            debug_yolo_obj = yolo_obj
            debug_yolo_reason = yolo_reason
            if is_selected_cv_anchor() and not is_e1_fast_tracking():
                special_label = display_label(selected_class_id)
                if yolo_obj is not None:
                    yolo_cx, yolo_cy = obj_center(yolo_obj)
                    update_last_yolo_box(yolo_obj)
                    update_last_cv_box(yolo_obj)
                    dynamic_color_model.learn(img, yolo_obj, PANEL_X, CAMERA_HEIGHT)
                    kalman_tracker.init(yolo_cx, yolo_cy)
                    target_obj = banana_yolo_box_for_output(yolo_obj)
                    output_valid = True
                    update_lock = True
                    roi_fail_frames = 0
                    roi_ambiguous_frames = 0
                    suspect_frames = 0
                    target_switch_gate.reset()
                    reset_reacquire_pending()
                    set_vision_state(VSTATE_TRACK, special_label + " yolo")
                    debug_yolo_reason = yolo_reason + "/" + special_label + "/yolo"
                else:
                    cv_obj, cv_reason = track_selected_banana_by_cv(img)
                    debug_yolo_reason = yolo_reason + "/" + special_label + "/" + cv_reason
                    if cv_obj is not None:
                        cv_cx, cv_cy = obj_center(cv_obj)
                        kalman_tracker.init(cv_cx, cv_cy)
                        update_last_cv_box(cv_obj)
                        target_obj = cv_obj
                        output_valid = True
                        update_lock = True
                        roi_fail_frames = 0
                        roi_ambiguous_frames = 0
                        suspect_frames = 0
                        target_switch_gate.reset()
                        reset_reacquire_pending()
                        set_vision_state(VSTATE_TRACK, special_label + " cv")
                    else:
                        age_last_cv_box()
                        target_switch_gate.miss(yolo_reason)
                        roi_fail_frames += 1
                        if cv_reason == "ambiguous":
                            roi_ambiguous_frames += 1
                        suspect_frames += 1
                        target_obj = cv_anchor_short_missing_target()
                        if target_obj is not None:
                            output_valid = True
                            update_lock = False
                            set_vision_state(VSTATE_TRACK, special_label + " short hold")
                        else:
                            target_obj = None
                            set_vision_state(VSTATE_SUSPECT, special_label + " " + cv_reason)
            elif yolo_obj is not None:
                yolo_cx, yolo_cy = obj_center(yolo_obj)
                _, _, ref_w, ref_h = base_track_box()
                fast_yolo = is_e1_fast_tracking()
                if fast_yolo:
                    target_state, gate_reason = classify_target_measurement(
                        yolo_obj,
                        ref_cx,
                        ref_cy,
                        ref_w,
                        ref_h,
                        kready,
                    )
                else:
                    target_state, gate_reason = "same", "obs"
                debug_yolo_reason = yolo_reason + "/" + target_state + "/" + gate_reason
                if target_state == "same":
                    if is_e1_fast_tracking():
                        out_cx, out_cy = kalman_tracker.update(yolo_cx, yolo_cy)
                    else:
                        kalman_tracker.init(yolo_cx, yolo_cy)
                        out_cx, out_cy = yolo_cx, yolo_cy
                    target_obj = box_at_center(out_cx, out_cy, selected_w, selected_h, yolo_obj.class_id, yolo_obj.score)
                    output_valid = True
                    update_lock = True
                    update_last_yolo_box(yolo_obj)
                    dynamic_color_model.learn(img, yolo_obj, PANEL_X, CAMERA_HEIGHT)
                    roi_fail_frames = 0
                    roi_ambiguous_frames = 0
                    suspect_frames = 0
                    reset_reacquire_pending()
                    set_vision_state(VSTATE_TRACK, "yolo")
                elif target_state == "switch":
                    kalman_tracker.init(yolo_cx, yolo_cy)
                    reset_track_identity_gates()
                    target_obj = box_at_center(yolo_cx, yolo_cy, selected_w, selected_h, yolo_obj.class_id, yolo_obj.score)
                    output_valid = True
                    update_lock = True
                    snap_output = True
                    update_last_yolo_box(yolo_obj)
                    update_anchor_lock(yolo_cx, yolo_cy)
                    dynamic_color_model.learn(img, yolo_obj, PANEL_X, CAMERA_HEIGHT)
                    roi_fail_frames = 0
                    roi_ambiguous_frames = 0
                    suspect_frames = 0
                    reset_reacquire_pending()
                    reset_tx_filter()
                    set_vision_state(VSTATE_TRACK, "switch")
                else:
                    last_yolo_seen_frames += 1
                    roi_fail_frames += 1
                    suspect_frames += 1
                    if (fast_yolo and
                            suspect_frames > yolo_missing_hold_frames() and
                            target_center_finder is not None):
                        target_center_finder.reset()
                    target_obj = held_or_smoothed_prediction()
                    if target_obj is not None:
                        output_valid = True
                        update_lock = False
                        if suspect_frames <= yolo_missing_hold_frames():
                            set_vision_state(VSTATE_TRACK, "yolo hold")
                    else:
                        set_vision_state(VSTATE_SUSPECT, "yolo " + gate_reason)
            else:
                target_switch_gate.miss(yolo_reason)
                roi_fail_frames += 1
                suspect_frames += 1
                if (is_e1_fast_tracking() and
                        suspect_frames > yolo_missing_hold_frames() and
                        target_center_finder is not None):
                    target_center_finder.reset()
                target_obj = held_or_smoothed_prediction()
                if target_obj is not None:
                    output_valid = True
                    update_lock = False
                    if suspect_frames <= yolo_missing_hold_frames():
                        set_vision_state(VSTATE_TRACK, "yolo hold")
                else:
                    set_vision_state(VSTATE_SUSPECT, "yolo " + yolo_reason)

            if target_obj is None and vision_track_state == VSTATE_SUSPECT:
                if is_reaim_running_task():
                    if suspect_frames <= yolo_missing_hold_frames():
                        output_valid = False
                    else:
                        clear_selection(False)
                        reset_tx_filter()
                        set_vision_state(VSTATE_SEARCH, "reaim search")
                        output_valid = False
                elif suspect_frames <= SUSPECT_WAIT_FRAMES:
                    output_valid = False
                elif roi_fail_frames >= ROI_FAIL_TO_LOST_FRAMES:
                    set_vision_state(VSTATE_LOST, "yolo failed")

        if selected_class_id >= 0 and vision_track_state in (VSTATE_LOST, VSTATE_REACQUIRE):
            set_vision_state(VSTATE_REACQUIRE)
            if yolo_fresh:
                reacquired_obj, reacquire_reason = reacquire_with_yolo(track_objs)
            else:
                reacquired_obj, reacquire_reason = None, "missing"
            banana_reacquire_debug = False
            if is_selected_cv_anchor() and not is_e1_fast_tracking():
                special_label = display_label(selected_class_id)
                if reacquired_obj is not None:
                    reacquired_cx, reacquired_cy = obj_center(reacquired_obj)
                    update_last_yolo_box(reacquired_obj)
                    update_last_cv_box(reacquired_obj)
                    dynamic_color_model.learn(img, reacquired_obj, PANEL_X, CAMERA_HEIGHT)
                    kalman_tracker.init(reacquired_cx, reacquired_cy)
                    reset_track_identity_gates()
                    target_obj = banana_yolo_box_for_output(reacquired_obj)
                    output_valid = True
                    update_lock = True
                    snap_output = False
                    roi_fail_frames = 0
                    roi_ambiguous_frames = 0
                    suspect_frames = 0
                    reset_reacquire_pending()
                    reset_tx_filter()
                    set_vision_state(VSTATE_TRACK, special_label + " yolo reacquired")
                    debug_yolo_obj = reacquired_obj
                    debug_yolo_reason = reacquire_reason + "/" + special_label + "/yolo"
                    reacquired_obj = None
                    reacquire_reason = special_label + " yolo"
                else:
                    cv_obj, cv_reason = track_selected_banana_by_cv(img)
                    if cv_obj is not None:
                        cv_cx, cv_cy = obj_center(cv_obj)
                        kalman_tracker.init(cv_cx, cv_cy)
                        reset_track_identity_gates()
                        update_last_cv_box(cv_obj)
                        target_obj = cv_obj
                        output_valid = True
                        update_lock = True
                        snap_output = False
                        roi_fail_frames = 0
                        roi_ambiguous_frames = 0
                        suspect_frames = 0
                        reset_reacquire_pending()
                        reset_tx_filter()
                        set_vision_state(VSTATE_TRACK, special_label + " cv reacquired")
                        reacquire_reason = special_label + " cv"
                    else:
                        age_last_cv_box()
                    debug_yolo_obj = reacquired_obj
                    debug_yolo_reason = reacquire_reason + "/" + special_label + "/" + cv_reason
                banana_reacquire_debug = True
            if not banana_reacquire_debug:
                debug_yolo_obj = reacquired_obj
                debug_yolo_reason = reacquire_reason
            if reacquired_obj is not None:
                reacquired_cx, reacquired_cy = obj_center(reacquired_obj)
                kalman_tracker.init(reacquired_cx, reacquired_cy)
                reset_track_identity_gates()
                target_obj = box_at_center(reacquired_cx, reacquired_cy, selected_w, selected_h, reacquired_obj.class_id, reacquired_obj.score)
                output_valid = True
                update_lock = True
                snap_output = False
                roi_fail_frames = 0
                roi_ambiguous_frames = 0
                suspect_frames = 0
                update_last_yolo_box(reacquired_obj)
                dynamic_color_model.learn(img, reacquired_obj, PANEL_X, CAMERA_HEIGHT)
                reset_reacquire_pending()
                reset_tx_filter()
                set_vision_state(VSTATE_TRACK, "reacquired")
            elif (not banana_reacquire_debug and
                    reacquire_reason not in ("confirming", "missing")):
                set_vision_state(VSTATE_REACQUIRE, reacquire_reason)

        if target_obj is None and selected_class_id >= 0 and vision_track_state == VSTATE_TRACK:
            set_vision_state(VSTATE_SUSPECT, "no target")

        target_objs = []
        control_obj = None
        if target_obj and output_valid:
            lost_frames = 0
            new_cx, new_cy = obj_center(target_obj)
            control_obj = box_at_center(new_cx, new_cy, selected_w, selected_h, target_obj.class_id, target_obj.score)
            if is_plain_running_task():
                snap_smooth_box(control_obj)
                update_anchor_lock(new_cx, new_cy)
            else:
                center_delta = center_delta_from_smooth(new_cx, new_cy)
                if ((center_delta <= OUTPUT_CENTER_DEADBAND_PIXELS or is_small_rect_jitter(control_obj)) and
                        smooth_x >= 0):
                    smooth_score = target_obj.score
                    control_obj = smoothed_target_obj()
                else:
                    if update_lock and last_good_cx >= 0 and last_good_cy >= 0:
                        track_vx = int(track_vx * VELOCITY_ALPHA + (new_cx - last_good_cx) * (1 - VELOCITY_ALPHA))
                        track_vy = int(track_vy * VELOCITY_ALPHA + (new_cy - last_good_cy) * (1 - VELOCITY_ALPHA))
                    if update_lock:
                        update_anchor_lock(new_cx, new_cy)
                    if snap_output:
                        snap_smooth_box(control_obj)
                    else:
                        smooth_box(control_obj)
            smoothed_obj = smoothed_target_obj()
            if smoothed_obj is not None:
                target_objs.append(smoothed_obj)
                update_preaim_if_waiting(smoothed_obj, time.ticks_ms())
        elif selected_class_id >= 0 and vision_track_state == VSTATE_SUSPECT and smooth_x >= 0:
            lost_frames += 1
        elif selected_class_id >= 0:
            lost_frames += 1

        refine_obj = control_obj
        refine_valid = output_valid
        e1_yolo_updated = (
            task_controller is not None and
            task_controller.task_id == TASK_EXT_CIRCLE and
            task_controller.running and
            update_lock)
        e1_yolo_held = (
            task_controller is not None and
            task_controller.task_id == TASK_EXT_CIRCLE and
            task_controller.running and
            output_valid and
            not update_lock and
            suspect_frames <= yolo_missing_hold_frames())
        if (task_controller is not None and
                task_controller.task_id == TASK_EXT_CIRCLE and
                task_controller.running and
                target_center_finder is not None and
                (e1_yolo_updated or e1_yolo_held) and
                (refine_obj is None or not refine_valid)):
            stable_obj = smoothed_target_obj()
            if stable_obj is not None:
                refine_obj = stable_obj
                refine_valid = True

        if (task_controller is not None and
                task_controller.needs_target_center_refinement() and
                refine_obj is not None and refine_valid and
                target_center_finder is not None):
            e1_center_ready = False
            center_input_obj = refine_obj
            circle_only_center = False
            if task_controller.task_id == TASK_EXT_CIRCLE:
                if not (e1_yolo_updated or e1_yolo_held):
                    if target_center_finder is not None:
                        target_center_finder.reset()
                    refine_valid = False
                circle_only_center = True
            if refine_valid:
                center_img = img
                if task_controller.task_id == TASK_EXT_CIRCLE:
                    center_img = mask_laser_for_detector(
                        img,
                        LASER_CIRCLE_MASK_RADIUS,
                        True,
                    )
                target_center_result = target_center_finder.update(
                    center_img,
                    center_input_obj,
                    PANEL_X,
                    CAMERA_HEIGHT,
                    circle_only_center,
                )
            else:
                target_center_result = None
            if (task_controller.task_id == TASK_EXT_CIRCLE and
                    target_center_result is not None and target_center_result.valid):
                normalize_circle_center_result(target_center_result)
            normalize_center_result_for_task(target_center_result)
            if (target_center_result is not None and target_center_result.valid and
                    accept_center_refine(center_input_obj, target_center_result, time.ticks_ms())):
                if task_controller.task_id == TASK_EXT_CIRCLE:
                    e1_center_ready = target_center_result.reason.find("hold") < 0
                    if e1_center_ready:
                        task_controller.runtime.circle_center_cx = target_center_result.cx
                        task_controller.runtime.circle_center_cy = target_center_result.cy
                    if e1_center_ready and target_center_result.width > 0:
                        task_controller.runtime.circle_target_w = target_center_result.width
                    if e1_center_ready and target_center_result.height > 0:
                        task_controller.runtime.circle_target_h = target_center_result.height
                    if e1_center_ready and target_center_result.radius > 0:
                        task_controller.runtime.circle_target_r = target_center_result.radius
                control_obj = box_at_center(target_center_result.cx,
                                            target_center_result.cy,
                                            12,
                                            12,
                                            refine_obj.class_id,
                                            refine_obj.score)
                output_valid = True
                target_objs = [control_obj]
            elif target_center_result is not None:
                task_controller.runtime.status = "CENTER " + target_center_result.reason
                if task_controller.task_id != TASK_EXT_CIRCLE:
                    control_obj = None
                    target_objs = []
                    output_valid = False
        else:
            e1_center_ready = False
            target_center_result = None

        if (task_controller is not None and
                task_controller.task_id == TASK_EXT_CIRCLE and
                task_controller.running and
                vision_track_state != VSTATE_TRACK):
            control_obj = None
            target_objs = []
            output_valid = False

        if report_on:
            if len(target_objs) > 0:
                p.report(APP_CMD_DETECT_RES, encode_objs(target_objs))
            else:
                body = encode_circle_kalman_debug(target_center_result)
                if len(body) > 0:
                    p.report(APP_CMD_DETECT_RES, body)

        task_tx_offset = None
        control_prediction = False
        if control_obj is not None and output_valid:
            control_prediction = bool(target_obj is not None and not update_lock and suspect_frames > 0)
            if (task_controller is not None and task_controller.running and
                    task_controller.task_id == TASK_EXT_CIRCLE):
                task_tx_offset = target_offset_for_circle(control_obj)
            else:
                task_tx_offset = target_offset_for_mcu(control_obj)
        if task_controller is not None:
            if task_controller.update_lock(control_obj, output_valid, task_tx_offset, time.ticks_ms()):
                clear_selection(False)
                reset_tx_filter()
                control_obj = None
                target_objs = []
                output_valid = False
                task_tx_offset = None
                if task_controller.runtime.done:
                    queue_done_signal()
            elif task_controller.runtime.done:
                clear_selection(False)
                reset_tx_filter()
                control_obj = None
                target_objs = []
                output_valid = False
                task_tx_offset = None
                queue_done_signal()
            control_obj = task_controller.override_control_obj(
                control_obj,
                output_valid,
                box_at_center,
                time.ticks_ms(),
                task_tx_offset,
                target_offset_for_circle,
            )
            if control_obj is None and task_controller.runtime.done:
                output_valid = False
                task_tx_offset = None
                queue_done_signal()
            elif control_obj is not None and output_valid:
                control_prediction = bool(target_obj is not None and not update_lock and suspect_frames > 0)
                if (task_controller.running and
                        task_controller.task_id == TASK_EXT_CIRCLE):
                    task_tx_offset = target_offset_for_circle(control_obj)
                else:
                    task_tx_offset = target_offset_for_mcu(control_obj)

        search_request = should_yaw_search_for_target(control_obj)
        circle_mode = (
            task_controller is not None and
            task_controller.running and
            task_controller.task_id == TASK_EXT_CIRCLE and
            task_controller.runtime.circle_start_ms >= 0 and
            control_obj is not None)
        if control_obj is not None:
            send_target(control_obj, False, circle_mode, control_prediction)
        else:
            if send_done_signal():
                pass
            elif (task_controller is not None and
                    task_controller.running and
                    task_controller.task_id == TASK_EXT_CIRCLE and
                    output_valid):
                circle_tx_send_from_prediction(time.ticks_ms())
            else:
                send_target(None, search_request)

        send_task_debug(control_obj, output_valid, task_tx_offset)
        send_debug_status(target_obj, output_valid, debug_yolo_obj, debug_yolo_reason)

        if selected_class_id < 0:
            draw_candidate_markers(img, objs)
        elif target_obj and output_valid:
            draw_smoothed_target(img, selected_class_id)
        elif vision_track_state == VSTATE_SUSPECT and lost_frames <= LOST_HOLD_FRAMES:
            draw_smoothed_target(img, selected_class_id)
        else:
            img.draw_rect(4 * UI_SCALE, 4 * UI_SCALE, 260 * UI_SCALE, 32 * UI_SCALE, color = COLOR_BLACK, thickness = -1)
            img.draw_string(8 * UI_SCALE, 8 * UI_SCALE, vision_state_name(), color = COLOR_LOST, scale = 2 * UI_SCALE)

        draw_target_center_result(img)
        draw_side_panel(img, objs, target_obj, selected_class_id)
        if control_obj is not None:
            draw_uart_overlay(img, control_obj)
        else:
            draw_uart_overlay(img, None)
        dis.show(img, fit = image.Fit.FIT_CONTAIN)


if __name__ == "__main__":
    run()
