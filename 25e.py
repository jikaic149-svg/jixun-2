import time as pytime

from maix import app, camera, display, image, time, uart

try:
    from maix import pinmap
except Exception:
    pinmap = None


CAM_W = 320
CAM_H = 240
UART_DEV = "/dev/ttyS2"
UART_BAUD = 115200
TARGET_MIN_AREA = 250
TARGET_SEND_PERIOD_MS = 50
STAT_LOG_PERIOD_MS = 1000
STATE_NAMES = ("RELAX", "ENC_LOCK", "CALIB", "IMU_STAB", "GRAV_CAL", "IDLE", "ANGLE_TEST", "TORQUE_TEST", "VISION_STAB")


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
    "pitch_target": 0.0,
    "pitch_err": 0.0,
    "yaw_target": 0.0,
    "yaw_err": 0.0,
    "angle_phase": 0,
    "torque_phase": 0,
    "torque_ramp": 0.0,
    "torque_move": 0.0,
    "pitch_pos": 0.0,
    "pitch_neg": 0.0,
    "yaw_pos": 0.0,
    "yaw_neg": 0.0,
    "torque_done": 0,
    "grav_target": 0.0,
    "grav_err": 0.0,
    "grav_enc": 0.0,
    "grav_current": 0.0,
    "grav_point": 0,
    "grav_done": 0,
    "rx": 0,
}

boot_ms = time.ticks_ms()
line_buf = bytearray()
serial = None
last_target_tx_ms = 0
last_stat_log_ms = 0


def ticks():
    return int(time.ticks_ms() - boot_ms)


def log(msg):
    print("[T:%06d] %s" % (ticks(), msg))


def state_name():
    state = stat["state"]
    if 0 <= state < len(STATE_NAMES):
        return STATE_NAMES[state]
    return "UNKNOWN"


def init_uart():
    global serial
    if pinmap:
        try:
            pinmap.set_pin_function("B0", "UART2_TX")
            pinmap.set_pin_function("B1", "UART2_RX")
        except Exception:
            pass
    serial = uart.UART(UART_DEV, UART_BAUD)
    log("uart %s %d" % (UART_DEV, UART_BAUD))


def uart_write(text):
    if serial is not None:
        serial.write(text.encode("ascii"))


def parse_line(line):
    global last_stat_log_ms

    idx = line.find("M,STAT,")
    if idx > 0:
        line = line[idx:]
    if line == "U6BOOT" or line == "PONG":
        return

    parts = line.split(",")
    if len(parts) >= 10 and parts[0] == "M" and parts[1] == "STAT":
        try:
            stat["time_ms"] = int(parts[2])
            stat["pitch_abs"] = float(parts[3])
            stat["yaw_abs"] = float(parts[4])
            stat["pitch_enc"] = float(parts[5])
            stat["pitch_current"] = float(parts[6])
            stat["yaw_current"] = float(parts[7])
            stat["dx"] = int(parts[8])
            stat["dy"] = int(parts[9])
            if len(parts) >= 12:
                stat["state"] = int(parts[10])
                stat["key"] = int(parts[11])
            if len(parts) >= 14:
                stat["key_raw"] = int(parts[12])
                stat["key_hold_ms"] = int(parts[13])
            if len(parts) >= 19:
                stat["pitch_target"] = float(parts[14])
                stat["pitch_err"] = float(parts[15])
                stat["yaw_target"] = float(parts[16])
                stat["yaw_err"] = float(parts[17])
                stat["angle_phase"] = int(parts[18])
            if len(parts) >= 27:
                stat["torque_phase"] = int(parts[19])
                stat["torque_ramp"] = float(parts[20])
                stat["torque_move"] = float(parts[21])
                stat["pitch_pos"] = float(parts[22])
                stat["pitch_neg"] = float(parts[23])
                stat["yaw_pos"] = float(parts[24])
                stat["yaw_neg"] = float(parts[25])
                stat["torque_done"] = int(parts[26])
            if len(parts) >= 33:
                stat["grav_target"] = float(parts[27])
                stat["grav_err"] = float(parts[28])
                stat["grav_enc"] = float(parts[29])
                stat["grav_current"] = float(parts[30])
                stat["grav_point"] = int(parts[31])
                stat["grav_done"] = int(parts[32])
        except Exception as e:
            log("bad stat: %s line=%s" % (e, line))
            return

        now = time.ticks_ms()
        if now - last_stat_log_ms >= STAT_LOG_PERIOD_MS:
            last_stat_log_ms = now
            log("t=%lu state=%s(%d) key=%d raw=%d hold=%lu pitch=%.2f enc=%.2f yaw=%.2f ip=%.2f iy=%.2f dx=%d dy=%d pt=%.2f pe=%.2f yt=%.2f ye=%.2f ph=%d tq=%d ramp=%.3f mv=%.2f pp=%.3f pn=%.3f yp=%.3f yn=%.3f done=%u gt=%.2f ge=%.2f genc=%.2f gcur=%.3f gp=%u gd=%u" %
                (stat["time_ms"], state_name(), stat["state"],
                 stat["key"], stat["key_raw"], stat["key_hold_ms"],
                 stat["pitch_abs"], stat["pitch_enc"],
                 stat["yaw_abs"], stat["pitch_current"], stat["yaw_current"],
                 stat["dx"], stat["dy"],
                 stat["pitch_target"], stat["pitch_err"],
                 stat["yaw_target"], stat["yaw_err"],
                 stat["angle_phase"],
                 stat["torque_phase"], stat["torque_ramp"], stat["torque_move"],
                 stat["pitch_pos"], stat["pitch_neg"],
                 stat["yaw_pos"], stat["yaw_neg"], stat["torque_done"],
                 stat["grav_target"], stat["grav_err"],
                 stat["grav_enc"], stat["grav_current"],
                 stat["grav_point"], stat["grav_done"]))
    return


def poll_uart():
    if serial is None:
        return
    data = serial.read()
    if not data:
        return
    raw = bytes(data)
    stat["rx"] += len(raw)
    for b in raw:
        if b == 10:
            line = bytes(line_buf).decode("ascii", "ignore").strip()
            line_buf.clear()
            parse_line(line)
        elif b != 13:
            if len(line_buf) < 360:
                line_buf.append(b)
            else:
                line_buf.clear()


def find_target(img):
    work = img.copy()
    try:
        work = work.to_format(image.Format.FMT_GRAYSCALE)
    except Exception:
        try:
            work = work.to_grayscale()
        except Exception:
            pass

    work.binary([[0, 115]])
    blobs = work.find_blobs([[200, 255]],
                            pixels_threshold=TARGET_MIN_AREA,
                            area_threshold=TARGET_MIN_AREA,
                            merge=True)
    if not blobs:
        return None

    blob = max(blobs, key=lambda b: b.pixels())
    x, y, w, h = blob.x(), blob.y(), blob.w(), blob.h()
    cx = x + w // 2
    cy = y + h // 2
    return {
        "x": x,
        "y": y,
        "w": w,
        "h": h,
        "cx": cx,
        "cy": cy,
        "dx": cx - CAM_W // 2,
        "dy": cy - CAM_H // 2,
    }


def send_target(target):
    global last_target_tx_ms
    now = time.ticks_ms()
    if now - last_target_tx_ms < TARGET_SEND_PERIOD_MS:
        return
    last_target_tx_ms = now
    if target:
        uart_write("%d,%d\n" % (target["dx"], target["dy"]))


def draw_overlay(img, target, fps):
    if target:
        img.draw_rect(target["x"], target["y"], target["w"], target["h"],
                      color=image.COLOR_WHITE, thickness=2)
        img.draw_cross(target["cx"], target["cy"],
                       color=image.COLOR_WHITE, size=12, thickness=1)

    img.draw_cross(CAM_W // 2, CAM_H // 2,
                   color=image.COLOR_BLACK, size=10, thickness=1)

    lines = [
        "FPS:%d RX:%d" % (int(fps), stat["rx"]),
        "t:%lu S:%s K:%d R:%d" % (stat["time_ms"], state_name(), stat["key"], stat["key_raw"]),
        "hold:%lums" % stat["key_hold_ms"],
        "P:%7.2f E:%7.2f" % (stat["pitch_abs"], stat["pitch_enc"]),
        "Y:%7.2f" % stat["yaw_abs"],
        "Ip:%6.2f Iy:%6.2f" % (stat["pitch_current"], stat["yaw_current"]),
        "dx:%+4d dy:%+4d" % (stat["dx"], stat["dy"]),
    ]
    y = 2
    for line in lines:
        img.draw_string(2, y, line, color=image.COLOR_WHITE)
        y += 14


def main():
    init_uart()
    cam = camera.Camera(CAM_W, CAM_H)
    disp = display.Display()

    while not app.need_exit():
        try:
            img = cam.read()
        except Exception as e:
            log("camera: %s" % e)
            pytime.sleep(0.05)
            continue
        if img is None:
            continue

        poll_uart()
        target = find_target(img)
        send_target(target)

        fps = time.fps()
        draw_overlay(img, target, fps)
        disp.show(img)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        log("fatal: %s" % e)
        import traceback
        traceback.print_exc()
