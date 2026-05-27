from maix import app, camera, display, image, time, video
import os
import sys


# MaixCam2 dataset recorder for jixun2 YOLO capture.
# Keep the recorded frames clean: no boxes, crosses, or UI overlays are encoded.

OUTPUT_DIR = "/root/videos"
DURATION_SEC = 120
WIDTH = 1280
HEIGHT = 720
WARMUP_FRAMES = 5
PRINT_EVERY_FRAMES = 30
ENABLE_PREVIEW = True
PREVIEW_SECONDS = 5
PREVIEW_EVERY_FRAMES = 3
END_NOTICE_SECONDS = 4
PREFIX = "jixun2_yolo"
DIGITAL_ZOOM_SCALE = 1.60
DIGITAL_ZOOM_CENTER_X = 0.50
DIGITAL_ZOOM_CENTER_Y = 0.50
CAPTURE_FORMAT = "auto"
PREVIEW_ONLY = False


def parse_bool(value):
    return value.lower() not in ("0", "false", "no", "off")


def clamp(value, low, high):
    return max(low, min(high, value))


def parse_args():
    cfg = {
        "duration": DURATION_SEC,
        "width": WIDTH,
        "height": HEIGHT,
        "out_dir": OUTPUT_DIR,
        "preview": ENABLE_PREVIEW,
        "preview_seconds": PREVIEW_SECONDS,
        "prefix": PREFIX,
        "zoom": DIGITAL_ZOOM_SCALE,
        "zoom_x": DIGITAL_ZOOM_CENTER_X,
        "zoom_y": DIGITAL_ZOOM_CENTER_Y,
        "capture_format": CAPTURE_FORMAT,
        "preview_only": PREVIEW_ONLY,
    }
    for arg in sys.argv[1:]:
        if arg in ("--help", "-h"):
            print("Usage:")
            print("  python maixcam2_record_jixun2.py [--duration=120] [--width=1280] [--height=720]")
            print("                                      [--out-dir=/root/videos] [--preview=1]")
            print("                                      [--preview-seconds=5] [--prefix=jixun2_yolo]")
            print("                                      [--zoom=1.60] [--zoom-x=0.50] [--zoom-y=0.50]")
            print("                                      [--capture-format=auto|rgb|yvu]")
            print("                                      [--preview-only=0|1]")
            print("")
            print("Notes:")
            print("  duration=0 records until app exit / KEY_OK.")
            print("  1280x720 is the validated MaixCam2 capture size for this workflow.")
            print("  zoom crops around normalized anchor (zoom-x, zoom-y), then resizes back.")
            print("  YVU420SP cannot crop on this device; zoom capture uses RGB888.")
            app.set_exit_flag(True)
            return None
        if arg.startswith("--duration="):
            cfg["duration"] = int(arg.split("=", 1)[1])
        elif arg.startswith("--width="):
            cfg["width"] = int(arg.split("=", 1)[1])
        elif arg.startswith("--height="):
            cfg["height"] = int(arg.split("=", 1)[1])
        elif arg.startswith("--out-dir="):
            cfg["out_dir"] = arg.split("=", 1)[1]
        elif arg.startswith("--preview="):
            cfg["preview"] = parse_bool(arg.split("=", 1)[1])
        elif arg.startswith("--preview-seconds="):
            cfg["preview_seconds"] = int(arg.split("=", 1)[1])
        elif arg.startswith("--prefix="):
            cfg["prefix"] = arg.split("=", 1)[1]
        elif arg.startswith("--zoom="):
            cfg["zoom"] = float(arg.split("=", 1)[1])
        elif arg.startswith("--zoom-x="):
            cfg["zoom_x"] = float(arg.split("=", 1)[1])
        elif arg.startswith("--zoom-y="):
            cfg["zoom_y"] = float(arg.split("=", 1)[1])
        elif arg.startswith("--capture-format="):
            cfg["capture_format"] = arg.split("=", 1)[1].lower()
        elif arg.startswith("--preview-only="):
            cfg["preview_only"] = parse_bool(arg.split("=", 1)[1])
    return cfg


def make_output_path(out_dir, prefix, width, height, zoom):
    os.makedirs(out_dir, exist_ok=True)
    stamp = time.ticks_ms()
    zoom_tag = "z%03d" % int(zoom * 100 + 0.5)
    name = "%s_%dx%d_%s_%d.h265" % (prefix, width, height, zoom_tag, stamp)
    return os.path.join(out_dir, name)


def show_end_notice(output_path, frames, bytes_written):
    print("")
    print("========================================")
    print("RECORDING FINISHED")
    print("Frames:", frames)
    print("Bytes:", bytes_written)
    print("Output:", output_path)
    print("========================================")
    try:
        disp = display.Display()
        img = image.Image(480, 640, image.Format.FMT_RGB888)
        img.draw_rect(0, 0, 480, 640, image.Color.from_rgb(0, 0, 0), thickness=-1)
        img.draw_string(38, 180, "RECORDING", image.Color.from_rgb(0, 255, 0), scale=3)
        img.draw_string(38, 240, "FINISHED", image.Color.from_rgb(0, 255, 0), scale=3)
        img.draw_string(38, 320, "frames: %d" % frames, image.Color.from_rgb(255, 255, 255), scale=2)
        img.draw_string(38, 360, "check /root/videos", image.Color.from_rgb(255, 255, 255), scale=2)
        disp.show(img)
        time.sleep(END_NOTICE_SECONDS)
    except Exception as err:
        print("End notice display failed:", err)
        time.sleep(END_NOTICE_SECONDS)


def apply_digital_zoom(img, scale, anchor_x, anchor_y):
    if scale <= 1.01:
        return img

    src_w = img.width()
    src_h = img.height()
    crop_w = clamp(int(src_w / scale), 32, src_w)
    crop_h = clamp(int(src_h / scale), 32, src_h)
    center_x = int(src_w * clamp(anchor_x, 0.0, 1.0))
    center_y = int(src_h * clamp(anchor_y, 0.0, 1.0))
    crop_x = clamp(center_x - crop_w // 2, 0, src_w - crop_w)
    crop_y = clamp(center_y - crop_h // 2, 0, src_h - crop_h)
    return img.crop(crop_x, crop_y, crop_w, crop_h).resize(src_w, src_h)


def resolve_capture_format(zoom, capture_format):
    if capture_format == "rgb":
        return image.Format.FMT_RGB888, "RGB888"
    if capture_format == "yvu":
        return image.Format.FMT_YVU420SP, "YVU420SP"
    if zoom > 1.01:
        return image.Format.FMT_RGB888, "RGB888"
    return image.Format.FMT_YVU420SP, "YVU420SP"


def run_preview(cam, seconds, zoom, zoom_x, zoom_y):
    if seconds <= 0:
        return
    print("Preview for %d seconds, then recording starts." % seconds)
    disp = display.Display()
    start = time.ticks_ms()
    frames = 0
    while not app.need_exit() and time.ticks_ms() - start < seconds * 1000:
        img = cam.read()
        if frames % PREVIEW_EVERY_FRAMES == 0:
            disp.show(apply_digital_zoom(img, zoom, zoom_x, zoom_y))
        frames += 1
    cam.skip_frames(3)
    cam.clear_buff()
    print("Recording starts now.")


def main():
    cfg = parse_args()
    if cfg is None:
        return

    output_path = make_output_path(cfg["out_dir"], cfg["prefix"], cfg["width"], cfg["height"], cfg["zoom"])

    print("Output:", output_path)
    print("Resolution: %dx%d" % (cfg["width"], cfg["height"]))
    print("Duration:", cfg["duration"], "sec")
    print("Codec: default video.Encoder output")
    print("Preview:", cfg["preview"])
    print("Preview only:", cfg["preview_only"])
    print("Digital zoom: %.2fx anchor=(%.2f, %.2f)" % (cfg["zoom"], cfg["zoom_x"], cfg["zoom_y"]))
    print("Clean frames: no overlays encoded")

    capture_fmt, capture_fmt_name = resolve_capture_format(cfg["zoom"], cfg["capture_format"])
    print("Capture format:", capture_fmt_name)
    if cfg["zoom"] > 1.01 and capture_fmt_name == "YVU420SP":
        print("ERROR: YVU420SP does not support crop/resize on this device. Use --capture-format=rgb or --zoom=1.0")
        app.set_exit_flag(True)
        return

    cam = camera.Camera(cfg["width"], cfg["height"], capture_fmt)
    cam.skip_frames(WARMUP_FRAMES)
    cam.clear_buff()

    if cfg["preview"]:
        run_preview(cam, cfg["preview_seconds"], cfg["zoom"], cfg["zoom_x"], cfg["zoom_y"])
        if cfg["preview_only"]:
            cam.close()
            app.set_exit_flag(True)
            return

    enc = video.Encoder(width=cam.width(), height=cam.height())
    f = open(output_path, "wb")

    start_ms = time.ticks_ms()
    duration_ms = cfg["duration"] * 1000
    frames = 0
    bytes_written = 0

    try:
        while not app.need_exit():
            img = cam.read()
            record_img = apply_digital_zoom(img, cfg["zoom"], cfg["zoom_x"], cfg["zoom_y"])
            frame = enc.encode(record_img)
            if frame:
                data = frame.to_bytes()
                f.write(data)
                bytes_written += len(data)
                frames += 1

            if frames and frames % PRINT_EVERY_FRAMES == 0:
                elapsed = max(1, time.ticks_ms() - start_ms)
                fps_now = frames * 1000.0 / elapsed
                size_mb = bytes_written / 1048576.0
                print("frames=%d fps=%.1f size=%.1fMB" % (frames, fps_now, size_mb))

            if duration_ms > 0 and time.ticks_ms() - start_ms >= duration_ms:
                break
    finally:
        f.flush()
        f.close()
        cam.close()
        print("Done.")
        print("Frames:", frames)
        print("Bytes:", bytes_written)
        print("Output:", output_path)
        print("Copy this file to:")
        print("  C:\\Users\\Administrator\\Desktop\\Yolo\\projects\\jixun2\\incoming")
        print("Convert on PC if needed:")
        print("  ffmpeg -framerate 50 -i input.h265 -c:v copy output.mp4")
        show_end_notice(output_path, frames, bytes_written)
        app.set_exit_flag(True)


if __name__ == "__main__":
    main()
