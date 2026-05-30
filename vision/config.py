class VisionConfig:
    model_name = "jixun2_final_manual25_banana_laser_20260529"
    model_file = "jixun2_final_manual25_banana_laser_20260529.mud"
    expected_labels = (
        "apple",
        "banana",
        "strawberry",
        "watermelon",
        "target",
        "green_square",
        "red_triangle",
        "yellow_trapezoid",
    )

    camera_width = 960
    camera_height = 720
    digital_zoom_scale = 1.60
    conf_th = 0.25
    iou_th = 0.45

    uart_dev = "/dev/ttyS2"
    uart_baud = 115200
    target_send_period_ms = 50
