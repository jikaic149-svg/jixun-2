import os

from config import VisionConfig


class YoloMaixDetector:
    def __init__(self, nn_module, config=VisionConfig):
        self.nn = nn_module
        self.config = config
        self.model_path = self.resolve_model_path()
        self.detector = self.create_detector(self.model_path)
        self.validate_labels()

    def resolve_model_path(self):
        candidates = [
            self.config.model_file,
            "./" + self.config.model_file,
            "/root/models/%s/%s" % (self.config.model_name, self.config.model_file),
            "/root/models/jixun2/%s" % self.config.model_file,
            "jixun2.mud",
            "./jixun2.mud",
            "/root/models/jixun2/jixun2.mud",
            "/root/models/maixhub/277595/jixun2.mud",
        ]
        for path in candidates:
            if os.path.exists(path):
                return path
        raise RuntimeError("model not found: %s" % self.config.model_file)

    def read_model_type(self, path):
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

    def create_detector(self, path):
        if self.read_model_type(path) == "yolo11":
            return self.nn.YOLO11(model=path, dual_buff=True)
        return self.nn.YOLOv5(model=path)

    def validate_labels(self):
        labels = tuple(self.detector.labels)
        if labels != self.config.expected_labels:
            raise RuntimeError("label mismatch: %s" % ",".join(labels))

    def detect(self, frame):
        return self.detector.detect(frame, conf_th=self.config.conf_th, iou_th=self.config.iou_th)
