from core.arbiter import TrackArbiter


class HybridTracker:
    def __init__(self, traditional_tracker=None, motion_predictor=None, arbiter=None):
        self.traditional_tracker = traditional_tracker
        self.motion_predictor = motion_predictor
        self.arbiter = arbiter or TrackArbiter()

    def update(self, traditional_result, yolo_result, prediction_result):
        return self.arbiter.choose(traditional_result, yolo_result, prediction_result)
