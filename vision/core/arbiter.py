class TrackArbiter:
    def choose(self, traditional, yolo, prediction):
        if traditional is not None and traditional.valid:
            return traditional
        if yolo is not None and yolo.valid:
            return yolo
        if prediction is not None and prediction.valid:
            return prediction
        return None
