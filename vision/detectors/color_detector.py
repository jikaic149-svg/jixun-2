class ColorDetector:
    def detect(self, frame, roi=None, thresholds=None):
        if thresholds is None:
            return []
        return frame.find_blobs(thresholds, roi=roi, merge=True)
