class Detector:
    def detect(self, frame):
        raise NotImplementedError


class Tracker:
    def update(self, frame, detections, gimbal_state):
        raise NotImplementedError


class Protocol:
    def poll(self):
        raise NotImplementedError

    def send_target(self, aim_error):
        raise NotImplementedError


class Renderer:
    def draw(self, frame, context):
        raise NotImplementedError
