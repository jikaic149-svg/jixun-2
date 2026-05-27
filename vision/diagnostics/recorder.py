class Recorder:
    def __init__(self, enabled=False):
        self.enabled = enabled

    def record(self, item):
        if self.enabled:
            print(item)
