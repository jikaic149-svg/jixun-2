SEARCH = "SEARCH"
LOCK_INIT = "LOCK_INIT"
TRACK_FAST = "TRACK_FAST"
VERIFY = "VERIFY"
RECOVER = "RECOVER"
LOST = "LOST"


class VisionStateMachine:
    def __init__(self):
        self.state = SEARCH

    def set(self, state):
        self.state = state

    def is_locked(self):
        return self.state in (TRACK_FAST, VERIFY, RECOVER)
