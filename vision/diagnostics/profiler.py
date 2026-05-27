class Profiler:
    def __init__(self):
        self.counters = {}

    def mark(self, name):
        self.counters[name] = self.counters.get(name, 0) + 1
