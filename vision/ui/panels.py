class CandidatePanel:
    def __init__(self, x, width):
        self.x = x
        self.width = width

    def contains(self, px):
        return px >= self.x
