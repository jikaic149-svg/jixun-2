def limit_step(current, target, max_step):
    delta = target - current
    if delta > max_step:
        return current + max_step
    if delta < -max_step:
        return current - max_step
    return target


class LowPass:
    def __init__(self, alpha, initial=0.0):
        self.alpha = alpha
        self.value = initial
        self.initialized = False

    def update(self, value):
        if not self.initialized:
            self.value = value
            self.initialized = True
        else:
            self.value = self.alpha * self.value + (1.0 - self.alpha) * value
        return self.value
