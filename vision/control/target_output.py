from core.filters import limit_step


class TargetOutputLimiter:
    def __init__(self, max_step_x=20, max_step_y=16, max_abs_x=160, max_abs_y=130):
        self.max_step_x = max_step_x
        self.max_step_y = max_step_y
        self.max_abs_x = max_abs_x
        self.max_abs_y = max_abs_y
        self.dx = 0
        self.dy = 0

    def update(self, aim_error):
        target_dx = max(-self.max_abs_x, min(self.max_abs_x, aim_error.dx))
        target_dy = max(-self.max_abs_y, min(self.max_abs_y, aim_error.dy))
        self.dx = limit_step(self.dx, target_dx, self.max_step_x)
        self.dy = limit_step(self.dy, target_dy, self.max_step_y)
        return self.dx, self.dy, aim_error.valid
