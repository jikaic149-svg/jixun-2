from core.filters import limit_step


class UartTargetProtocol:
    def __init__(self, serial=None, period_ms=50):
        self.serial = serial
        self.period_ms = period_ms
        self.safe_dx = 0
        self.safe_dy = 0

    def write(self, text):
        if self.serial is not None:
            self.serial.write(text.encode("ascii"))

    def send_target(self, dx, dy, valid=True):
        if not valid:
            self.safe_dx = limit_step(self.safe_dx, 0, 20)
            self.safe_dy = limit_step(self.safe_dy, 0, 16)
            self.write("%d,%d,0\n" % (self.safe_dx, self.safe_dy))
            return
        self.safe_dx = limit_step(self.safe_dx, dx, 20)
        self.safe_dy = limit_step(self.safe_dy, dy, 16)
        self.write("%d,%d,1\n" % (self.safe_dx, self.safe_dy))
