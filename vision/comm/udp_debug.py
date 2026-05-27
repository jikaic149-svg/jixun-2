try:
    import socket
except Exception:
    socket = None


class UdpDebugSender:
    def __init__(self, enabled, host, port):
        self.enabled = bool(enabled)
        self.host = host
        self.port = int(port)
        self.sock = None
        self.fail_count = 0

    def init(self):
        if not self.enabled or socket is None:
            return False
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            except Exception:
                pass
            return True
        except Exception:
            self.sock = None
            return False

    def send(self, text):
        if not self.enabled:
            return False
        if self.sock is None and not self.init():
            return False
        try:
            if not text.endswith("\n"):
                text += "\n"
            self.sock.sendto(text.encode("ascii", "ignore"), (self.host, self.port))
            return True
        except Exception:
            self.fail_count += 1
            return False
