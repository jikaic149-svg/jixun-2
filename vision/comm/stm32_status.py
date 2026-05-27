from core.types import GimbalState


def parse_status_line(line):
    state = GimbalState()
    idx = line.find("M,STAT,")
    if idx > 0:
        line = line[idx:]
    if not line.startswith("M,STAT,"):
        return state
    parts = line.split(",")
    try:
        state.time_ms = int(parts[2])
        state.state = int(parts[3])
        state.dx = int(parts[4])
        state.dy = int(parts[5])
        state.valid = True
        if len(parts) > 10:
            state.pitch_abs = float(parts[10])
            state.yaw_abs = float(parts[12])
    except Exception:
        state.valid = False
    return state
