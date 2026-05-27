def clamp(value, min_value, max_value):
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value


def center(obj):
    return obj.x + obj.w // 2, obj.y + obj.h // 2


def dist2(a_x, a_y, b_x, b_y):
    dx = a_x - b_x
    dy = a_y - b_y
    return dx * dx + dy * dy


def expand_box(x, y, w, h, margin, max_w, max_h):
    x1 = clamp(x - margin, 0, max_w - 1)
    y1 = clamp(y - margin, 0, max_h - 1)
    x2 = clamp(x + w + margin, 0, max_w)
    y2 = clamp(y + h + margin, 0, max_h)
    return x1, y1, x2 - x1, y2 - y1
