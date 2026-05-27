from core.geometry import clamp


class DigitalZoom:
    def __init__(self, scale=1.0, center_x=0.5, center_y=0.5):
        self.scale = scale
        self.center_x = center_x
        self.center_y = center_y

    def apply(self, img):
        if self.scale <= 1.01:
            return img
        src_w = img.width()
        src_h = img.height()
        crop_w = clamp(int(src_w / self.scale), 32, src_w)
        crop_h = clamp(int(src_h / self.scale), 32, src_h)
        cx = int(src_w * clamp(self.center_x, 0.0, 1.0))
        cy = int(src_h * clamp(self.center_y, 0.0, 1.0))
        x = clamp(cx - crop_w // 2, 0, src_w - crop_w)
        y = clamp(cy - crop_h // 2, 0, src_h - crop_h)
        return img.crop(x, y, crop_w, crop_h).resize(src_w, src_h)
