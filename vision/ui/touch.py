class TouchMapper:
    def __init__(self, image_module, display):
        self.image = image_module
        self.display = display

    def to_image_pos(self, img, x, y):
        mapped = self.image.resize_map_pos_reverse(
            img.width(), img.height(), self.display.width(), self.display.height(),
            self.image.Fit.FIT_CONTAIN, x, y)
        return int(mapped[0]), int(mapped[1])
