class Renderer:
    def draw_crosses(self, img, aim_point, target_point, colors):
        ax, ay = aim_point
        img.draw_cross(ax, ay, color=colors["aim"], size=18, thickness=2)
        if target_point is not None:
            tx, ty = target_point
            img.draw_cross(tx, ty, color=colors["target"], size=24, thickness=3)
