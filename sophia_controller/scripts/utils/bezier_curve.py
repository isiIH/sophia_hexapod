class BezierCurve():

    def __init__(self, init_pos, final_pos, height):
        self.p0 = init_pos
        self.p1 = init_pos + [0, 0, height]
        self.p2 = final_pos + [0, 0, height]
        self.p3 = final_pos

    def get_point(self, t):
        b0 = (1 - t)**3
        b1 = 3 * t * (1 - t)**2
        b2 = 3 * (t**2) * (1 - t)
        b3 = t**3

        return b0 * self.p0 + b1 * self.p1 + b2 * self.p2 + b3 * self.p3