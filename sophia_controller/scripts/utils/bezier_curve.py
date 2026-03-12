class BezierCurve():

    def __init__(self, init_point, final_point, height):
        self.height = [0, 0, height]
        self.p0 = init_point
        self.p1 = init_point + self.height
        self.p2 = final_point + self.height
        self.p3 = final_point

    def update(self, init_point, final_point):
        self.p0 = init_point
        self.p1 = init_point + self.height
        self.p2 = final_point + self.height
        self.p3 = final_point

    def get_point(self, t):
        b0 = (1 - t)**3
        b1 = 3 * t * (1 - t)**2
        b2 = 3 * (t**2) * (1 - t)
        b3 = t**3

        return b0 * self.p0 + b1 * self.p1 + b2 * self.p2 + b3 * self.p3