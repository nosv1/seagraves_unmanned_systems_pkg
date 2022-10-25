class DetectedObject:
    def __init__(self, distance: float, angle: float):
        self.distance = distance
        self.angle = angle  # (-180, 180) degrees

    def __str__(self):
        return f"Range: {self.distance}, Angle: {self.angle}"