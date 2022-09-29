from Point import Point

class Waypoint(Point):
    def __init__(self, x: float=0.0, y: float=0.0, z: float=0.0, yaw: float=0.0, radius: float=0) -> None:
        super().__init__(x=x, y=y, z=z)
        self.exit_yaw = yaw
        self.radius = radius

    def __str__(self) -> str:
        return super().__str__()

    def point_within_radius(self, point: Point) -> bool:
        return self.distance_to(point) < self.radius