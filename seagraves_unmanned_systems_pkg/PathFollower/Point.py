from __future__ import annotations

from math import atan2, pi

class Point:
    def __init__(self, x: float, y: float, z: float) -> None:
        self.x: float = x
        self.y: float = y
        self.z: float = z

    def __str__(self) -> str:
        return f"[{self.x}, {self.y}, {self.z}]"

    def __eq__(self, other: Point) -> bool:
        return (
            self.x == other.x and 
            self.y == other.y and 
            self.z == other.z
        )

    def distance_to(self, other: Point) -> float:
        dx: float = self.x - other.x
        dy: float = self.y - other.y
        dz: float = self.z - other.z
        return (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5

    def heading_to(self, other: Point) -> float:
        dx: float = other.x - self.x
        dy: float = other.y - self.y
        heading: float = atan2(dy, dx)
        return heading if heading >= 0 else heading + 2 * pi