from math import atan2, cos, hypot, sin
from dataclasses import dataclass

@dataclass
class Point:
    x: float
    y: float

    def __add__(self, other: "Point") -> "Point":
        return Point(self.x + other.x, self.y + other.y)
    
    def __mul__(self, scalar: float) -> "Point":
        return Point(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar: float) -> "Point":
        return Point(self.x * scalar, self.y * scalar)

    def __sub__(self, other: "Point") -> "Point":
        return Point(self.x - other.x, self.y - other.y)

    def rotate(self, angle: float) -> "Point":
        return Point(self.x * cos(angle) - self.y * sin(angle),
                     self.x * sin(angle) + self.y * cos(angle))

    def angle(self) -> float:
        return atan2(self.y, self.x)
    
    def norm(self):
        return hypot(self.x, self.y)

