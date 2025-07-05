from dataclasses import dataclass
from .path import StraightPart
from .point import Point
import matplotlib.pyplot as plt


@dataclass
class ParkingSpace:
    start: Point
    width: float
    length: float
    baseline_left: float = 0.0
    baseline_right: float = 0.0
    road_width: float = 8.0

    def segments(self) -> list[StraightPart]:
        p0 = Point(self.start.x - self.baseline_left, self.start.y)
        p1 = self.start
        p2 = Point(p1.x, p1.y - self.width)
        p3 = Point(p2.x + self.length, p2.y)
        p4 = Point(p3.x, p3.y + self.width)
        p5 = Point(p4.x + self.baseline_right, p4.y)

        top_left = Point(self.start.x - self.baseline_left, self.start.y + self.road_width)
        top_right = Point(self.start.x + self.baseline_right + self.length, self.start.y + self.road_width)

        return [StraightPart(p0, p1), StraightPart(p1, p2),
                StraightPart(p2, p3), StraightPart(p3, p4),
                StraightPart(p4, p5), StraightPart(top_left, top_right)]

    def draw(self, ax: plt.Axes, *, color="black", lw=2):
        for seg in self.segments():
            seg.draw(ax, color=color, lw=lw, label="_nolegend_")

