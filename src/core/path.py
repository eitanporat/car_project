from dataclasses import dataclass, field
from .point import Point
from ..utils.plot_utils import get_label
import matplotlib.pyplot as plt
import numpy as np
from math import cos, sin, sqrt
from typing import Optional, List, Tuple


class PathPart:
    """Abstract base: any drawable segment of the rear-axle path."""

    def start_point(self) -> Point: ...
    def end_point(self) -> Point: ...
    def draw(self, ax: plt.Axes, **kwargs): ...
    def intersects(self, other: "PathPart") -> bool: ...
    def sample_points(self, spacing: float = 0.1) -> list[Point]:
        """Return a list of points sampled along the path part at fixed arc-length intervals."""
        raise NotImplementedError


@dataclass
class StraightPart(PathPart):
    start: Point
    end: Point

    def start_point(self) -> Point:
        return self.start

    def end_point(self) -> Point:
        return self.end

    def draw(self, ax: plt.Axes, **kwargs):
        ax.plot([self.start.x, self.end.x],
                [self.start.y, self.end.y],
                **kwargs)
    
    def intersects(self, other: "PathPart") -> bool:
        if isinstance(other, StraightPart):
            return self._intersects_straight(other)
        elif isinstance(other, ArcPart):
            return other._intersects_straight(self)
        return False
    
    def _intersects_straight(self, other: "StraightPart") -> bool:
        """Check if two line segments intersect using parametric equations."""
        x1, y1 = self.start.x, self.start.y
        x2, y2 = self.end.x, self.end.y
        x3, y3 = other.start.x, other.start.y
        x4, y4 = other.end.x, other.end.y
        
        # Calculate direction vectors
        dx1, dy1 = x2 - x1, y2 - y1
        dx2, dy2 = x4 - x3, y4 - y3
        
        # Calculate denominator for parametric equations
        denom = dx1 * dy2 - dy1 * dx2
        
        # Lines are parallel if denominator is zero
        if abs(denom) < 1e-10:
            return False
        
        # Calculate parameters
        t = ((x3 - x1) * dy2 - (y3 - y1) * dx2) / denom
        u = ((x3 - x1) * dy1 - (y3 - y1) * dx1) / denom
        
        # Check if intersection occurs within both line segments
        return 0 <= t <= 1 and 0 <= u <= 1

    def sample_points(self, spacing: float = 0.1) -> list[Point]:
        points = []
        dx, dy = self.end.x - self.start.x, self.end.y - self.start.y
        length = np.hypot(dx, dy)
        n = max(1, int(np.ceil(length / spacing)))
        for i in range(n):
            t = i / n
            x = self.start.x + t * dx
            y = self.start.y + t * dy
            points.append(Point(x, y))
        points.append(self.end)
        return points


@dataclass
class ArcPart(PathPart):
    centre: Point
    radius: float
    start_angle: float      # radians
    sweep_angle: float      # radians (+CCW, −CW)

    # helpers
    def _point_at(self, θ: float) -> Point:
        return Point(self.centre.x + self.radius * cos(θ),
                     self.centre.y + self.radius * sin(θ))

    def start_point(self) -> Point:
        return self._point_at(self.start_angle)

    def end_point(self) -> Point:
        return self._point_at(self.start_angle + self.sweep_angle)

    def draw(self, ax: plt.Axes, **kwargs):
        θ = np.linspace(self.start_angle,
                        self.start_angle + self.sweep_angle, 60)
        xs = self.centre.x + self.radius * np.cos(θ)
        ys = self.centre.y + self.radius * np.sin(θ)
        ax.plot(xs, ys, **kwargs)

        # dotted diagnostic circle
        full = np.linspace(0, 2*np.pi, 120)
        ax.plot(self.centre.x + self.radius*np.cos(full),
                self.centre.y + self.radius*np.sin(full),
                linestyle=":", color="gray", lw=0.8, label="_nolegend_")

        ax.plot(self.centre.x, self.centre.y, "+", color="black", label=get_label(ax, "Pivot centre"))
    
    def intersects(self, other: "PathPart") -> bool:
        if isinstance(other, StraightPart):
            return self._intersects_straight(other)
        elif isinstance(other, ArcPart):
            return self._intersects_arc(other)
        return False
    
    def _intersects_straight(self, line: StraightPart) -> bool:
        """Check if arc intersects with a line segment."""
        dx = line.end.x - line.start.x
        dy = line.end.y - line.start.y
        fx = line.start.x - self.centre.x
        fy = line.start.y - self.centre.y
        
        a = dx * dx + dy * dy
        b = 2 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - self.radius * self.radius
        
        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            return False
        
        sqrt_disc = sqrt(discriminant)
        t1 = (-b - sqrt_disc) / (2 * a)
        t2 = (-b + sqrt_disc) / (2 * a)
        
        for t in [t1, t2]:
            if 0 <= t <= 1:
                int_x = line.start.x + t * dx
                int_y = line.start.y + t * dy
                angle = np.arctan2(int_y - self.centre.y, int_x - self.centre.x)
                if self._angle_in_arc(angle):
                    return True
        return False
    
    def _intersects_arc(self, other: "ArcPart") -> bool:
        dist = sqrt((self.centre.x - other.centre.x)**2 + 
                   (self.centre.y - other.centre.y)**2)
        if dist > self.radius + other.radius or dist < abs(self.radius - other.radius):
            return False
        return True
    
    def _angle_in_arc(self, angle: float) -> bool:
        start = self.start_angle % (2 * np.pi)
        end = (self.start_angle + self.sweep_angle) % (2 * np.pi)
        angle = angle % (2 * np.pi)
        
        if self.sweep_angle >= 0:
            if start <= end:
                return start <= angle <= end
            else:
                return angle >= start or angle <= end
        else:
            if start >= end:
                return end <= angle <= start
            else:
                return angle <= start or angle >= end

    def sample_points(self, spacing: float = 0.1) -> list[Point]:
        points = []
        arc_len = abs(self.radius * self.sweep_angle)
        n = max(1, int(np.ceil(arc_len / spacing)))
        for i in range(n):
            theta = self.start_angle + self.sweep_angle * (i / n)
            points.append(self._point_at(theta))
        points.append(self._point_at(self.start_angle + self.sweep_angle))
        return points


@dataclass
class Path:
    parts: list[PathPart] = field(default_factory=list)

    def add_part(self, part: PathPart):
        self.parts.append(part)

    def draw(self, ax: plt.Axes,
             *, seg_color="green", axle_color="red",
             axle_marker="x", axle_size=7):
        # Draw and label each start-point
        for idx, part in enumerate(self.parts):
            p_start = part.start_point()
            ax.plot(p_start.x, p_start.y, axle_marker,
                    color=axle_color, ms=axle_size, label="_nolegend_")
            # ax.text(
            #     p_start.x, p_start.y, str(idx),
            #     color=axle_color, fontsize=axle_size,
            #     ha='center', va='center'
            # )
            part.draw(ax, color=seg_color, lw=2)

        # Draw and label final end-point
        if self.parts:
            p_end = self.parts[-1].end_point()
            ax.plot(p_end.x, p_end.y, axle_marker,
                    color=axle_color, ms=axle_size, label="_nolegend_")
            ax.text(
                p_end.x, p_end.y, str(len(self.parts)),
                color=axle_color, fontsize=axle_size,
                ha='center', va='center'
            )