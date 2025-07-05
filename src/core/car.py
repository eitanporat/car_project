from __future__ import annotations

"""Car geometry, kinematics and collision-checking utilities.

The module defines:
  • ``CarSpecs``   – immutable physical dimensions of a vehicle
  • ``CarState``   – pose (position + heading) and associated helpers
  • ``CollisionException`` – raised whenever a manoeuvre hits a curb

All motion primitives (*straight* / *rotate*) perform optional collision
checking against a ``ParkingSpace`` instance and optionally record the
exact axle-trajectory into a ``Path`` object.
"""

from dataclasses import dataclass, replace
from math import cos, sin, hypot
from typing import List, Optional, Union

import numpy as np
import matplotlib.pyplot as plt

from .action import Action
from .direction import Direction
from .parking_space import ParkingSpace
from .path import ArcPart, Path, StraightPart
from ..utils.plot_utils import get_label
from .point import Point


class CollisionException(Exception):
    """Raised when a motion primitive would intersect the parking space."""


# ───────────────────────────── Car objects ──────────────────────────────── #

@dataclass(frozen=True)
class CarSpecs:
    length: float
    width: float
    rear_axle: float
    minimum_turning_radius: float


@dataclass
class CarState:
    """Full pose of a car plus geometry / collision helpers."""

    specs: CarSpecs
    orientation: float  # radians
    center: Point

    # ------------------------------------------------------------------ #
    # geometry helpers
    # ------------------------------------------------------------------ #
    def _rot_offset(self, dx: float, dy: float) -> Point:
        c, s = cos(self.orientation), sin(self.orientation)
        return Point(self.center.x + dx * c - dy * s,
                     self.center.y + dx * s + dy * c)

    def rear_axle_point(self) -> Point:
        return self._rot_offset(-self.specs.length / 2 + self.specs.rear_axle, 0)

    def curvature_center(self, radius: float, side: Direction) -> Point:
        offset = +radius if side is Direction.LEFT else -radius
        return self._rot_offset(-self.specs.length / 2 + self.specs.rear_axle, offset)

    def get_corners(self) -> list[Point]:
        hl, hw = self.specs.length / 2, self.specs.width / 2
        return [
            self._rot_offset(-hl, hw),  # rear-left
            self._rot_offset(hl, hw),   # front-left
            self._rot_offset(hl, -hw),  # front-right
            self._rot_offset(-hl, -hw), # rear-right
        ]

    # ------------------------------------------------------------------ #
    # collision helpers (static + dynamic tests)
    # ------------------------------------------------------------------ #
    def _static_rectangle_hits_segment(self, seg: StraightPart) -> bool:
        """True if the *current* rectangle overlaps *seg* (curb)."""
        corners = self.get_corners()
        rect_edges = [StraightPart(corners[i], corners[(i + 1) % 4]) for i in range(4)]

        # Edge–edge intersections
        if any(e.intersects(seg) for e in rect_edges):
            return True

        # Segment fully inside rectangle – test one end-point
        px = seg.start
        inside = True
        for i in range(4):
            a, b = corners[i], corners[(i + 1) % 4]
            if (b.x - a.x) * (px.y - a.y) - (b.y - a.y) * (px.x - a.x) < 0:
                inside = False
                break
        return inside

    def _dynamic_paths_hit_curb(
        self,
        paths: List[Union[ArcPart, StraightPart]],
        curb_segments: list[StraightPart],
    ) -> bool:
        return any(cp.intersects(seg) for cp in paths for seg in curb_segments)

    def _check_collision_with_parking_space(
        self,
        paths: List[Union[ArcPart, StraightPart]],
        parking_space: ParkingSpace,
    ) -> None:
        curb_segments = parking_space.segments()
        if self._dynamic_paths_hit_curb(paths, curb_segments):
            raise CollisionException("Car would hit parking space (dynamic)")
        if any(self._static_rectangle_hits_segment(seg) for seg in curb_segments):
            raise CollisionException("Car would hit parking space (static)")

    # ------------------------------------------------------------------ #
    # kinematics
    # ------------------------------------------------------------------ #
    def _corner_paths_for_rotation(self, angle: float, radius: float, side: Direction) -> List[ArcPart]:
        sign = +1 if side is Direction.LEFT else -1
        pivot = self.curvature_center(radius, side)
        out: List[ArcPart] = []
        for c in self.get_corners():
            r = hypot(c.x - pivot.x, c.y - pivot.y)
            a0 = np.arctan2(c.y - pivot.y, c.x - pivot.x)
            out.append(ArcPart(pivot, r, a0, sign * angle))
        return out

    def _corner_paths_for_straight(self, dist: float) -> List[StraightPart]:
        cθ, sθ = cos(self.orientation), sin(self.orientation)
        return [
            StraightPart(corner, Point(corner.x + cθ * dist, corner.y + sθ * dist))
            for corner in self.get_corners()
        ]

    # ................................................................. #
    # public motion primitives
    # ................................................................. #
    def rotate(
        self,
        angle: float,
        radius: float,
        side: Direction,
        path: Optional[Path] = None,
        parking_space: Optional[ParkingSpace] = None,
    ) -> "CarState":
        if radius < self.specs.minimum_turning_radius:
            raise ValueError("radius < minimum_turning_radius")
        if parking_space is not None:
            self._check_collision_with_parking_space(
                self._corner_paths_for_rotation(angle, radius, side), parking_space
            )

        sign = +1 if side is Direction.LEFT else -1
        pivot = self.curvature_center(radius, side)
        new_center = pivot + (self.center - pivot).rotate(sign * angle)
        new_heading = (self.orientation + sign * angle) % (2 * np.pi)

        if path is not None:
            a0 = (self.rear_axle_point() - pivot).angle()
            path.add_part(ArcPart(pivot, radius, a0, sign * angle))

        return replace(self, center=new_center, orientation=new_heading)

    def straight(
        self,
        dist: float,
        path: Optional[Path] = None,
        parking_space: Optional[ParkingSpace] = None,
    ) -> "CarState":
        if parking_space is not None:
            self._check_collision_with_parking_space(
                self._corner_paths_for_straight(dist), parking_space
            )

        cθ, sθ = cos(self.orientation), sin(self.orientation)
        new_center = Point(self.center.x + cθ * dist, self.center.y + sθ * dist)
        nxt = replace(self, center=new_center)

        if path is not None:
            path.add_part(StraightPart(self.rear_axle_point(), nxt.rear_axle_point()))
        return nxt

    # ------------------------------------------------------------------ #
    # drawing helpers (unchanged API)
    # ------------------------------------------------------------------ #
    def draw(
        self,
        ax: plt.Axes,
        *,
        color: str = "blue",
        state: str = "Start",
        fill: bool = False,
        fill_color: Optional[str] = None,
        show_axle: bool = True,
        lw: float = 1.5,
    ) -> None:
        corners = self.get_corners()
        xs = [p.x for p in corners] + [corners[0].x]
        ys = [p.y for p in corners] + [corners[0].y]

        if fill:
            fc = fill_color or color
            ax.fill(xs[:-1], ys[:-1], fc, edgecolor=color, lw=lw,
                    label=get_label(ax, state) if not state.startswith("_") else "_nolegend_")
        else:
            ax.plot(xs, ys, color=color, lw=lw,
                    label=get_label(ax, state) if not state.startswith("_") else "_nolegend_")
        if show_axle:
            ra = self.rear_axle_point()
            ax.plot(ra.x, ra.y, "rx", ms=2, mew=2, label="_nolegend_")

    def apply_action(
        self,
        action: Action,
        *,
        path: Optional[Path] = None,
        parking_space: Optional[ParkingSpace] = None,
    ) -> "CarState":
        if action.kind == "straight":
            return self.straight(action.magnitude, path, parking_space)
        if action.kind == "rotate":
            if action.direction is None or action.radius is None:
                raise ValueError("Rotate action missing direction and/or radius")
            return self.rotate(action.magnitude, action.radius, action.direction, path, parking_space)
        raise ValueError(f"Unknown Action.kind: {action.kind!r}")