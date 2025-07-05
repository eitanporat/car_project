from math import radians, sin, cos, hypot
from typing import Tuple
import matplotlib.pyplot as plt

from ..core.car import CarState
from ..core.point import Point
from ..core.parking_space import ParkingSpace

def line_distance(
    state: CarState,
    parking: ParkingSpace,     # unused (kept for API symmetry)
    goal:   CarState,
    λ: float = 0.5,            # ← weight for rear-axle term
) -> float:
    """

    Returns a weighted sum:
        a = ⟂ distance from car's mid-line to goal centre
        b = distance from rear axle to goal centre
        metric = a + λ·b
    """

    dx = goal.center.x - state.center.x
    dy = goal.center.y - state.center.y

    c, s = cos(state.orientation), sin(state.orientation)   # heading cos/sin

    # normal to car's axis is (-sinθ, +cosθ)
    a = abs(dx * s - dy * c)

    ra = state.rear_axle_point()
    _, d_curb, _ = distance_from_curb(state, parking, goal)
    return (2, a + λ * d_curb)


# ------------------------------------------------------------
# basic helper
# ------------------------------------------------------------
def _point_to_segment_dist(p: Point, a: Point, b: Point) -> float:
    """Shortest distance from point p to segment (a-b)."""
    abx, aby = b.x - a.x, b.y - a.y
    apx, apy = p.x - a.x, p.y - a.y
    ab_len2 = abx * abx + aby * aby
    if ab_len2 == 0.0:  # a == b
        return hypot(apx, apy)
    t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab_len2))
    cx = a.x + t * abx
    cy = a.y + t * aby
    return hypot(p.x - cx, p.y - cy)


# ------------------------------------------------------------
# heuristic
# ------------------------------------------------------------
def distance_from_curb(
    state: CarState,
    parking: ParkingSpace,
    goal: CarState,        # kept for BFSParking API symmetry
) -> Tuple[float]:
    """
    Heuristic: minimum perpendicular distance from either long
    side of the car to the curb (segment index 2 of ParkingSpace).
    Returns (d,) so it works as a lexicographic key.
    """
    curb_seg = parking.segments()[2]       # the curb

    # car corners: index order is [front-left, front-right, rear-right, rear-left]
    corners = state.get_corners()

    # distances from both long sides (corner0-1 and corner2-3) to curb
    curb = 0.5 * (curb_seg.start + curb_seg.end)
    left_side  = _point_to_segment_dist(curb, corners[0], corners[1])
    right_side = _point_to_segment_dist(curb, corners[2], corners[3])

    d = min(left_side, right_side) + min(abs(state.orientation - radians(180)), abs(state.orientation)) 

    delta = state.center - goal.center
    return (1, d, delta.norm())

def distance_from_top_curb(
    state: CarState,
    parking: ParkingSpace,
    goal: CarState,        # kept for BFSParking API symmetry
) -> Tuple[float]:
    """
    Heuristic: minimum perpendicular distance from either long
    side of the car to the curb (segment index 2 of ParkingSpace).
    Returns (d,) so it works as a lexicographic key.
    """
    curb_seg = parking.segments()[0]       # the curb

    # car corners: index order is [front-left, front-right, rear-right, rear-left]
    corners = state.get_corners()

    # distances from both long sides (corner0-1 and corner2-3) to curb
    curb = curb_seg.end + Point(-state.specs.length + state.specs.rear_axle, state.specs.width / 2)
    left_side  = (curb - 0.5 * (corners[0] + corners[1])).norm()
    
    right_side = (curb - 0.5 * (corners[2] + corners[3])).norm()

    d = min(left_side, right_side) + min(abs(state.orientation - radians(180)), abs(state.orientation)) 

    delta = state.center - goal.center

    return (1, d, delta.norm())


def line_then_curb(
    state: CarState,
    parking: ParkingSpace,
    goal:   CarState,
    *,
    threshold: float = 5.0,
) -> Tuple[float]:
    """
    Hybrid heuristic:

    1. Compute line_distance.
    2. If rear-axle is within *threshold* metres of goal axle,
       fall back to distance_from_curb.

    Returns (score,) so it can be used as a lexicographic key.
    """

    ra = state.rear_axle_point()
    axle_dist = hypot(ra.x - goal.center.x, ra.y - goal.center.y)

    if axle_dist < threshold:
        curb_score = distance_from_curb(state, parking, goal)
        return curb_score

    line_score = line_distance(state, parking, goal)
    return line_score