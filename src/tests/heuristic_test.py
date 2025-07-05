from __future__ import annotations

import argparse
from math import radians
from typing import List

import matplotlib.pyplot as plt
import numpy as np

from ..algorithms.bfs_parking import BFSParking, ExpansionConfig
from ..core.car import CarSpecs, CarState
from ..core.parking_space import ParkingSpace
from ..core.point import Point

# ────────────────────────────────────────────────────────────────
# Heuristic functions
# ────────────────────────────────────────────────────────────────
from ..algorithms.heuristic import (
    distance_from_top_curb,
    line_distance,
    distance_from_curb,
    line_then_curb,
)

HEURISTICS = {
    "none": None,
    "line": line_distance,
    "curb": distance_from_curb,
    "curb_top": distance_from_top_curb,
    "combo": line_then_curb,
}

# ────────────────────────────────────────────────────────────────
# CLI
# ────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument(
    "--heuristic",
    choices=HEURISTICS.keys(),
    default="none",
    help="Choose heuristic: none | line | curb | curb_top | combo (default: none)",
)
parser.add_argument("--no-plot", action="store_true")
args = parser.parse_args()
heuristic_fn = HEURISTICS[args.heuristic]

# ────────────────────────────────────────────────────────────────
# Scenario
# ────────────────────────────────────────────────────────────────
specs = CarSpecs(5, 2, 1, 7)
length, width = 8.0, 4.0

start = CarState(
    specs,
    0.0,
    Point(-length / 2 - specs.length / 2, specs.width / 2 + 0.5),
)
goal1 = CarState(
    specs,
    0.0,
    Point(0.0, specs.width / 2 + 0.5 - width),
)
goal2 = CarState(
    specs,
    radians(180),
    Point(0.0, specs.width / 2 + 0.5 - width),
)

parking = ParkingSpace(Point(-length / 2, 0), width, length, 10000, 10000)

# ────────────────────────────────────────────────────────────────
# Planner configuration
# ────────────────────────────────────────────────────────────────
exp_cfg = ExpansionConfig(
    step_resolution=2.0,
    step_angle_resolution=np.deg2rad(10),
    max_straight=10.0,
    max_radius=100.0,
    max_radius_count=30,
    depth=4,
    max_expansions=1_500,          # moved here!
    max_turn_sweep=2 * np.pi,
    max_dist_ratio=1.5,
)

bfs = BFSParking(
    start_state=start,
    goal_states=[goal1, goal2],
    parking_space=parking,
    expansion_config=exp_cfg,
    heuristic=heuristic_fn,
    grid_resolution=0.5,
    angle_resolution=np.deg2rad(10),
)

result = bfs.search(progress_interval=100)
assert result is not None, "Search failed to return a path"
path, actions = result

# ────────────────────────────────────────────────────────────────
# Re-create intermediate poses (for numbering on the plot)
# ────────────────────────────────────────────────────────────────
poses: List[CarState] = [start]
cur = start
for act in actions:
    if act.kind == "straight":
        cur = cur.straight(act.magnitude, path=None, parking_space=parking)
    else:  # rotate
        cur = cur.rotate(
            act.magnitude, act.radius, act.direction, path=None, parking_space=parking
        )
    poses.append(cur)

# ────────────────────────────────────────────────────────────────
# Plot
# ────────────────────────────────────────────────────────────────
if not args.no_plot:
    title_h = {
        None: "Pure BFS (no heuristic)",
        line_distance: "Best-first with line_distance heuristic",
        distance_from_curb: "Best-first with distance_from_curb heuristic",
        line_then_curb: "Best-first with line_then_curb heuristic",
        distance_from_top_curb: "Best-first with distance_from_top_curb heuristic",
    }[heuristic_fn]

    fig, ax = plt.subplots(figsize=(9, 6))
    ax.set_xlim(-25, 25)
    ax.set_ylim(-25, 25)
    ax.set_aspect("equal")
    ax.grid(True)
    ax.set_title(title_h)

    parking.draw(ax)
    goal1.draw(
        ax,
        state="Goal",
        color="green",
        fill=True,
        fill_color="lightgreen",
        show_axle=False,
    )
    start.draw(
        ax,
        state="Start",
        color="red",
        fill=True,
        fill_color="lightcoral",
        show_axle=False,
    )
    path.draw(ax, seg_color="orange")

    # draw numbered poses
    for idx, pose in enumerate(poses):
        pose.draw(ax, color="blue")
        ra = pose.rear_axle_point()
        ax.text(
            ra.x,
            ra.y + 0.5,
            str(idx),
            fontsize=12,
            color="black",
            ha="center",
            va="bottom",
        )

    ax.legend(loc="upper left")
    plt.show()