from __future__ import annotations

import argparse
from math import radians

import matplotlib.pyplot as plt
import numpy as np

from ..algorithms.bfs_parking import BFSParking, ExpansionConfig
from ..core.car import CarSpecs, CarState
from ..core.parking_space import ParkingSpace
from ..core.path import Path
from ..core.point import Point


p = argparse.ArgumentParser()
p.add_argument("--no-plot", action="store_true")
args = p.parse_args()


specs = CarSpecs(5, 2, 1, 7)
length, width = 10.0, 4.0

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
parking = ParkingSpace(Point(-length / 2, 0), width, length, 10, 10)

expansion_config = ExpansionConfig(
    step_resolution=2,
    step_angle_resolution=np.deg2rad(10),
    max_radius=100,
    max_radius_count=5,
    max_straight=8.0,
    depth=4,
)

bfs = BFSParking(
    start_state=start,
    goal_states=[goal1, goal2],
    parking_space=parking,
    expansion_config=expansion_config,
    grid_resolution=2,
    angle_resolution=np.deg2rad(10),
)

result = bfs.search(progress_interval=100)
assert result is not None, "Planner returned no path"
path, bfs_actions = result

# Replay BFS to collect its states
bfs_states = [start]
st = start
for act in bfs_actions:
    st = st.apply_action(act, path=None, parking_space=parking)
    bfs_states.append(st)
# --------------------------------------------------------------------- #
# Optional plot
# --------------------------------------------------------------------- #

if not args.no_plot:
    fig, ax = plt.subplots(figsize=(9, 6))
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)
    ax.set_aspect("equal")
    ax.grid(True)
    ax.set_title("BFS trajectory")

    parking.draw(ax)
    goal1.draw(ax, state="Goal", color="green")

    path.draw(ax, seg_color="orange")

    # numbered BFS states
    for idx, s in enumerate(bfs_states):
        s.draw(ax, color="orange")
        ra = s.rear_axle_point()
        ax.text(
            ra.x, ra.y + 0.5,    # lift further above the marker
            str(idx),
            color="black",       # now black
            fontsize=12,
            ha="center", va="bottom"
        )

    ax.legend(loc="upper left")
    plt.show()