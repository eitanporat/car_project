"""
Compare a hand-crafted 4-step manoeuvre against the BFS planner
and print any poses the planner failed to visit.

Run
----
python bfs_reference_test.py            # with plot
python bfs_reference_test.py --no-plot  # headless
"""

from __future__ import annotations

import argparse
from math import radians, degrees, pi, hypot

import matplotlib.pyplot as plt
import numpy as np

from ..algorithms.bfs_parking import BFSParking
from ..core.action import Action
from ..core.car import CarSpecs, CarState
from ..core.parking_space import ParkingSpace
from ..core.path import Path
from ..core.point import Point
from ..core.direction import Direction


# --------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------- #

p = argparse.ArgumentParser()
p.add_argument("--no-plot", action="store_true")
args = p.parse_args()

# --------------------------------------------------------------------- #
# Hand-crafted reference manoeuvre
# --------------------------------------------------------------------- #

specs = CarSpecs(5, 2, 1, 7)
length, width = 10.0, 5.0

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

reference_actions = [
    Action("rotate", radians(10),  Direction.LEFT,  100),
    Action("rotate", radians(-40),  Direction.RIGHT,  12),
    Action("rotate", radians(-40), Direction.LEFT,    7),
    Action("rotate", radians(10),  Direction.RIGHT,   7),
]

ref_states = [start]
st = start
for act in reference_actions:
    st = st.apply_action(act, path=None, parking_space=parking)
    ref_states.append(st)

# --------------------------------------------------------------------- #
# BFS configured to include those primitives
# --------------------------------------------------------------------- #

bfs = BFSParking(
    start_state=start,
    goal_states=[goal1, goal2],
    parking_space=parking,
    #
    grid_resolution=2,
    angle_resolution=np.deg2rad(20),
    step_resolution=2,
    step_angle_resolution=np.deg2rad(20),
    max_radius=100,
    max_radius_count=5,
    max_straight=8.0,
    max_depth=3,
)

print(bfs._goal_hashes)
print([bfs._state_hash(s) for s in ref_states])

result = bfs.search(progress_interval=100)
assert result is not None, "Planner returned no path"
path, bfs_actions = result

# Replay BFS to collect its states
bfs_states = [start]
st = start
for act in bfs_actions:
    st = st.apply_action(act, path=None, parking_space=parking)
    bfs_states.append(st)

print([bfs._state_hash(s) for s in bfs_states])
# --------------------------------------------------------------------- #
# Compare hashes – print missing poses
# --------------------------------------------------------------------- #

ref_hashes = {bfs._state_hash(s) for s in ref_states}
bfs_hashes = bfs.visited

missing = ref_hashes - bfs_hashes
if missing:
    print("❌  Planner missed the following poses:")
    for h in missing:
        s = next(s for s in ref_states if bfs._state_hash(s) == h)
        print(
            f"  hash={h}   "
            f"x={s.center.x:.3f} m  y={s.center.y:.3f} m  "
            f"heading={degrees(s.orientation):.1f}°"
        )
else:
    print("✅  All manual poses were encountered by the BFS planner!")

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

    # Manual path (grey)
    ref_path = Path()
    st = start
    for act in reference_actions:
        st = st.apply_action(act, path=ref_path, parking_space=parking)
    # ref_path.draw(ax, seg_color="gray")
    # start.draw(ax, state="Start", color="gray")
    # st.draw(ax, state="End (Reference)", color="gray")
    # BFS path (orange)
    path.draw(ax, seg_color="orange")

    # Intermediate outlines
    # for s in ref_states:
    #     s.draw(ax, color="gray")
    for s in bfs_states:
        s.draw(ax, color="orange")

    ax.legend(loc="upper left")
    plt.show()