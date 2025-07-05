from __future__ import annotations

import argparse
from math import radians
from typing import List

import matplotlib.pyplot as plt
import numpy as np

from ..algorithms.bidir_bfs_parking import BidirBFSParking, ExpansionConfig
from ..core.car import CarSpecs, CarState
from ..core.parking_space import ParkingSpace
from ..core.point import Point
from ..core.path import Path

# ──────────────────────────────────────────────────────────────────────
# heuristic functions
# ──────────────────────────────────────────────────────────────────────
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

# ──────────────────────────────────────────────────────────────────────
# main
# ──────────────────────────────────────────────────────────────────────
def main() -> None:
    # CLI ----------------------------------------------------------------
    p = argparse.ArgumentParser()
    p.add_argument(
        "--heuristic", choices=HEURISTICS, default="none",
        help="Forward-front heuristic: none | line | curb | curb_top | combo"
    )
    p.add_argument("--no-plot", action="store_true", help="Disable plotting")
    args = p.parse_args()

    heur_fwd = HEURISTICS[args.heuristic]
    heur_rev = distance_from_top_curb                          # fixed

    # scenario -----------------------------------------------------------
    specs = CarSpecs(5, 2, 1, 7)
    length, width = 8.0, 3.0

    start = CarState(specs, 0.0,
                     Point(-length / 2 - specs.length / 2, specs.width / 2 + 0.5))
    goal1 = CarState(specs, 0.0,
                     Point(0.0, specs.width / 2 + 0.5 - width))
    goal2 = CarState(specs, radians(180),
                     Point(0.0, specs.width / 2 + 0.5 - width))

    parking = ParkingSpace(start=Point(-length / 2, 0), 
                           width=width, 
                           length=length, 
                           baseline_left=10_000, 
                           baseline_right=10_000, road_width=3.5)

    # planner configuration ---------------------------------------------
    common_cfg = dict(
        step_resolution=2.0,
        step_angle_resolution=np.deg2rad(5),
        max_straight=10.0,
        max_radius=100.0,
        max_radius_count=30,
        max_expansions=1_500,
    )
    cfg_start = ExpansionConfig(depth=2, heuristic=heur_fwd, **common_cfg)
    cfg_goal  = ExpansionConfig(depth=3, heuristic=heur_rev, **common_cfg)

    planner = BidirBFSParking(
        start_state=start,
        goal_states=[goal1, goal2],
        parking_space=parking,
        config_start=cfg_start,
        config_goal=cfg_goal,
        grid_resolution=0.05,
        angle_resolution=np.deg2rad(5),
    )

    result = planner.search(progress_interval=100)
    assert result, "Search failed to return a path"
    path_all, actions = result
    split = planner.last_split              # index of MEET action

    # -------------------------------------------------------------------
    # reconstruct forward half (start → meet)
    # -------------------------------------------------------------------
    path_fwd = Path()
    poses_fwd: List[CarState] = [start]
    state = start
    for act in actions[:split]:
        state = state.apply_action(act, path=path_fwd, parking_space=parking)
        poses_fwd.append(state)
    meet_fwd = state                        # last forward pose

    # -------------------------------------------------------------------
    # reconstruct reverse half in *forward travel order* (meet → goal)
    # -------------------------------------------------------------------
    path_rev = Path()
    poses_rev: List[CarState] = []
    state = meet_fwd
    for act in actions[split:]:             # play remaining actions forward
        state = state.apply_action(act, path=path_rev, parking_space=parking)
        poses_rev.append(state)

    if args.no_plot:
        return

    # -------------------------------------------------------------------
    # drawing
    # -------------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(9, 6))
    ax.set_xlim(-25, 25); ax.set_ylim(-25, 25)
    ax.set_aspect("equal"); ax.grid(True)
    ax.set_title(
        f"Bidirectional BFS — FWD:{args.heuristic.upper()} depth={cfg_start.depth} | "
        f"REV:TOP-CURB depth={cfg_goal.depth}"
    )

    # environment
    parking.draw(ax)
    goal1.draw(ax, state="Goal",  color="green", fill=True,
               fill_color="lightgreen", show_axle=False)
    start.draw(ax, state="Start", color="red",   fill=True,
               fill_color="lightcoral", show_axle=False)

    # paths
    path_fwd.draw(ax, seg_color="blue")
    path_rev.draw(ax, seg_color="purple")

    # numbered poses -----------------------------------------------------
    # forward half
    for idx, pose in enumerate(poses_fwd):
        pose.draw(ax, color="blue")
        ra = pose.rear_axle_point()
        ax.text(ra.x, ra.y + 0.5, str(idx), fontsize=12,
                ha="center", va="bottom", color="black")

    # reverse half
    offset = len(poses_fwd)    # MEET is already labelled len(poses_fwd)-1
    for jdx, pose in enumerate(poses_rev, 1):
        pose.draw(ax, color="purple")
        ra = pose.rear_axle_point()
        ax.text(ra.x, ra.y + 0.5, str(offset + jdx - 1), fontsize=12,
                ha="center", va="bottom", color="black")


    ax.legend(loc="upper left")
    plt.show()


if __name__ == "__main__":
    main()