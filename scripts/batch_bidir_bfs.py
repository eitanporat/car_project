import os
import json
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

from src.algorithms.bidir_bfs_parking import BidirBFSParking, ExpansionConfig
from src.algorithms.heuristic import distance_from_top_curb
from src.core.car import CarSpecs, CarState
from src.core.parking_space import ParkingSpace
from src.core.point import Point
from src.core.path import Path

# Output directory
RESULTS_DIR = "results/bidir_bfs_sweep"
os.makedirs(RESULTS_DIR, exist_ok=True)

# Sweep parameters
LENGTHS = np.arange(7.0, 12.01, 0.5)
WIDTHS = np.arange(2.0, 7.01, 0.5)

# Car specs (fixed)
SPECS = CarSpecs(length=5, width=2, rear_axle=1, minimum_turning_radius=7)

# Common config
COMMON_CFG = dict(
    step_resolution=2.0,
    step_angle_resolution=np.deg2rad(5),
    max_straight=10.0,
    max_radius=100.0,
    max_radius_count=30,
    max_expansions=1500,
)

# Helper: sample points along a PathPart at fixed arc-length intervals
def sample_pathpart(part, spacing=0.1):
    points = []
    if hasattr(part, 'start') and hasattr(part, 'end'):  # StraightPart
        start, end = part.start, part.end
        dx, dy = end.x - start.x, end.y - start.y
        length = np.hypot(dx, dy)
        n = max(1, int(np.ceil(length / spacing)))
        for i in range(n):
            t = i / n
            x = start.x + t * dx
            y = start.y + t * dy
            points.append(Point(x, y))
        points.append(end)
    elif hasattr(part, 'centre') and hasattr(part, 'radius') and hasattr(part, 'start_angle') and hasattr(part, 'sweep_angle'):  # ArcPart
        arc_len = abs(part.radius * part.sweep_angle)
        n = max(1, int(np.ceil(arc_len / spacing)))
        for i in range(n):
            theta = part.start_angle + part.sweep_angle * (i / n)
            points.append(part._point_at(theta))
        points.append(part._point_at(part.start_angle + part.sweep_angle))
    return points

for length in tqdm(LENGTHS, desc="Length sweep"):
    for width in tqdm(WIDTHS, desc=f"Width sweep (L={length:.1f})", leave=False):
        # Setup scenario
        start = CarState(SPECS, 0.0, Point(-length / 2 - SPECS.length / 2, SPECS.width / 2 + 0.5))
        goal1 = CarState(SPECS, 0.0, Point(0.0, SPECS.width / 2 + 0.5 - width))
        goal2 = CarState(SPECS, np.deg2rad(180), Point(0.0, SPECS.width / 2 + 0.5 - width))
        parking = ParkingSpace(
            start=Point(-length / 2, 0),
            width=width,
            length=length,
            baseline_left=10000,
            baseline_right=10000,
            road_width=3.5,
        )
        # Configs
        cfg_start = ExpansionConfig(depth=2, heuristic=None, **COMMON_CFG)
        cfg_goal = ExpansionConfig(depth=2, heuristic=distance_from_top_curb, **COMMON_CFG)
        # Planner
        planner = BidirBFSParking(
            start_state=start,
            goal_states=[goal1, goal2],
            parking_space=parking,
            config_start=cfg_start,
            config_goal=cfg_goal,
            grid_resolution=0.05,
            angle_resolution=np.deg2rad(5),
        )
        # Run search
        result = planner.search(progress_interval=100)
        # Prepare output paths
        base = f"L{length:.1f}_W{width:.1f}"
        outdir = os.path.join(RESULTS_DIR, base)
        os.makedirs(outdir, exist_ok=True)
        json_path = os.path.join(outdir, "result.json")
        img_rear = os.path.join(outdir, "rear_trajectory.png")
        img_full = os.path.join(outdir, "full_trajectory.png")
        # Default output
        output = {"states": None, "actions": None}
        if result:
            path_all, actions = result
            # Reconstruct states
            split = planner.last_split
            poses = [start]
            state = start
            for act in actions:
                state = state.apply_action(act, path=None, parking_space=parking)
                poses.append(state)
            # Save states and actions
            output["states"] = [
                {"x": float(s.center.x), "y": float(s.center.y), "theta": float(s.orientation)} for s in poses
            ]
            output["actions"] = [
                {"kind": a.kind, "magnitude": float(a.magnitude), "direction": str(a.direction) if a.direction else None, "radius": float(a.radius) if a.radius else None} for a in actions
            ]
            # Sample rear axle trajectory along the path (arc length, not x)
            sampled_points = []
            for part in path_all.parts:
                sampled_points.extend(part.sample_points(spacing=0.1))
            xs = np.array([p.x for p in sampled_points])
            ys = np.array([p.y for p in sampled_points])
            # Plot rear axle trajectory
            plt.figure(figsize=(8, 6))
            plt.plot(xs, ys, 'b.-', label='Rear Axle Trajectory')
            plt.xlabel('x [m]')
            plt.ylabel('y [m]')
            plt.title(f'Rear Axle Trajectory (L={length:.1f}, W={width:.1f})')
            plt.grid(True)
            plt.legend()
            plt.savefig(img_rear)
            plt.close()
            # Plot full trajectory with car outlines at each sampled point
            plt.figure(figsize=(8, 6))
            plt.plot(xs, ys, 'b.-', label='Rear Axle Trajectory')
            for pt in sampled_points:
                # Place a car at this rear axle point, using the closest pose orientation
                # (approximate by nearest pose in poses)
                closest_pose = min(poses, key=lambda s: (s.rear_axle_point().x - pt.x)**2 + (s.rear_axle_point().y - pt.y)**2)
                corners = closest_pose.get_corners()
                cx = [p.x for p in corners] + [corners[0].x]
                cy = [p.y for p in corners] + [corners[0].y]
                plt.plot(cx, cy, 'k-', alpha=0.3)
            plt.xlabel('x [m]')
            plt.ylabel('y [m]')
            plt.title(f'Full Trajectory with Car Outlines (L={length:.1f}, W={width:.1f})')
            plt.grid(True)
            plt.legend()
            plt.savefig(img_full)
            plt.close()
        else:
            # No solution: save empty images
            plt.figure(figsize=(8, 6))
            plt.title('No Solution')
            plt.savefig(img_rear)
            plt.close()
            plt.figure(figsize=(8, 6))
            plt.title('No Solution')
            plt.savefig(img_full)
            plt.close()
        # Save JSON
        with open(json_path, 'w') as f:
            json.dump(output, f, indent=2) 