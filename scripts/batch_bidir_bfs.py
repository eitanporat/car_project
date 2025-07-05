import os
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend for headless operation
import matplotlib.pyplot as plt
from tqdm import tqdm
import datetime
import argparse

from src.algorithms.bidir_bfs_parking import BidirBFSParking, ExpansionConfig
from src.algorithms.heuristic import distance_from_top_curb
from src.core.car import CarSpecs, CarState
from src.core.parking_space import ParkingSpace
from src.core.point import Point
from src.core.path import Path

def parse_args():
    parser = argparse.ArgumentParser(description='Batch bidirectional BFS parking sweep')
    parser.add_argument('--length-min', type=float, default=7.0, help='Minimum parking length (default: 7.0)')
    parser.add_argument('--length-max', type=float, default=20.0, help='Maximum parking length (default: 20.0)')
    parser.add_argument('--length-step', type=float, default=2.0, help='Length step size (default: 2.0)')
    parser.add_argument('--width-min', type=float, default=2.0, help='Minimum parking width (default: 2.0)')
    parser.add_argument('--width-max', type=float, default=7.0, help='Maximum parking width (default: 7.0)')
    parser.add_argument('--width-step', type=float, default=1.0, help='Width step size (default: 1.0)')
    parser.add_argument('--fwd-depth', type=int, default=2, help='Forward search depth (default: 2)')
    parser.add_argument('--rev-depth', type=int, default=2, help='Reverse search depth (default: 2)')
    parser.add_argument('--step-resolution', type=float, default=2.0, help='Step resolution (default: 2.0)')
    parser.add_argument('--angle-resolution', type=float, default=5.0, help='Angle resolution in degrees (default: 5.0)')
    parser.add_argument('--max-straight', type=float, default=10.0, help='Max straight distance (default: 10.0)')
    parser.add_argument('--max-radius', type=float, default=100.0, help='Max turning radius (default: 100.0)')
    parser.add_argument('--max-radius-count', type=int, default=30, help='Max radius count (default: 30)')
    parser.add_argument('--max-expansions', type=int, default=1500, help='Max expansions (default: 1500)')
    parser.add_argument('--grid-resolution', type=float, default=0.05, help='Grid resolution (default: 0.05)')
    parser.add_argument('--progress-interval', type=int, default=100, help='Progress reporting interval (default: 100)')
    return parser.parse_args()

def main():
    args = parse_args()
    
    # Set up dated results directory
    run_time = datetime.datetime.now().strftime('%H-%M-%S')
    results_dir = os.path.join('results', run_time)
    os.makedirs(results_dir, exist_ok=True)
    
    # Sweep parameters
    LENGTHS = np.arange(args.length_min, args.length_max + args.length_step, args.length_step)
    WIDTHS = np.arange(args.width_min, args.width_max + args.width_step, args.width_step)
    
    # Car specs (fixed)
    SPECS = CarSpecs(length=5, width=2, rear_axle=1, minimum_turning_radius=7)
    
    # Common config
    COMMON_CFG = dict(
        step_resolution=args.step_resolution,
        step_angle_resolution=np.deg2rad(args.angle_resolution),
        max_straight=args.max_straight,
        max_radius=args.max_radius,
        max_radius_count=args.max_radius_count,
        max_expansions=args.max_expansions,
    )

    print(f"Starting sweep: {len(LENGTHS)} lengths × {len(WIDTHS)} widths = {len(LENGTHS) * len(WIDTHS)} total configurations")
    print(f"Length range: {args.length_min:.1f} to {args.length_max:.1f} (step {args.length_step:.1f})")
    print(f"Width range: {args.width_min:.1f} to {args.width_max:.1f} (step {args.width_step:.1f})")
    print(f"Forward depth: {args.fwd_depth}, Reverse depth: {args.rev_depth}")
    print(f"Results will be saved to: {results_dir}")
    
    # Save sweep configuration to config.json
    config = {
        "sweep_params": {
            "length_min": args.length_min,
            "length_max": args.length_max,
            "length_step": args.length_step,
            "width_min": args.width_min,
            "width_max": args.width_max,
            "width_step": args.width_step,
            "total_configurations": len(LENGTHS) * len(WIDTHS)
        },
        "algorithm_params": {
            "fwd_depth": args.fwd_depth,
            "rev_depth": args.rev_depth,
            "step_resolution": args.step_resolution,
            "angle_resolution": args.angle_resolution,
            "max_straight": args.max_straight,
            "max_radius": args.max_radius,
            "max_radius_count": args.max_radius_count,
            "max_expansions": args.max_expansions,
            "grid_resolution": args.grid_resolution,
            "progress_interval": args.progress_interval,
            "road_width": 5.0
        },
        "car_specs": {
            "length": 5.0,
            "width": 2.0,
            "rear_axle": 1.0,
            "minimum_turning_radius": 7.0
        }
    }
    
    config_path = os.path.join(results_dir, "config.json")
    with open(config_path, 'w') as f:
        json.dump(config, f, indent=2)
    
    # Sweep over parking lot dimensions
    for length in tqdm(LENGTHS, desc="Length sweep"):
        for width in tqdm(WIDTHS, desc=f"Bidirectional BFS (depth FWD={args.fwd_depth}  REV={args.rev_depth})", leave=False):
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
                road_width=5.0,
            )
            # Configs
            cfg_start = ExpansionConfig(depth=args.fwd_depth, heuristic=None, **COMMON_CFG)
            cfg_goal = ExpansionConfig(depth=args.rev_depth, heuristic=distance_from_top_curb, **COMMON_CFG)
            # Planner
            planner = BidirBFSParking(
                start_state=start,
                goal_states=[goal1, goal2],
                parking_space=parking,
                config_start=cfg_start,
                config_goal=cfg_goal,
                grid_resolution=args.grid_resolution,
                angle_resolution=np.deg2rad(args.angle_resolution),
            )
            # Run search
            result = planner.search(progress_interval=args.progress_interval)
            # Prepare output paths
            base = f"L{length:.1f}_W{width:.1f}"
            json_path = os.path.join(results_dir, f"{base}.json")
            img_rear = os.path.join(results_dir, f"{base}_rear.png")
            img_full = os.path.join(results_dir, f"{base}_full.png")
            # Default output
            output = {"states": None, "actions": None, "visited_states": 0}
            if result:
                path_all, actions = result
                # Get visited states count from planner metadata
                visited_states = planner.total_visited_states
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
                output["visited_states"] = visited_states
                # Use only the original poses for plotting
                rear_points = [s.rear_axle_point() for s in poses]
                xs = np.array([p.x for p in rear_points])
                ys = np.array([p.y for p in rear_points])
                # Plot 1: Rear axle trajectory only, with parking, start, and goal
                plt.figure(figsize=(10, 6))
                plt.subplots_adjust(left=0.05, right=0.95, top=0.9, bottom=0.1)
                ax1 = plt.gca()
                ax1.set_xlim(-20, 20)
                ax1.set_ylim(-8, 8)
                ax1.set_aspect("equal")
                ax1.grid(True)
                parking.draw(ax1)
                start.draw(ax1, state="Start", color="red", fill=True, fill_color="lightcoral", show_axle=True)
                goal1.draw(ax1, state="Goal", color="green", fill=True, fill_color="lightgreen", show_axle=True)
                # Plot rear axle points as red "x" markers
                ax1.plot(xs, ys, 'rx', ms=2, mew=2, label='_nolegend_')
                # Plot the actual path segments
                path_all.draw(ax1, seg_color="orange", axle_color="red")
                # Add numbered text labels at each pose
                for idx, pose in enumerate(poses):
                    ra = pose.rear_axle_point()
                    ax1.text(ra.x, ra.y + 0.5, str(idx), fontsize=12,
                            ha="center", va="bottom", color="black")
                ax1.set_xlabel('x [m]')
                ax1.set_ylabel('y [m]')
                ax1.set_title(f'Rear Axle Trajectory (L={length:.1f}, W={width:.1f})')
                ax1.legend(loc="upper left")
                plt.tight_layout()
                plt.savefig(img_rear)
                plt.close()
                # Plot 2: Full trajectory with car outlines at each pose
                plt.figure(figsize=(10, 6))
                plt.subplots_adjust(left=0.05, right=0.95, top=0.9, bottom=0.1)
                ax2 = plt.gca()
                ax2.set_xlim(-20, 20)
                ax2.set_ylim(-8, 8)
                ax2.set_aspect("equal")
                ax2.grid(True)
                parking.draw(ax2)
                start.draw(ax2, state="Start", color="red", fill=True, fill_color="lightcoral", show_axle=True)
                goal1.draw(ax2, state="Goal", color="green", fill=True, fill_color="lightgreen", show_axle=True)
                # Plot rear axle points as red "x" markers
                ax2.plot(xs, ys, 'rx', ms=2, mew=2, label='_nolegend_')
                # Plot the actual path segments
                path_all.draw(ax2, seg_color="orange", axle_color="red")
                for s in poses:
                    corners = s.get_corners()
                    cx = [p.x for p in corners] + [corners[0].x]
                    cy = [p.y for p in corners] + [corners[0].y]
                    ax2.plot(cx, cy, 'k-', alpha=0.3)
                # Add numbered text labels at each pose
                for idx, pose in enumerate(poses):
                    ra = pose.rear_axle_point()
                    ax2.text(ra.x, ra.y + 0.5, str(idx), fontsize=12,
                            ha="center", va="bottom", color="black")
                ax2.set_xlabel('x [m]')
                ax2.set_ylabel('y [m]')
                ax2.set_title(f'Full Trajectory with Car Outlines (L={length:.1f}, W={width:.1f})')
                ax2.legend(loc="upper left")
                plt.tight_layout()
                plt.savefig(img_full)
                plt.close()
            else:
                # Get visited states count from planner metadata
                visited_states = planner.total_visited_states
                output["visited_states"] = visited_states
                # No solution: save empty images
                # Plot 1: Parking space, start, and goal (no trajectory)
                plt.figure(figsize=(10, 6))
                plt.subplots_adjust(left=0.05, right=0.95, top=0.9, bottom=0.1)
                ax1 = plt.gca()
                ax1.set_xlim(-20, 20)
                ax1.set_ylim(-8, 8)
                ax1.set_aspect("equal")
                ax1.grid(True)
                parking.draw(ax1)
                start.draw(ax1, state="Start", color="red", fill=True, fill_color="lightcoral", show_axle=True)
                goal1.draw(ax1, state="Goal", color="green", fill=True, fill_color="lightgreen", show_axle=True)
                ax1.set_xlabel('x [m]')
                ax1.set_ylabel('y [m]')
                ax1.set_title(f'No Solution Found (L={length:.1f}, W={width:.1f})')
                ax1.legend(loc="upper left")
                plt.tight_layout()
                plt.savefig(img_rear)
                plt.close()
                # Plot 2: Same as plot 1 (no trajectory to show)
                plt.figure(figsize=(10, 6))
                ax2 = plt.gca()
                plt.subplots_adjust(left=0.05, right=0.95, top=0.9, bottom=0.1)
                ax2.set_xlim(-20, 20)
                ax2.set_ylim(-8, 8)
                ax2.set_aspect("equal")
                ax2.grid(True)
                parking.draw(ax2)
                start.draw(ax2, state="Start", color="red", fill=True, fill_color="lightcoral", show_axle=True)
                goal1.draw(ax2, state="Goal", color="green", fill=True, fill_color="lightgreen", show_axle=True)
                ax2.set_xlabel('x [m]')
                ax2.set_ylabel('y [m]')
                ax2.set_title(f'No Solution Found (L={length:.1f}, W={width:.1f})')
                ax2.legend(loc="upper left")
                plt.tight_layout()
                plt.savefig(img_full)
                plt.close()
            # Save JSON
            with open(json_path, 'w') as f:
                json.dump(output, f, indent=2)
    
    print(f"\nSweep completed! Results saved to: {results_dir}")

if __name__ == "__main__":
    main() 