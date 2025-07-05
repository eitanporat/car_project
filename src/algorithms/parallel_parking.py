from ..core.car import CarSpecs, CarState, CollisionException
from ..core.parking_space import ParkingSpace
from ..core.path import Path
from ..core.point import Point
from ..core.direction import Direction
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--length", type=float, default=10)
    parser.add_argument("--width", type=float, default=5)
    args = parser.parse_args()
    
    parking_length = args.length
    parking_width = args.width
    base_line = 10

    specs = CarSpecs(length=5, width=2, rear_axle=1, minimum_turning_radius=7)
    state = CarState(specs, orientation=0.0, center=Point(-parking_length / 2 - specs.length / 2, specs.width / 2 + 0.5))
    
    desired_state = CarState(specs, orientation=0.0, center=Point(0, specs.width / 2 + 0.5 - parking_width))
    path = Path()

    fig, ax = plt.subplots(figsize=(9, 6))
    ax.set_xlim(-20, 20); ax.set_ylim(-20, 20); ax.set_aspect("equal")
    ax.grid(True); ax.set_title("Parallel-parking")
    
    parking = ParkingSpace(start=Point(-parking_length / 2, 0),
                           width=parking_width, length=parking_length,
                           baseline_left=base_line, baseline_right=base_line)

    parking.draw(ax)
    state.draw(ax, state="Start", color="blue")

    try:
        state = state.rotate(np.deg2rad(10), 100, Direction.LEFT, path, parking_space=parking)
        state.draw(ax, state="Middle", color="blue")

        state = state.rotate(np.deg2rad(-40), 12, Direction.RIGHT, path, parking_space=parking)
        state.draw(ax, state="Middle", color="blue")

        state = state.rotate(np.deg2rad(-40), 7, Direction.LEFT, path, parking_space=parking)
        state.draw(ax, state="Middle", color="blue")

        state = state.rotate(np.deg2rad(10), 7, Direction.RIGHT, path, parking_space=parking)
        state.draw(ax, state="End", color="blue")
        
    except CollisionException as e:
        print(f"Collision detected: {e}")
        ax.set_title("Parallel-parking (collision detected)", color="red")

    path.draw(ax, seg_color="gray")
    desired_state.draw(ax, state="Desired", color="green")

    ax.legend(loc="upper left")
    plt.show()