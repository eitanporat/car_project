#!/usr/bin/env python3
"""
Create a simple parking space diagram for the README.
"""

import matplotlib.pyplot as plt
import numpy as np
from math import radians
from src import CarSpecs, CarState, ParkingSpace, Point

def create_parking_diagram():
    """Create a simple parking space diagram using existing drawing methods"""
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Use the same setup as the reference test
    specs = CarSpecs(5, 2, 1, 7)
    length, width = 10.0, 5.0
    
    # Create start state (positioned outside parking space)
    start = CarState(
        specs,
        0.0,
        Point(-length / 2 - specs.length / 2, specs.width / 2 + 0.5),
    )
    
    # Create goal state inside parking space
    goal = CarState(
        specs,
        0.0,
        Point(0.0, specs.width / 2 + 0.5 - width),
    )
    
    # Create parking space
    parking = ParkingSpace(Point(-length / 2, 0), width, length, 10, 10)
    
    # Set plot properties
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)
    ax.set_aspect("equal")
    ax.grid(True)
    ax.set_title("Parking Space and Car Geometry", fontsize=16, fontweight='bold')
    
    # Draw parking space using existing method
    parking.draw(ax)
    
    # Draw start car using existing method
    start.draw(ax, state="Start", color="blue")
    
    # Draw goal car using existing method (without axle)
    goal.draw(ax, state="Goal", color="green")
    
    # Add legend
    ax.legend(loc='upper left', fontsize=10)
    
    # Save the plot
    plt.tight_layout()
    plt.savefig('docs/images/parking_diagram.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    print("Parking diagram saved to: docs/images/parking_diagram.png")

if __name__ == "__main__":
    create_parking_diagram() 