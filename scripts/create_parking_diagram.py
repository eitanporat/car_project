#!/usr/bin/env python3
"""
Create a simple parking space diagram for the README.
"""

import matplotlib.pyplot as plt
import numpy as np
from src import CarSpecs, CarState, ParkingSpace, Point

def create_parking_diagram():
    """Create a simple parking space diagram using existing drawing methods"""
    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    
    # Create car specs
    car_specs = CarSpecs(length=5.0, width=2.0, rear_axle=1.0, minimum_turning_radius=7.0)
    
    # Create car state (positioned outside parking space)
    car_state = CarState(specs=car_specs, orientation=0.0, center=Point(-8, 2))
    
    # Create parking space
    parking = ParkingSpace(start=Point(-5, 0), width=4, length=10)
    
    # Set plot properties
    ax.set_xlim(-12, 8)
    ax.set_ylim(-2, 8)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (meters)', fontsize=12)
    ax.set_ylabel('Y (meters)', fontsize=12)
    ax.set_title('Parking Space and Car Geometry', fontsize=16, fontweight='bold')
    
    # Draw parking space using existing method
    parking.draw(ax)
    
    # Draw car using existing method
    car_state.draw(ax, state="Car", color="blue")
    
    # Add a goal state inside the parking space
    goal_state = CarState(specs=car_specs, orientation=0.0, center=Point(-2, 1))
    goal_state.draw(ax, state="Goal", color="green")
    
    # Add legend
    ax.legend(loc='upper right', fontsize=10)
    
    # Save the plot
    plt.tight_layout()
    plt.savefig('docs/images/parking_diagram.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    print("Parking diagram saved to: docs/images/parking_diagram.png")

if __name__ == "__main__":
    create_parking_diagram() 