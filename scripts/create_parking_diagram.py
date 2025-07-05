#!/usr/bin/env python3
"""
Create a simple parking space diagram for the README.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Rectangle, FancyBboxPatch
import numpy as np
from src import CarSpecs, CarState, ParkingSpace, Point

def create_parking_diagram():
    """Create a simple parking space diagram"""
    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    
    # Create car specs
    car_specs = CarSpecs(length=5.0, width=2.0, rear_axle=1.0, minimum_turning_radius=7.0)
    
    # Create car state (positioned outside parking space)
    car_state = CarState(specs=car_specs, orientation=0.0, center=Point(-8, 2))
    
    # Create parking space
    parking = ParkingSpace(start=Point(-5, 0), width=4, length=10)
    
    # Draw parking space boundaries
    # Bottom curb (parking space boundary)
    ax.plot([parking.start.x, parking.start.x + parking.length], 
            [parking.start.y, parking.start.y], 'k-', linewidth=3, label='Bottom Curb')
    
    # Left curb
    ax.plot([parking.start.x, parking.start.x], 
            [parking.start.y, parking.start.y + parking.width], 'k-', linewidth=3, label='Left Curb')
    
    # Right curb
    ax.plot([parking.start.x + parking.length, parking.start.x + parking.length], 
            [parking.start.y, parking.start.y + parking.width], 'k-', linewidth=3, label='Right Curb')
    
    # Top curb (road boundary)
    ax.plot([parking.start.x - parking.baseline_left, parking.start.x + parking.length + parking.baseline_right], 
            [parking.start.y + parking.width + parking.road_width, parking.start.y + parking.width + parking.road_width], 
            'k-', linewidth=3, label='Road Boundary')
    
    # Extended baseline segments
    ax.plot([parking.start.x - parking.baseline_left, parking.start.x - parking.baseline_left], 
            [parking.start.y, parking.start.y + parking.width + parking.road_width], 'k--', linewidth=2, alpha=0.7)
    ax.plot([parking.start.x + parking.length + parking.baseline_right, parking.start.x + parking.length + parking.baseline_right], 
            [parking.start.y, parking.start.y + parking.width + parking.road_width], 'k--', linewidth=2, alpha=0.7)
    
    # Draw car as rectangle
    car_corners = car_state.get_corners()
    car_x = [corner.x for corner in car_corners]
    car_y = [corner.y for corner in car_corners]
    
    # Create car rectangle
    car_rect = patches.Polygon(list(zip(car_x, car_y)), 
                              facecolor='lightblue', 
                              edgecolor='blue', 
                              linewidth=2,
                              alpha=0.8)
    ax.add_patch(car_rect)
    
    # Draw rear axle point
    rear_axle = car_state.rear_axle_point()
    ax.plot(rear_axle.x, rear_axle.y, 'ro', markersize=8, label='Rear Axle')
    
    # Add car center point
    ax.plot(car_state.center.x, car_state.center.y, 'go', markersize=6, label='Car Center')
    
    # Add dimensions
    # Parking space dimensions
    ax.annotate(f'{parking.length}m', 
                xy=(parking.start.x + parking.length/2, parking.start.y - 0.5),
                xytext=(parking.start.x + parking.length/2, parking.start.y - 0.5),
                ha='center', va='top', fontsize=12, fontweight='bold',
                bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
    
    ax.annotate(f'{parking.width}m', 
                xy=(parking.start.x - 0.5, parking.start.y + parking.width/2),
                xytext=(parking.start.x - 0.5, parking.start.y + parking.width/2),
                ha='right', va='center', fontsize=12, fontweight='bold',
                bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
    
    # Car dimensions
    ax.annotate(f'{car_specs.length}m', 
                xy=(car_state.center.x, car_state.center.y + car_specs.width/2 + 0.5),
                xytext=(car_state.center.x, car_state.center.y + car_specs.width/2 + 0.5),
                ha='center', va='bottom', fontsize=10, fontweight='bold',
                bbox=dict(boxstyle="round,pad=0.2", facecolor="lightblue", alpha=0.8))
    
    ax.annotate(f'{car_specs.width}m', 
                xy=(car_state.center.x - car_specs.length/2 - 0.3, car_state.center.y),
                xytext=(car_state.center.x - car_specs.length/2 - 0.3, car_state.center.y),
                ha='right', va='center', fontsize=10, fontweight='bold',
                bbox=dict(boxstyle="round,pad=0.2", facecolor="lightblue", alpha=0.8))
    
    # Add labels
    ax.text(car_state.center.x, car_state.center.y + car_specs.width/2 + 1.5, 
            'Car', ha='center', va='bottom', fontsize=14, fontweight='bold',
            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.9))
    
    ax.text(parking.start.x + parking.length/2, parking.start.y + parking.width/2, 
            'Parking Space', ha='center', va='center', fontsize=14, fontweight='bold',
            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.9))
    
    # Set plot properties
    ax.set_xlim(-12, 8)
    ax.set_ylim(-2, 8)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (meters)', fontsize=12)
    ax.set_ylabel('Y (meters)', fontsize=12)
    ax.set_title('Parking Space and Car Geometry', fontsize=16, fontweight='bold')
    
    # Add legend
    ax.legend(loc='upper right', fontsize=10)
    
    # Save the plot
    plt.tight_layout()
    plt.savefig('docs/images/parking_diagram.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    print("Parking diagram saved to: docs/images/parking_diagram.png")

if __name__ == "__main__":
    create_parking_diagram() 