# Car Parking Simulation Package

# Core exports
from .core.car import CarSpecs, CarState, CollisionException
from .core.parking_space import ParkingSpace
from .core.path import Path, ArcPart, StraightPart
from .core.point import Point
from .core.action import Action
from .core.direction import Direction
from .core.node import Node

# Algorithm exports
from .algorithms.bfs_parking import BFSParking, ExpansionConfig
from .algorithms.bidir_bfs_parking import BidirBFSParking
# from .algorithms.parallel_parking import parallel_park  # No function exported
from .algorithms.heuristic import (
    distance_from_curb,
    distance_from_top_curb,
    line_distance,
    line_then_curb
)

__all__ = [
    # Core
    'CarSpecs', 'CarState', 'CollisionException',
    'ParkingSpace', 'Path', 'ArcPart', 'StraightPart',
    'Point', 'Action', 'Direction', 'Node',
    # Algorithms
    'BFSParking', 'ExpansionConfig', 'BidirBFSParking',
    # 'parallel_park',  # No function exported
    # Heuristics
    'distance_from_curb', 'distance_from_top_curb',
    'line_distance', 'line_then_curb'
]
