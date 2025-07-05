from dataclasses import dataclass
from typing import Optional, Tuple

from .car import CarState
from .action import Action


@dataclass
class Node:
    state: CarState
    parent_hash: Optional[Tuple[int, int, int]]
    action: Optional[Action]
    depth: int
    # NEW fields — automatically filled when each node is created
    key: Tuple[float, ...]          # queue / heuristic key
    dist: float                     # distance to nearest goal