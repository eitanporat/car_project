# bfs_base.py
from __future__ import annotations

from abc import ABC, abstractmethod
from math import hypot, pi
from typing import Callable, Iterable, Optional, Tuple

from ..core.action import Action
from ..core.car import CarState
from ..core.parking_space import ParkingSpace
from ..core.path import Path


class _ParkingSearchBase(ABC):
    """
    Shared scaffolding for parking planners (BFS, A*, etc.).

    •  Discretises (x, y, θ) with `grid_resolution` and `angle_resolution`
       so states can be hashed and revisited-state checks are O(1).

    •  Pre-computes the hashes of all goal poses (`_goal_hashes`).

    Sub-classes must implement `search()`.
    """

    # ------------------------------------------------------------------
    # Constructor
    # ------------------------------------------------------------------
    def __init__(
        self,
        start_state: CarState,
        goal_states: CarState | Iterable[CarState],
        parking_space: ParkingSpace,
        *,
        heuristic: Optional[
            Callable[[CarState, ParkingSpace, CarState], Tuple[float, ...]]
        ],
        grid_resolution: float,
        angle_resolution: float,
        max_depth: int,
    ) -> None:
        self.start_state = start_state
        self.goal_states = (
            [goal_states] if isinstance(goal_states, CarState) else list(goal_states)
        )
        if not self.goal_states:
            raise ValueError("goal_states may not be empty")

        self.parking_space = parking_space
        self.grid_resolution = grid_resolution
        self.angle_resolution = angle_resolution
        self.max_depth = max_depth
        self.heuristic = heuristic

        # Pre-hash goal poses once; goal test can then be O(1)
        self._goal_hashes: set[Tuple[int, int, int]] = {
            self._state_hash(g) for g in self.goal_states
        }

    # ------------------------------------------------------------------
    # Helper: discretised state hash
    # ------------------------------------------------------------------
    def _state_hash(self, s: CarState) -> Tuple[int, int, int]:
        return (
            round(s.rear_axle_point().x / self.grid_resolution),
            round(s.rear_axle_point().y / self.grid_resolution),
            round((s.orientation % (2 * pi)) / self.angle_resolution),
        )

    # ------------------------------------------------------------------
    # Helper: (distance, heading-error) pair for tie-breaking / heuristics
    # ------------------------------------------------------------------
    def _distance_heading_error(self, s: CarState) -> Tuple[float, float]:
        best_d, best_a = float("inf"), float("inf")
        for g in self.goal_states:
            d = hypot(s.center.x - g.center.x, s.center.y - g.center.y)
            a = min(
                abs(s.orientation - g.orientation),
                2 * pi - abs(s.orientation - g.orientation),
            )
            if (d, a) < (best_d, best_a):
                best_d, best_a = d, a
        return best_d, best_a

    def _is_goal(self, s: CarState) -> bool:
        return self._state_hash(s) in self._goal_hashes

    def _make_key(self, s: CarState) -> Tuple[float, ...]:
        """
        • If no heuristic → (euclidean-distance, heading-error)  
        • If heuristic     → pick the best (lowest-lexicographic) tuple
          returned by `self.heuristic` against all goal poses.
        """
        if self.heuristic is None:
            return self._distance_heading_error(s)
        return min(
            self.heuristic(s, self.parking_space, g) for g in self.goal_states
        )

    def _dist(self, s: CarState) -> float:
        return min(
            hypot(s.center.x - g.center.x, s.center.y - g.center.y)
            for g in self.goal_states
        )

    @abstractmethod
    def search(self, **kwargs) -> Optional[Tuple[Path, list[Action]]]:
        ...