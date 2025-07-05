from __future__ import annotations
"""
One-sided breadth-/best-first parking planner.
"""

import heapq
from collections import deque
from dataclasses import dataclass
from math import hypot, pi
from typing import Callable, Deque, Dict, Iterable, List, Optional, Tuple

import numpy as np

from ..core.action import Action
from .bfs_base import _ParkingSearchBase
from ..core.car import CarState, CollisionException
from ..core.direction import Direction
from ..core.node import Node
from ..core.parking_space import ParkingSpace
from ..core.path import Path

DistanceHeuristic = Callable[[CarState, ParkingSpace, CarState], Tuple[float, ...]]


# --------------------------------------------------------------------- #
# configuration
# --------------------------------------------------------------------- #
@dataclass
class ExpansionConfig:
    step_resolution: Optional[float] = None
    step_angle_resolution: Optional[float] = None
    max_straight: float = 20.0
    max_radius: Optional[float] = None
    max_radius_count: int = 10
    depth: int = 4
    max_turn_sweep: float = 2 * pi
    max_dist_ratio: float = 1.5
    max_expansions: int = 100_000
    heuristic: Optional[DistanceHeuristic] = None


# --------------------------------------------------------------------- #
# planner
# --------------------------------------------------------------------- #
class BFSParking(_ParkingSearchBase):
    """Depth-bounded breadth-/best-first (single-front) parking planner."""

    # ───────────────────────────────────────────────────────────────────── #
    # shared helpers
    # ───────────────────────────────────────────────────────────────────── #
    @staticmethod
    def _build_radius_grid(min_r: float, max_r: float, count: int) -> List[float]:
        if count <= 1 or max_r <= min_r:
            return [min_r]
        return list(np.geomspace(min_r, max_r, num=count))

    @staticmethod
    def _expand_generic(
        state: CarState,
        *,
        step_dist: float,
        step_angle: float,
        max_straight: float,
        max_turn_sweep: float,
        radii: List[float],
        parking_space: ParkingSpace,
    ) -> List[Tuple[CarState, Action]]:
        out: List[Tuple[CarState, Action]] = []

        # straight primitives
        for direction in (1, -1):
            delta = step_dist * direction
            travelled = 0.0
            while abs(travelled) < max_straight:
                try:
                    nxt = state.straight(delta, None, parking_space)
                except CollisionException:
                    break
                out.append((nxt, Action("straight", delta)))
                travelled += step_dist
                delta += step_dist * direction

        # rotate primitives
        for radius in radii:
            for turn_dir in (Direction.LEFT, Direction.RIGHT):
                for sign in (-1, 1):
                    k = 1
                    while abs(k * step_angle) <= max_turn_sweep:
                        angle = k * step_angle * sign
                        try:
                            nxt = state.rotate(angle, radius, turn_dir, None, parking_space)
                        except CollisionException:
                            break
                        out.append((nxt, Action("rotate", angle, turn_dir, radius)))
                        k += 1
        return out

    # ───────────────────────────────────────────────────────────────────── #
    # constructor
    # ───────────────────────────────────────────────────────────────────── #
    def __init__(
        self,
        start_state: CarState,
        goal_states: CarState | Iterable[CarState],
        parking_space: ParkingSpace,
        *,
        expansion_config: ExpansionConfig = ExpansionConfig(),
        heuristic: Optional[DistanceHeuristic] = None,
        grid_resolution: float = 0.25,
        angle_resolution: float = np.deg2rad(5),
        step_radius_resolution: float = 1.0,
    ) -> None:
        # choose heuristic: explicit arg overrides config
        chosen_h = heuristic if heuristic is not None else expansion_config.heuristic

        super().__init__(
            start_state,
            goal_states,
            parking_space,
            heuristic=chosen_h,
            grid_resolution=grid_resolution,
            angle_resolution=angle_resolution,
            max_depth=expansion_config.depth,
        )

        # primitive step sizes
        self.step_dist = expansion_config.step_resolution or grid_resolution
        self.step_angle = expansion_config.step_angle_resolution or angle_resolution
        self.max_straight = expansion_config.max_straight
        self.max_turn_sweep = expansion_config.max_turn_sweep

        # radius grid
        min_r = start_state.specs.minimum_turning_radius
        max_r = (
            expansion_config.max_radius
            if expansion_config.max_radius is not None
            else min_r + 4 * step_radius_resolution
        )
        self.radii = self._build_radius_grid(
            min_r, max_r, expansion_config.max_radius_count
        )

        # pruning threshold
        self.initial_goal_dist = min(
            hypot(start_state.center.x - g.center.x, start_state.center.y - g.center.y)
            for g in self.goal_states
        )
        self.max_allowable_dist = self.initial_goal_dist * expansion_config.max_dist_ratio

        # bookkeeping
        self.parents: Dict[Tuple[int, int, int], Node] = {}
        self.visited: set[Tuple[int, int, int]] = set()
        self.max_expansions = expansion_config.max_expansions

    # node expansion
    def _expand(self, state: CarState):
        return self._expand_generic(
            state,
            step_dist=self.step_dist,
            step_angle=self.step_angle,
            max_straight=self.max_straight,
            max_turn_sweep=self.max_turn_sweep,
            radii=self.radii,
            parking_space=self.parking_space,
        )

    # ───────────────────────────────────────────────────────────────────── #
    # search loop
    # ───────────────────────────────────────────────────────────────────── #
    def search(
        self, *, progress_interval: int = 1000
    ) -> Optional[Tuple[Path, List[Action]]]:
        # choose queue type
        if self.heuristic is None:
            q: Deque[Node] = deque()
            push, pop = q.append, q.popleft
        else:
            q: List[Tuple[Tuple[float, ...], int, Node]] = []
            counter = 0

            def push(n: Node):
                nonlocal counter
                heapq.heappush(q, (n.key, counter, n))
                counter += 1

            pop = lambda: heapq.heappop(q)[-1]  # noqa: E731

        # seed root node
        root = Node(
            self.start_state,
            parent_hash=None,
            action=None,
            depth=0,
            key=self._make_key(self.start_state),
            dist=self._dist(self.start_state),
        )
        push(root)
        h0 = self._state_hash(self.start_state)
        self.visited.add(h0)
        self.parents[h0] = root
        best_node = root

        # logging
        print("Depth 0 - frontier size = 1")
        expansions = 0
        cur_depth = 0

        # main loop
        while q:
            node = pop()
            expansions += 1

            if expansions >= self.max_expansions:
                print(
                    "Expansion budget exhausted - returning best-so-far."
                )
                return self._reconstruct(best_node)

            if node.depth > cur_depth:
                cur_depth = node.depth
                print(f"Depth {cur_depth} - frontier size = {len(q)+1}")

            if self._is_goal(node.state):
                return self._reconstruct(node)

            if node.key < best_node.key:
                best_node = node

            if progress_interval and expansions % progress_interval == 0:
                print(
                    f"[{expansions}] visited={len(self.visited)} best_key={best_node.key} "
                    f"dist={best_node.dist:.2f} depth={best_node.depth}"
                )

            if node.depth >= self.max_depth:
                continue

            for child_state, act in self._expand(node.state):
                if self._dist(child_state) > self.max_allowable_dist:
                    continue
                h = self._state_hash(child_state)
                if h in self.visited:
                    continue
                self.visited.add(h)
                child = Node(
                    child_state,
                    parent_hash=self._state_hash(node.state),
                    action=act,
                    depth=node.depth + 1,
                    key=self._make_key(child_state),
                    dist=self._dist(child_state),
                )
                push(child)
                self.parents[h] = child

        print("No exact goal - returning closest candidate.")
        return self._reconstruct(best_node)

    # ───────────────────────────────────────────────────────────────────── #
    # reconstruction - now uses apply_action
    # ───────────────────────────────────────────────────────────────────── #
    def _reconstruct(self, node: Node) -> Tuple[Path, List[Action]]:
        actions: List[Action] = []
        n = node
        while n.parent_hash is not None:
            actions.append(n.action)  # type: ignore[arg-type]
            n = self.parents[n.parent_hash]
        actions.reverse()

        path = Path()
        state = self.start_state
        for act in actions:
            state = state.apply_action(act, path=path, parking_space=self.parking_space)
        return path, actions