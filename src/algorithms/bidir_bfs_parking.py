from __future__ import annotations
"""
Bidirectional BFS parking planner with per-front heuristics, budgets and logs.
"""

from collections import deque
from typing import Deque, Dict, Iterable, List, Optional, Tuple

import numpy as np

from .bfs_parking import BFSParking, ExpansionConfig, DistanceHeuristic
from ..core.action import Action
from ..core.car import CarState
from ..core.node import Node
from ..core.parking_space import ParkingSpace
from ..core.path import Path


class BidirBFSParking(BFSParking):
    """
    Meet-in-the-middle BFS.

    • `config_start` drives the forward tree; `config_goal` drives the reverse tree.
    • Each config may have its own heuristic, depth and expansion budget.
    • The planner logs depth changes per front and prints a progress message every
      `progress_interval` node pops.
    """

    # ───────────────────────────────────────── constructor ───────────────────────────────────────── #
    def __init__(
        self,
        start_state: CarState,
        goal_states: CarState | Iterable[CarState],
        parking_space: ParkingSpace,
        *,
        config_start: ExpansionConfig = ExpansionConfig(depth=3),
        config_goal:  ExpansionConfig = ExpansionConfig(depth=3),
        grid_resolution: float = 0.25,
        angle_resolution: float = np.deg2rad(5),
        step_radius_resolution: float = 1.0,
    ) -> None:
        # forward-front initialisation
        super().__init__(
            start_state,
            goal_states,
            parking_space,
            expansion_config=config_start,
            heuristic=config_start.heuristic,
            grid_resolution=grid_resolution,
            angle_resolution=angle_resolution,
            step_radius_resolution=step_radius_resolution,
        )

        # reverse-front configuration
        self.cfg_goal        = config_goal
        self.heuristic_goal  = config_goal.heuristic

        self.depth_start     = config_start.depth
        self.depth_goal      = config_goal.depth
        self.max_fwd_exp     = config_start.max_expansions
        self.max_rev_exp     = config_goal.max_expansions

        self.goal_step_dist   = config_goal.step_resolution or grid_resolution
        self.goal_step_angle  = config_goal.step_angle_resolution or angle_resolution
        self.goal_max_straight= config_goal.max_straight
        self.goal_max_turn_sweep = config_goal.max_turn_sweep

        # goal-side radius grid — fallback uses goal's own step_resolution
        min_r = start_state.specs.minimum_turning_radius
        fallback_step = config_goal.step_resolution or grid_resolution
        max_r = (
            config_goal.max_radius
            if config_goal.max_radius is not None
            else min_r + 4 * fallback_step
        )
        self.goal_radii = self._build_radius_grid(
            min_r, max_r, config_goal.max_radius_count
        )

        # Metadata storage for search results
        self.visited_states_fwd = 0
        self.visited_states_rev = 0
        self.total_visited_states = 0
        self.pops_fwd = 0
        self.pops_rev = 0
        self.total_pops = 0

    # ───────────────────────────────────────── helper keys ───────────────────────────────────────── #
    def _make_key_start(self, s: CarState) -> Tuple[float, ...]:
        return self._make_key(s)                     # uses self.heuristic

    def _make_key_goal(self, s: CarState) -> Tuple[float, ...]:
        if self.heuristic_goal is None:
            return self._distance_heading_error(s)
        return min(self.heuristic_goal(s, self.parking_space, g) for g in self.goal_states)

    # ───────────────────────────────────────── goal-side expander ───────────────────────────────────────── #
    def _expand_goal(self, state: CarState):
        return self._expand_generic(
            state,
            step_dist=self.goal_step_dist,
            step_angle=self.goal_step_angle,
            max_straight=self.goal_max_straight,
            max_turn_sweep=self.goal_max_turn_sweep,
            radii=self.goal_radii,
            parking_space=self.parking_space,
        )

    # ───────────────────────────────────────── search ───────────────────────────────────────── #
    def search(
        self, *, progress_interval: int = 1000
    ) -> Optional[Tuple[Path, List[Action]]]:

        # ---------- seed forward ----------
        fwd_q: Deque[Node] = deque()
        h0 = self._state_hash(self.start_state)
        root_fwd = Node(
            self.start_state, None, None, 0,
            key=self._make_key_start(self.start_state), dist=0.0
        )
        fwd_q.append(root_fwd)
        parents_fwd: Dict[Tuple[int, int, int], Node] = {h0: root_fwd}
        visited_fwd = {h0}

        # ---------- seed reverse ----------
        rev_q: Deque[Node] = deque()
        parents_rev: Dict[Tuple[int, int, int], Node] = {}
        visited_rev: set[Tuple[int, int, int]] = set()
        for g in self.goal_states:
            h = self._state_hash(g)
            n = Node(g, None, None, 0,
                     key=self._make_key_goal(g), dist=0.0)
            rev_q.append(n)
            parents_rev[h] = n
            visited_rev.add(h)

        pops_fwd = pops_rev = 0
        last_depth_fwd = last_depth_rev = 0
        print(f"Bidirectional BFS (depth FWD={self.depth_start}  REV={self.depth_goal})")

        # ---------- main loop ----------
        while True:
            # choose which frontier to expand
            use_fwd = (
                pops_fwd < self.max_fwd_exp and fwd_q and
                (not rev_q or pops_rev >= self.max_rev_exp or len(fwd_q) <= len(rev_q))
            )
            if not use_fwd and not rev_q:
                print("No connection found (queues empty).")
                # Store metadata before returning
                self.visited_states_fwd = len(visited_fwd)
                self.visited_states_rev = len(visited_rev)
                self.total_visited_states = len(visited_fwd) + len(visited_rev)
                self.pops_fwd = pops_fwd
                self.pops_rev = pops_rev
                self.total_pops = pops_fwd + pops_rev
                return None

            # pop node
            if use_fwd:
                node = fwd_q.popleft()
                pops_fwd += 1
                if node.depth > last_depth_fwd:
                    last_depth_fwd = node.depth
                    print(f"[FWD] depth={node.depth}  queue={len(fwd_q)}")
                max_depth, expander, key_fn = (
                    self.depth_start, self._expand, self._make_key_start
                )
                q_here, parents_here, visited_here = fwd_q, parents_fwd, visited_fwd
                visited_other, parents_other = visited_rev, parents_rev
            else:
                node = rev_q.popleft()
                pops_rev += 1
                if node.depth > last_depth_rev:
                    last_depth_rev = node.depth
                    print(f"[REV] depth={node.depth}  queue={len(rev_q)}")
                max_depth, expander, key_fn = (
                    self.depth_goal, self._expand_goal, self._make_key_goal
                )
                q_here, parents_here, visited_here = rev_q, parents_rev, visited_rev
                visited_other, parents_other = visited_fwd, parents_fwd

            # periodic log
            total_pops = pops_fwd + pops_rev
            if progress_interval and total_pops % progress_interval == 0:
                print(f"[{total_pops}] fwd_visited={len(visited_fwd)}  rev_visited={len(visited_rev)}")

            # depth limit
            if node.depth >= max_depth:
                continue

            # expand current node
            for nxt, act in expander(node.state):
                h = self._state_hash(nxt)
                if h in visited_here:
                    continue
                visited_here.add(h)
                child = Node(
                    nxt, self._state_hash(node.state), act, node.depth + 1,
                    key=key_fn(nxt), dist=0.0
                )
                parents_here[h] = child
                q_here.append(child)

                # check for connection
                if h in visited_other:
                    print(h)
                    print(f"✓ meet after pops  FWD={pops_fwd}  REV={pops_rev}")
                    # Store metadata before returning
                    self.visited_states_fwd = len(visited_fwd)
                    self.visited_states_rev = len(visited_rev)
                    self.total_visited_states = len(visited_fwd) + len(visited_rev)
                    self.pops_fwd = pops_fwd
                    self.pops_rev = pops_rev
                    self.total_pops = pops_fwd + pops_rev
                    return self._stitch(h, parents_fwd, parents_rev)

            # budget exhaustion
            done_fwd = not fwd_q or pops_fwd >= self.max_fwd_exp
            done_rev = not rev_q or pops_rev >= self.max_rev_exp
            if done_fwd and done_rev:
                print("No connection found (budgets exhausted).")
                # Store metadata before returning
                self.visited_states_fwd = len(visited_fwd)
                self.visited_states_rev = len(visited_rev)
                self.total_visited_states = len(visited_fwd) + len(visited_rev)
                self.pops_fwd = pops_fwd
                self.pops_rev = pops_rev
                self.total_pops = pops_fwd + pops_rev
                return None

    # ───────────────────────────────────────── stitch ───────────────────────────────────────── #
    def _stitch(
        self,
        meet_hash: Tuple[int, int, int],
        parents_start: Dict[Tuple[int, int, int], Node],
        parents_goal:  Dict[Tuple[int, int, int], Node],
    ) -> Tuple[Path, List[Action]]:

        # -------- forward half (start → meet) --------
        fwd: List[Action] = []
        n = parents_start[meet_hash]
        while n.parent_hash is not None:
            fwd.append(n.action)                       # type: ignore[arg-type]
            n = parents_start[n.parent_hash]
        fwd.reverse()

        # -------- reverse half (meet → goal) --------
        rev: List[Action] = []
        n = parents_goal[meet_hash]
        while n.parent_hash is not None:
            rev.append(n.action.inverse())             # type: ignore[arg-type]
            n = parents_goal[n.parent_hash]
        # (no reverse() here – rev is already in meet→goal order)

        plan = fwd + rev
        self.last_split = len(fwd)                     # record index of meeting point

        # rebuild full path for caller
        path = Path()
        state = self.start_state
        for act in plan:
            state = state.apply_action(act, path=path, parking_space=self.parking_space)

        return path, plan