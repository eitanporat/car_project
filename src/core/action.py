# action.py
from typing import NamedTuple, Optional
from .direction import Direction

class Action(NamedTuple):
    kind: str
    magnitude: float
    direction: Optional[Direction] = None
    radius:    Optional[float]    = None

    def inverse(self) -> "Action":
        if self.kind == "straight":
            return Action("straight", -self.magnitude)
        else:  # rotate
            return Action("rotate", -self.magnitude,
                          self.direction, self.radius)