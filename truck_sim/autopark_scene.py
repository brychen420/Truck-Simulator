"""Fixed perpendicular reverse-parking scene.

All geometry is derived from TruckConfig so it scales with vehicle parameters.
The truck starts heading right (psi1=0) at (-10, 8).
The parking slot opens upward at y≈0 and extends downward.
"""

import math
from dataclasses import dataclass, field
from config import TruckConfig
from parking import ParkingSpot


@dataclass
class WallSegment:
    """An axis-aligned half-plane wall.

    axis  : 'x' or 'y'  — which axis the boundary is along
    side  : 'min' → forbidden zone is on the LOW side (point < value)
            'max' → forbidden zone is on the HIGH side (point > value)
    value : the boundary coordinate
    active_min / active_max : range on the OTHER axis where this wall is active
                              (use -inf / +inf for unlimited)
    """
    axis:       str
    side:       str
    value:      float
    active_min: float
    active_max: float

    def point_outside(self, x: float, y: float, margin: float = 0.0) -> bool:
        """True if (x, y) is on the forbidden side of this wall (within margin)."""
        coord  = x if self.axis == 'x' else y
        other  = y if self.axis == 'x' else x
        if not (self.active_min <= other <= self.active_max):
            return False
        if self.side == 'min':
            return coord < self.value + margin
        return coord > self.value - margin


@dataclass
class AutoParkScene:
    spot:          ParkingSpot
    walls:         list           # list[WallSegment]
    back_wall_y:   float          # y value of the back wall (for rendering)
    # Goal: trailer axle target for the planner
    goal_xT:       float
    goal_yT:       float
    goal_psi2:     float          # +π/2  (trailer heading upward / north)
    goal_dpsi:     float          # 0.0   (truck & trailer aligned)
    # Initial truck pose
    initial_xR:    float          # -10.0
    initial_yR:    float          #   8.0
    initial_psi1:  float          #   0.0  (heading right)

    def point_in_collision(self, x: float, y: float, margin: float = 0.0) -> bool:
        """True if world point (x, y) violates any wall (within margin)."""
        return any(w.point_outside(x, y, margin) for w in self.walls)


def build_scene(cfg: TruckConfig) -> AutoParkScene:
    """Construct the auto-park scene scaled to the given vehicle config."""
    spot_len = cfg.truck_length + cfg.trailer_length + 2.5   # long axis  ≈ 11.39 m
    spot_wid = max(cfg.truck_width, cfg.trailer_width) + 2.5  # short axis ≈  4.44 m

    # Slot is vertical (angle = π/2), centred at x=0, opening faces y=0
    spot_y_c = -(spot_len / 2) - 1.0        # centre y  ≈ -6.69 m

    spot = ParkingSpot(
        x=0.0, y=spot_y_c,
        angle=math.pi / 2,
        length=spot_len,
        width=spot_wid,
    )

    # Goal: entire vehicle reversed into slot with psi2=+π/2 (trailer pointing north).
    # In this pose the trailer TAIL is the deepest point (southernmost).
    # We position the truck front 1 m below the slot entrance (y = slot_top - 1 = -2):
    #   truck_front_y = slot_top - 1.0 = -1.0 - 1.0 = -2.0
    #   goal_yT = truck_front_y - (LT + truck_length)
    goal_xT   = 0.0
    goal_yT   = -2.0 - cfg.LT - cfg.truck_length   # trailer axle  ≈ -9.74 m

    hw = spot_wid / 2
    INF = float('inf')

    # Deepest point at goal pose (psi2=+π/2, dpsi=0) is the trailer TAIL:
    #   trailer_tail_y = goal_yT + LT - trailer_length
    # Back wall sits 1.5 m below that.
    vehicle_bottom_y = goal_yT + cfg.LT - cfg.trailer_length
    back_wall_y      = vehicle_bottom_y - 1.5   # aligns with spot bottom

    walls = [
        # ── Alley walls (active only below the road, y < 0) ──────────────
        WallSegment('x', 'min', -(hw + 0.5), -INF,  0.0),   # left
        WallSegment('x', 'max', +(hw + 0.5), -INF,  0.0),   # right
        # ── Back wall (placed below deepest parked truck point) ───────────
        WallSegment('y', 'min', back_wall_y,  -INF, INF),
        # ── Road outer boundary (prevent planner from escaping) ───────────
        WallSegment('y', 'max',  20.0,  -INF, INF),
        WallSegment('x', 'min', -40.0,  -INF, INF),
        WallSegment('x', 'max',  40.0,  -INF, INF),
    ]

    return AutoParkScene(
        spot=spot,
        walls=walls,
        back_wall_y=back_wall_y,
        goal_xT=goal_xT,
        goal_yT=goal_yT,
        goal_psi2=+math.pi / 2,
        goal_dpsi=0.0,
        initial_xR=-10.0,
        initial_yR=8.0,
        initial_psi1=0.0,
    )
