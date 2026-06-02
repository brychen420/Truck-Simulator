"""Parking challenge: spot geometry, spawning, and success detection."""

import math
import random
from dataclasses import dataclass
from config import TruckConfig
from kinematics import TruckTrailerState


def _rect_corners(cx, cy, yaw, length, width):
    hl, hw = length / 2, width / 2
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    return [
        (cx + x * cos_y - y * sin_y,
         cy + x * sin_y + y * cos_y)
        for x, y in [(-hl, -hw), (-hl, hw), (hl, hw), (hl, -hw)]
    ]


@dataclass
class ParkingSpot:
    x:      float   # centre X (m)
    y:      float   # centre Y (m)
    angle:  float   # 0.0 or π/2, aligned to world axes
    length: float   # long dimension (m)
    width:  float   # short dimension (m)

    def corners(self):
        return _rect_corners(self.x, self.y, self.angle, self.length, self.width)

    def contains_point(self, px: float, py: float) -> bool:
        dx, dy = px - self.x, py - self.y
        cos_a, sin_a = math.cos(self.angle), math.sin(self.angle)
        lx =  dx * cos_a + dy * sin_a
        ly = -dx * sin_a + dy * cos_a
        return abs(lx) <= self.length / 2 and abs(ly) <= self.width / 2

    def contains_polygon(self, corners) -> bool:
        return all(self.contains_point(x, y) for x, y in corners)


def spawn_spot(cfg: TruckConfig,
               ref_x: float = 0.0, ref_y: float = 0.0) -> ParkingSpot:
    """Generate a spot at a random distance (30-65 m), snapped to 5 m grid.

    Spot is aligned to 0° or 90° and sized to fit the whole truck+trailer
    with a small margin.
    """
    total_len = cfg.truck_length + cfg.trailer_length
    max_wid   = max(cfg.truck_width, cfg.trailer_width)

    spot_len = total_len + 2.5   # ~2.5 m buffer on length
    spot_wid = max_wid  + 1.5   # ~1.5 m buffer on width

    angle = random.choice([0.0, math.pi / 2])
    dist  = random.uniform(30, 65)
    theta = random.uniform(0, 2 * math.pi)

    # Snap centre to 5 m grid
    x = round((ref_x + dist * math.cos(theta)) / 5) * 5.0
    y = round((ref_y + dist * math.sin(theta)) / 5) * 5.0

    return ParkingSpot(x, y, angle, spot_len, spot_wid)


class ParkingManager:
    """Tracks the active parking spot, detects success, and respawns."""

    PARK_SPEED   = 0.5   # m/s  — max speed to count as parked
    SUCCESS_HOLD = 3.0   # s    — how long to show the success banner

    def __init__(self, cfg: TruckConfig, ref_x: float = 0.0, ref_y: float = 0.0):
        self.cfg           = cfg
        self.spot          = spawn_spot(cfg, ref_x, ref_y)
        self.success_count = 0
        self._parked       = False
        self._timer        = 0.0

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _all_corners(self, state: TruckTrailerState):
        """8 world-space corners of the full truck+trailer footprint."""
        cfg = self.cfg
        # Truck body
        tc_off = cfg.truck_length / 2 - cfg.LH
        tcx = state.xR + tc_off * math.cos(state.psi1)
        tcy = state.yR + tc_off * math.sin(state.psi1)
        # Trailer body
        tl_off = cfg.LT - cfg.trailer_length / 2
        tlx = state.xT + tl_off * math.cos(state.psi2)
        tly = state.yT + tl_off * math.sin(state.psi2)
        return (_rect_corners(tcx, tcy, state.psi1, cfg.truck_length, cfg.truck_width) +
                _rect_corners(tlx, tly, state.psi2, cfg.trailer_length, cfg.trailer_width))

    # ── Public API ────────────────────────────────────────────────────────────

    def update(self, state: TruckTrailerState, vR: float, dt: float) -> bool:
        """Call once per frame. Returns True the moment parking is detected."""
        if self._parked:
            self._timer -= dt
            if self._timer <= 0:
                self._parked = False
                self.spot = spawn_spot(self.cfg, state.xR, state.yR)
            return False

        if (self.spot.contains_polygon(self._all_corners(state))
                and abs(vR) <= self.PARK_SPEED):
            self._parked = True
            self._timer  = self.SUCCESS_HOLD
            self.success_count += 1
            return True
        return False

    @property
    def is_parked(self) -> bool:
        return self._parked

    @property
    def success_timer(self) -> float:
        return self._timer

    def distance_to(self, state: TruckTrailerState) -> float:
        return math.hypot(self.spot.x - state.xR, self.spot.y - state.yR)

    def vehicle_inside(self, state: TruckTrailerState) -> bool:
        """True when all 8 corners are inside the spot (regardless of speed)."""
        return self.spot.contains_polygon(self._all_corners(state))
