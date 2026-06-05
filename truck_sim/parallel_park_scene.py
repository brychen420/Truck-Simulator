"""Fixed parallel-parking scene (roadside parking).

The parking slot runs east-west (angle=0), bordered by a kerb (curb wall)
on the south side and flanked by two parked-car obstacles on each end.
The truck starts to the right (east) of the slot, heading east, in the
travel lane. It must reverse-parallel-park into the slot.
"""

import math
from config import TruckConfig
from parking import ParkingSpot
from autopark_scene import WallSegment, AutoParkScene


def build_parallel_scene(cfg: TruckConfig) -> AutoParkScene:
    """Construct the parallel-parking scene scaled to the given vehicle config."""
    total_chain = cfg.LH + cfg.LT
    full_len    = cfg.truck_length + total_chain + cfg.trailer_length
    slot_len    = full_len + 4.0
    slot_wid    = max(cfg.truck_width, cfg.trailer_width) + 2.0

    half_slot  = slot_len / 2
    kerb_y     = -(slot_wid / 2)
    # Parked-car walls cover only the kerb lane (up to the slot's north edge).
    # Keeping this below initial_yR (= slot_wid/2 + 1.5) ensures the start
    # state is collision-free.
    road_top_y = slot_wid / 2   # = north edge of the parking slot ≈ +1.97 m
    INF        = float('inf')

    spot = ParkingSpot(
        x=0.0, y=0.0,
        angle=0.0,        # horizontal (long axis along X) — distinguishes from perp scene
        length=slot_len,
        width=slot_wid,
    )

    # Goal: truck+trailer aligned east (psi=0), centred in slot Y,
    # truck front 1 m inside the right end of the slot.
    goal_yT = 0.0
    goal_xT = half_slot - 1.0 - cfg.truck_length - cfg.LT

    # Start: truck to the east of slot, heading east, in travel lane
    initial_psi1 = 0.0
    initial_xR   = half_slot + 8.0
    initial_yR   = slot_wid / 2 + 1.5

    walls = [
        # Kerb — vehicle must stay above (north of) this line
        WallSegment('y', 'min', kerb_y,              -INF,       INF),
        # Left parked-car face (east face of cars west of the slot)
        WallSegment('x', 'min', -(half_slot + 0.5),  kerb_y,     road_top_y),
        # Right parked-car face (west face of cars east of the slot)
        WallSegment('x', 'max', +(half_slot + 0.5),  kerb_y,     road_top_y),
        # Road top — prevent planner from escaping north
        WallSegment('y', 'max',  20.0,               -INF,       INF),
        # Far east/west limits
        WallSegment('x', 'min', -60.0,               -INF,       INF),
        WallSegment('x', 'max',  60.0,               -INF,       INF),
    ]

    return AutoParkScene(
        spot=spot,
        walls=walls,
        back_wall_y=kerb_y,   # repurposed to hold the south boundary for the renderer
        goal_xT=goal_xT,
        goal_yT=goal_yT,
        goal_psi2=0.0,        # trailer heading east
        goal_dpsi=0.0,        # truck & trailer aligned
        initial_xR=initial_xR,
        initial_yR=initial_yR,
        initial_psi1=initial_psi1,
    )
