from dataclasses import dataclass


@dataclass
class TruckConfig:
    L:             float = 2.896   # wheelbase (m)
    LH:            float = 1.159   # rear axle to hitch (m)
    LT:            float = 2.693   # trailer axle to hitch (m)
    truck_length:  float = 5.046   # m
    truck_width:   float = 1.935   # m
    trailer_length: float = 3.84   # m
    trailer_width:  float = 1.63   # m
    max_steer:     float = 0.75    # rad (~43 deg)
    max_speed:     float = 8.0     # m/s
    accel:         float = 3.0     # m/s^2
    steer_rate:    float = 1.2     # rad/s
    friction:      float = 2.0     # speed decay coefficient
    steer_return:  float = 5.0     # steer self-center coefficient
    speed_deadzone: float = 0.05
    steer_deadzone: float = 0.01


@dataclass
class SimConfig:
    fps:               int   = 60
    pixels_per_m:      float = 40.0
    grid_spacing:      float = 5.0     # m
    jackknife_warn_deg:  float = 60.0
    jackknife_limit_deg: float = 85.0
    window_w: int = 1280
    window_h: int = 720
