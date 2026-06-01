import pygame
from pygame.locals import K_w, K_s, K_a, K_d
from config import TruckConfig


class InputHandler:
    """Convert pygame key state to continuous (δf, VR) with inertia."""

    def __init__(self, cfg: TruckConfig):
        self.cfg = cfg
        self._speed = 0.0   # current speed with inertia
        self._steer = 0.0   # current steer angle with inertia

    def update(self, keys, dt: float) -> tuple:
        """Return (delta_f, vR) for this frame."""
        cfg = self.cfg

        # Throttle: W accelerates forward, S accelerates backward
        if keys[K_w]:
            self._speed = min(self._speed + cfg.accel * dt, cfg.max_speed)
        elif keys[K_s]:
            self._speed = max(self._speed - cfg.accel * dt, -cfg.max_speed)
        else:
            # Coast to stop (friction)
            self._speed *= (1.0 - cfg.friction * dt)
            if abs(self._speed) < cfg.speed_deadzone:
                self._speed = 0.0

        # Steering: A = left (positive δf = CCW), D = right (negative δf = CW)
        # Positive δf → ψ̇₁ = vR/L * tan(δf) > 0 → counterclockwise → left turn
        if keys[K_a]:
            self._steer = min(self._steer + cfg.steer_rate * dt, cfg.max_steer)
        elif keys[K_d]:
            self._steer = max(self._steer - cfg.steer_rate * dt, -cfg.max_steer)
        else:
            # Auto-center (simulates steering return torque)
            self._steer *= (1.0 - cfg.steer_return * dt)
            if abs(self._steer) < cfg.steer_deadzone:
                self._steer = 0.0

        return self._steer, self._speed

    def reset(self):
        self._speed = 0.0
        self._steer = 0.0

    @property
    def speed(self) -> float:
        return self._speed

    @property
    def steer(self) -> float:
        return self._steer
