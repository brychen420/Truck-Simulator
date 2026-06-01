import math
from dataclasses import dataclass
from config import TruckConfig


@dataclass
class TruckTrailerState:
    xR:   float   # truck rear axle X (m)
    yR:   float   # truck rear axle Y (m)
    psi1: float   # truck yaw (rad)
    xT:   float   # trailer axle X (m)
    yT:   float   # trailer axle Y (m)
    psi2: float   # trailer yaw (rad)

    @property
    def hitch_angle(self) -> float:
        """Hitch angle Δψ = ψ₁ - ψ₂, normalized to [-π, π]."""
        a = self.psi1 - self.psi2
        return (a + math.pi) % (2 * math.pi) - math.pi

    @property
    def hitch_angle_deg(self) -> float:
        return math.degrees(self.hitch_angle)


def _normalize(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def initial_state(cfg: TruckConfig, psi1_init: float = 0.0) -> TruckTrailerState:
    """Create initial state with truck and trailer aligned (Δψ = 0)."""
    xR, yR = 0.0, 0.0
    psi1 = psi1_init
    # Hitch point H is LH behind rear axle
    xH = xR - cfg.LH * math.cos(psi1)
    yH = yR - cfg.LH * math.sin(psi1)
    # Trailer axle is LT behind H (same heading as truck initially)
    xT = xH - cfg.LT * math.cos(psi1)
    yT = yH - cfg.LT * math.sin(psi1)
    return TruckTrailerState(xR, yR, psi1, xT, yT, psi1)


def hitch_point(state: TruckTrailerState, cfg: TruckConfig) -> tuple:
    """Hitch point H computed from truck side."""
    xH = state.xR - cfg.LH * math.cos(state.psi1)
    yH = state.yR - cfg.LH * math.sin(state.psi1)
    return xH, yH


class TruckTrailerKinematics:
    """Forward integrator using paper equations (1)-(6)."""

    def __init__(self, cfg: TruckConfig):
        self.cfg = cfg

    def _derivatives(self, state: TruckTrailerState, delta_f: float, vR: float) -> dict:
        cfg = self.cfg
        dpsi = state.psi1 - state.psi2          # Δψ
        tan_df = math.tan(delta_f)
        # Scalar factor shared by ẋT and ẏT (equations 4-5)
        factor = math.cos(dpsi) + (cfg.LH / cfg.L) * math.sin(dpsi) * tan_df
        return {
            # Truck (equations 1-3)
            'dxR':   vR * math.cos(state.psi1),
            'dyR':   vR * math.sin(state.psi1),
            'dpsi1': vR / cfg.L * tan_df,
            # Trailer (equations 4-6)
            'dxT':   vR * math.cos(state.psi2) * factor,
            'dyT':   vR * math.sin(state.psi2) * factor,
            'dpsi2': vR / cfg.LT * (math.sin(dpsi) - (cfg.LH / cfg.L) * math.cos(dpsi) * tan_df),
        }

    def _apply(self, state: TruckTrailerState, d: dict, dt: float) -> TruckTrailerState:
        return TruckTrailerState(
            xR=   state.xR   + d['dxR']   * dt,
            yR=   state.yR   + d['dyR']   * dt,
            psi1= _normalize(state.psi1   + d['dpsi1'] * dt),
            xT=   state.xT   + d['dxT']   * dt,
            yT=   state.yT   + d['dyT']   * dt,
            psi2= _normalize(state.psi2   + d['dpsi2'] * dt),
        )

    def step(self, state: TruckTrailerState, delta_f: float, vR: float, dt: float) -> TruckTrailerState:
        """Euler integration, one step of dt."""
        cfg = self.cfg
        delta_f = max(-cfg.max_steer, min(cfg.max_steer, delta_f))
        if abs(vR) < 0.01:
            vR = 0.0
        d = self._derivatives(state, delta_f, vR)
        return self._apply(state, d, dt)

    def step_rk4(self, state: TruckTrailerState, delta_f: float, vR: float, dt: float) -> TruckTrailerState:
        """RK4 integration, more accurate for high-speed scenarios."""
        cfg = self.cfg
        delta_f = max(-cfg.max_steer, min(cfg.max_steer, delta_f))
        if abs(vR) < 0.01:
            vR = 0.0
        k1 = self._derivatives(state, delta_f, vR)
        s2 = self._apply(state, k1, dt / 2)
        k2 = self._derivatives(s2, delta_f, vR)
        s3 = self._apply(state, k2, dt / 2)
        k3 = self._derivatives(s3, delta_f, vR)
        s4 = self._apply(state, k3, dt)
        k4 = self._derivatives(s4, delta_f, vR)
        combined = {k: (k1[k] + 2 * k2[k] + 2 * k3[k] + k4[k]) / 6 for k in k1}
        return self._apply(state, combined, dt)
