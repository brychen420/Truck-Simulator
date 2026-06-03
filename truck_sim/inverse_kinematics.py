"""Inverse kinematics for the truck-trailer system.

Converts a virtual trailer bicycle control (δT, VT) into the physical
truck front-wheel steering angle δf and rear-axle speed VR.

Derivation
----------
The forward kinematics trailer equation (paper eq. 6) is:

    ψ̇₂ = (VR / LT) · [sin(Δψ) − (LH/L) · cos(Δψ) · tan(δf)]

The virtual trailer bicycle model expresses the same motion as:

    ψ̇₂ = (VT / LT) · tan(δT)

Equating and using VT = VR · F  (F = cos(Δψ) + (LH/L)·sin(Δψ)·tan(δf)):

    tan(δf) = [sin(Δψ) − cos(Δψ)·tan(δT)]
              ──────────────────────────────────────────────
              (LH/L)·[sin(Δψ)·tan(δT) + cos(Δψ)]

    VR = VT / F

Singularity: denominator → 0 near jackknife territory; return (None, None).
"""

import math
from config import TruckConfig

_DENOM_THRESH = 1e-4


class InverseKinematics:
    def __init__(self, cfg: TruckConfig):
        self.cfg = cfg
        self._ratio = cfg.LH / cfg.L      # LH/L ≈ 0.400

    def delta_T_to_delta_f(self, delta_T: float, dpsi: float) -> float | None:
        """
        Compute δf from virtual trailer steer δT and hitch angle Δψ = ψ₁ - ψ₂.
        Returns None on singularity (near jackknife).
        Result is clamped to ±max_steer.
        """
        r       = self._ratio
        tan_dT  = math.tan(delta_T)
        sin_d   = math.sin(dpsi)
        cos_d   = math.cos(dpsi)

        denom = r * (sin_d * tan_dT + cos_d)
        if abs(denom) < _DENOM_THRESH:
            return None

        numer   = sin_d - cos_d * tan_dT
        delta_f = math.atan(numer / denom)
        lim     = self.cfg.max_steer
        return max(-lim, min(lim, delta_f))

    def vT_to_vR(self, vT: float, delta_f: float, dpsi: float) -> float | None:
        """
        Compute VR from virtual trailer speed VT, solved δf, and Δψ.
        Returns None if the kinematic factor F is near-zero.
        """
        r = self._ratio
        factor = math.cos(dpsi) + r * math.sin(dpsi) * math.tan(delta_f)
        if abs(factor) < _DENOM_THRESH:
            return None
        return vT / factor

    def solve(self, delta_T: float, vT: float,
              dpsi: float) -> tuple:
        """
        Full IK: returns (delta_f, vR) or (None, None) on singularity.
        """
        delta_f = self.delta_T_to_delta_f(delta_T, dpsi)
        if delta_f is None:
            return None, None
        vR = self.vT_to_vR(vT, delta_f, dpsi)
        if vR is None:
            return None, None
        return delta_f, vR

    @property
    def max_delta_T(self) -> float:
        """Maximum virtual trailer steer angle derived from max_steer of truck.

        At dpsi=0: tan(δT_max) = (LH/L)·tan(δf_max)
        """
        return math.atan(self._ratio * math.tan(self.cfg.max_steer))
