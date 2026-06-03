"""Auto-park trajectory controller.

Pre-computes all planned states at init using the same step_rk4 as the planner,
so execution exactly matches the planned trajectory instead of accumulating
Euler integration error over many small frames.

Between step boundaries the state is linearly interpolated for smooth animation.
"""

import math
from kinematics import TruckTrailerState, TruckTrailerKinematics


def _lerp_angle(a: float, b: float, t: float) -> float:
    diff = ((b - a + math.pi) % (2 * math.pi)) - math.pi
    return a + diff * t


def _lerp_state(s0: TruckTrailerState, s1: TruckTrailerState,
                t: float) -> TruckTrailerState:
    return TruckTrailerState(
        xR=s0.xR + (s1.xR - s0.xR) * t,
        yR=s0.yR + (s1.yR - s0.yR) * t,
        psi1=_lerp_angle(s0.psi1, s1.psi1, t),
        xT=s0.xT + (s1.xT - s0.xT) * t,
        yT=s0.yT + (s1.yT - s0.yT) * t,
        psi2=_lerp_angle(s0.psi2, s1.psi2, t),
    )


class AutoParkController:
    """Replay a planned path using the same RK4 integration as the planner.

    All planned states are computed once at construction with step_rk4, so the
    executed trajectory is identical to the planned one regardless of frame rate.
    Between step boundaries the state is interpolated linearly for smooth visuals.

    update() returns (state, delta_f, vR).  The caller must set the vehicle
    state to the returned value and must NOT call kin.step() separately.
    """

    def __init__(self, path: list,
                 start_state: TruckTrailerState,
                 kin: TruckTrailerKinematics):
        self._path   = path
        self._idx    = 0
        self._t_acc  = 0.0

        # Pre-compute every boundary state with step_rk4, matching the planner.
        self._states = [start_state]
        s = start_state
        for delta_f, vR, step_dt in path:
            s = kin.step_rk4(s, delta_f, vR, step_dt)
            self._states.append(s)

    # ── Properties ────────────────────────────────────────────────────────────

    @property
    def total_steps(self) -> int:
        return len(self._path)

    @property
    def current_step(self) -> int:
        return self._idx

    @property
    def is_finished(self) -> bool:
        return self._idx >= len(self._path)

    # ── Update ────────────────────────────────────────────────────────────────

    def update(self, dt: float) -> tuple:
        """Advance by dt seconds. Returns (state, delta_f, vR).

        state is the vehicle state that the caller should use this frame.
        The caller must NOT integrate state independently.
        """
        if self.is_finished:
            return self._states[-1], 0.0, 0.0

        _, _, step_dt = self._path[self._idx]
        self._t_acc += dt
        if self._t_acc >= step_dt:
            self._t_acc -= step_dt
            self._idx += 1

        if self.is_finished:
            return self._states[-1], 0.0, 0.0

        # Interpolate within the current planned step for smooth animation
        _, _, cur_step_dt = self._path[self._idx]
        t_frac = min(1.0, self._t_acc / cur_step_dt) if cur_step_dt > 0 else 1.0
        state  = _lerp_state(self._states[self._idx],
                              self._states[self._idx + 1],
                              t_frac)

        cur_df, cur_vr, _ = self._path[self._idx]
        return state, cur_df, cur_vr
