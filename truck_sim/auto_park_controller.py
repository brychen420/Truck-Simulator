"""Auto-park trajectory controller.

Replays a pre-planned list of (delta_f, vR, dt) triples into the simulation's
existing main loop at the render frame rate.  The controller itself does NOT
call kin.step() — it only supplies (delta_f, vR) to the caller, which continues
using the single authoritative kinematics integration path in main.py.
"""


class AutoParkController:
    """Step through a planned path at real-time frame rate.

    The path is a flat list of (delta_f, vR, step_dt) triples produced by
    hybrid_astar.run_hybrid_astar().  Each triple covers one DT_SUB interval
    (0.2 s); the controller advances through them as wall-clock dt accumulates.
    """

    def __init__(self, path: list):
        self._path  = path          # list of (delta_f, vR, step_dt)
        self._idx   = 0             # index of currently active step
        self._t_acc = 0.0           # time accumulated in current step

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
        """Advance by dt seconds. Returns (delta_f, vR) for this frame.

        Once all steps are consumed, returns (0.0, 0.0).
        """
        if self.is_finished:
            return 0.0, 0.0

        delta_f, vR, step_dt = self._path[self._idx]
        self._t_acc += dt
        if self._t_acc >= step_dt:
            self._t_acc -= step_dt
            self._idx   += 1

        return delta_f, vR
