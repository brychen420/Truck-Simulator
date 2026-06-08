"""Auto-park trajectory controller.

Replays a pre-planned list of (delta_f, vR, step_dt) triples.  The controller
owns only the path index; the caller drives the integration step with step_dt
so the physics exactly match what the planner computed.
"""


class AutoParkController:
    """Step through a planned path, one triple at a time.

    The path is a flat list of (delta_f, vR, step_dt) triples produced by
    run_hybrid_astar().  Call consume() to pop the current step; the caller
    is responsible for integrating with the returned step_dt.
    """

    def __init__(self, path: list):
        self._path  = path      # list of (delta_f, vR, step_dt)
        self._idx   = 0

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

    # ── Consume ───────────────────────────────────────────────────────────────

    def consume(self) -> tuple:
        """Pop and return (delta_f, vR, step_dt) for the current step.

        Advances the index by one.  Returns (0.0, 0.0, 0.0) if finished.
        """
        if self.is_finished:
            return 0.0, 0.0, 0.0
        triple = self._path[self._idx]
        self._idx += 1
        return triple
