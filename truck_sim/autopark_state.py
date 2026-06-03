"""Auto-park state machine — APMode enum, AutoParkState container, and all
transition logic extracted from main.py so the main loop stays thin."""

import enum
import math
from dataclasses import dataclass, field

import pygame
from pygame.locals import K_p, K_l, K_r, K_ESCAPE, K_w, K_s, K_a, K_d

from config import TruckConfig
from kinematics import TruckTrailerState, initial_state
from hybrid_astar import HybridAstarPlanner, replay_path
from auto_park_controller import AutoParkController
from autopark_scene import AutoParkScene


# ── Mode enum ─────────────────────────────────────────────────────────────────

class APMode(enum.Enum):
    MANUAL    = 'manual'
    PLANNING  = 'planning'
    EXECUTING = 'executing'
    DONE      = 'done'
    FAILED    = 'failed'


# ── State container ───────────────────────────────────────────────────────────

@dataclass
class AutoParkState:
    """Holds every ap_* variable that was previously scattered in main()."""
    mode:        APMode                    = APMode.MANUAL
    ctrl:        AutoParkController | None = None
    planner:     HybridAstarPlanner | None = None
    path:        list                      = field(default_factory=list)
    path_states: list                      = field(default_factory=list)
    start_state: TruckTrailerState | None  = None
    fail_timer:  float                     = 0.0


# ── Factory ───────────────────────────────────────────────────────────────────

def init_autopark(truck_cfg: TruckConfig, scene: AutoParkScene,
                  ap_n_sub: int) -> AutoParkState:
    """Create a fully-initialised AutoParkState with a live planner."""
    aps = AutoParkState()
    aps.planner = HybridAstarPlanner(truck_cfg, scene, n_sub=ap_n_sub)
    return aps


# ── Helper used both here and in main ─────────────────────────────────────────

def make_initial_state(scene: AutoParkScene, cfg: TruckConfig) -> TruckTrailerState:
    """Place truck at scene start position with trailer aligned behind."""
    psi1 = scene.initial_psi1
    xR   = scene.initial_xR
    yR   = scene.initial_yR
    xH   = xR - cfg.LH * math.cos(psi1)
    yH   = yR - cfg.LH * math.sin(psi1)
    xT   = xH - cfg.LT * math.cos(psi1)
    yT   = yH - cfg.LT * math.sin(psi1)
    return TruckTrailerState(xR=xR, yR=yR, psi1=psi1, xT=xT, yT=yT, psi2=psi1)


# ── Event handlers ────────────────────────────────────────────────────────────

def handle_autopark_key(aps: AutoParkState, key: int,
                        state: TruckTrailerState,
                        truck_cfg: TruckConfig,
                        scene: AutoParkScene,
                        handler,
                        autopark_enabled: bool) -> tuple[TruckTrailerState, bool]:
    """Handle a KEYDOWN event that may affect the auto-park state machine.

    Returns (new_vehicle_state, should_quit).
    should_quit is True only when K_ESCAPE is pressed in MANUAL mode.
    """
    if key == K_ESCAPE:
        if aps.mode in (APMode.PLANNING, APMode.EXECUTING):
            if aps.planner:
                aps.planner.abort()
            aps.mode = APMode.MANUAL
            aps.ctrl = None
            return state, False
        else:
            return state, True   # signal quit to caller

    if key == K_r:
        new_state = make_initial_state(scene, truck_cfg) if autopark_enabled else initial_state(truck_cfg)
        handler.reset()
        if aps.mode != APMode.MANUAL:
            if aps.planner:
                aps.planner.abort()
            aps.mode        = APMode.MANUAL
            aps.ctrl        = None
            aps.path_states = []
        return new_state, False

    if key == K_p and autopark_enabled and aps.planner is not None:
        if aps.mode == APMode.MANUAL:
            aps.start_state = state
            aps.planner.start(state)
            aps.mode        = APMode.PLANNING
            aps.path_states = []
        elif aps.mode in (APMode.DONE, APMode.FAILED):
            handler.reset()
            aps.start_state = state
            aps.planner.start(state)
            aps.mode        = APMode.PLANNING
            aps.ctrl        = None
            aps.path_states = []

    if (key == K_l and aps.mode == APMode.DONE
            and aps.path and aps.start_state is not None
            and aps.planner is not None):
        state    = aps.start_state
        aps.ctrl = AutoParkController(aps.path)
        aps.mode = APMode.EXECUTING

    return state, False


def handle_wasd_abort(aps: AutoParkState, keys) -> None:
    """Cancel auto-execution if the driver touches any WASD key."""
    if aps.mode == APMode.EXECUTING:
        if any(keys[k] for k in (K_w, K_s, K_a, K_d)):
            aps.ctrl = None
            aps.mode = APMode.MANUAL


# ── Per-frame state machine update ───────────────────────────────────────────

def update(aps: AutoParkState, current_state: TruckTrailerState,
           truck_cfg: TruckConfig, dt: float) -> None:
    """Advance the state machine by one frame (call once per main loop tick)."""

    # PLANNING → EXECUTING / FAILED
    if aps.mode == APMode.PLANNING and aps.planner and aps.planner.is_done:
        path = aps.planner.result
        if path:
            aps.path        = path
            aps.ctrl        = AutoParkController(path)
            aps.path_states = replay_path(
                aps.start_state or current_state, path, truck_cfg,
                sample_every=5, dt_sim=aps.planner.dt_sub)
            aps.mode        = APMode.EXECUTING
        else:
            aps.mode       = APMode.FAILED
            aps.fail_timer = 4.0

    # FAILED → MANUAL (timer)
    if aps.mode == APMode.FAILED:
        aps.fail_timer -= dt
        if aps.fail_timer <= 0:
            aps.mode = APMode.MANUAL

    # EXECUTING → DONE
    if aps.mode == APMode.EXECUTING and aps.ctrl and aps.ctrl.is_finished:
        aps.ctrl = None
        aps.mode = APMode.DONE


# ── Control input selector ────────────────────────────────────────────────────

def get_control(aps: AutoParkState | None, handler, keys, dt: float) -> tuple[float, float]:
    """Return (delta_f, vR) from the auto-park controller or the manual handler."""
    if (aps is not None
            and aps.mode == APMode.EXECUTING
            and aps.ctrl is not None
            and aps.planner is not None):
        return aps.ctrl.update(aps.planner.dt_sub)
    return handler.update(keys, dt)
