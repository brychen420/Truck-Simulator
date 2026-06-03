"""Hybrid A* planner for truck-trailer reverse parking.

State space : 4D trailer-centric  (xT, yT, ψ₂, Δψ)
Controls    : virtual trailer steering (δT, VT) converted to (δf, VR) via IK
Integration : RK4 at DT_SUB = 1/60 s per substep, N_SUB = 60 substeps per node
             (matches simulation frame rate exactly → zero integration mismatch)
Threading   : planning runs in a background daemon thread; main loop polls is_done
"""

import math
import heapq
import threading
from dataclasses import dataclass, field
from typing import Optional

from config import TruckConfig
from kinematics import TruckTrailerState, TruckTrailerKinematics
from autopark_scene import AutoParkScene
from inverse_kinematics import InverseKinematics
from parking import _rect_corners  # reuse existing corner helper

# ── Planner parameters ────────────────────────────────────────────────────────
XY_RES   = 1.0              # m  — spatial grid cell size
YAW_RES  = math.pi / 18    # rad — 10° yaw bins  → 36 bins per revolution
DPSI_RES = math.pi / 18    # rad — 10° hitch bins

DT_PLAN  = 1.0              # s  — planning horizon per node expansion
N_SUB    = 60               # integration substeps per expansion  (= DT_PLAN × 60 fps)
DT_SUB   = 1.0 / 60        # s  — exactly matches simulation frame rate (≈ 0.01667 s)

VT_VALS       = [1.0, -1.0]   # virtual trailer speed options (m/s)
N_STEER       = 9              # number of δT samples (including 0)
REVERSE_WEIGHT = 1.0           # no penalty for reverse (backing in is the goal)
H_W_PSI       = 2.0            # heuristic weight for heading error
H_W_DPSI      = 1.5            # heuristic weight for hitch angle
JACKKNIFE_LIM = math.radians(55)  # hard limit during planning (physical limit ~60°)
WALL_MARGIN   = 0.3            # m — safety buffer; planner stays this far from walls
DT_LB2        = 0.5            # rad — reasonable trailer virtual-steer bound (paper §4.3)

MAX_EXPANSIONS = 100_000

# Goal tolerances
GOAL_XY_TOL   = 0.8          # m
GOAL_PSI_TOL  = math.radians(15)
GOAL_DPSI_TOL = math.radians(10)  # tightened from 20° — prevents angled parking in narrow slot

_N_YAW_BINS = round(2 * math.pi / YAW_RES)  # 36


# ── Internal node ─────────────────────────────────────────────────────────────

@dataclass(order=True)
class _Node:
    f:       float                    # priority  (g + h)
    _count:  int   = field(compare=True)  # tie-breaker (monotone counter)
    g:       float = field(compare=False)
    xT:      float = field(compare=False)
    yT:      float = field(compare=False)
    psi2:    float = field(compare=False)
    dpsi:    float = field(compare=False)
    parent:  Optional['_Node']         = field(default=None, compare=False, repr=False)
    segment: Optional[list]            = field(default=None, compare=False)
    # segment = list of (delta_f, vR, DT_SUB) for the N_SUB substeps leading to this node


# ── Utility helpers ───────────────────────────────────────────────────────────

def _normalize(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _angle_diff(a: float, b: float) -> float:
    return _normalize(a - b)


def _disc(xT: float, yT: float, psi2: float, dpsi: float) -> tuple:
    """Discretize 4D state into a hashable grid key."""
    psi2_norm = math.fmod(psi2, 2 * math.pi)
    if psi2_norm < 0:
        psi2_norm += 2 * math.pi
    return (
        int(round(xT   / XY_RES)),
        int(round(yT   / XY_RES)),
        int(round(psi2_norm / YAW_RES)) % _N_YAW_BINS,
        int(round(dpsi / DPSI_RES)),
    )


def _reconstruct_state(xT: float, yT: float, psi2: float, dpsi: float,
                        cfg: TruckConfig) -> TruckTrailerState:
    """Rebuild full 6D state from trailer-centric 4D representation.

    Geometry (confirmed against kinematics.py conventions):
      psi1 = psi2 + dpsi
      H = trailer axle + LT * hat(psi2)      (hitch is LT ahead of trailer axle)
      R = H + LH * hat(psi1)                 (rear axle is LH ahead of hitch)
    """
    psi1 = _normalize(psi2 + dpsi)
    xH   = xT + cfg.LT * math.cos(psi2)
    yH   = yT + cfg.LT * math.sin(psi2)
    xR   = xH + cfg.LH * math.cos(psi1)
    yR   = yH + cfg.LH * math.sin(psi1)
    return TruckTrailerState(xR=xR, yR=yR, psi1=psi1, xT=xT, yT=yT, psi2=psi2)


def _all_corners(state: TruckTrailerState, cfg: TruckConfig) -> list:
    """8 world-space corners (4 truck + 4 trailer). Reuses parking._rect_corners."""
    tc_off = cfg.truck_length / 2 - cfg.LH
    tcx = state.xR + tc_off * math.cos(state.psi1)
    tcy = state.yR + tc_off * math.sin(state.psi1)
    tl_off = cfg.LT - cfg.trailer_length / 2
    tlx = state.xT + tl_off * math.cos(state.psi2)
    tly = state.yT + tl_off * math.sin(state.psi2)
    return (
        _rect_corners(tcx, tcy, state.psi1, cfg.truck_length, cfg.truck_width) +
        _rect_corners(tlx, tly, state.psi2, cfg.trailer_length, cfg.trailer_width)
    )


def _in_collision(state: TruckTrailerState, cfg: TruckConfig,
                   scene: AutoParkScene, dpsi: float) -> bool:
    if abs(dpsi) > JACKKNIFE_LIM:
        return True
    for x, y in _all_corners(state, cfg):
        if scene.point_in_collision(x, y, margin=WALL_MARGIN):
            return True
    return False


def _dT_from_dF(delta_f: float, dpsi: float, cfg: TruckConfig) -> float | None:
    """Forward map δf → δT at given hitch angle Δψ (paper Eq. 15).

    δT = atan( (L·sin(Δψ) − LH·cos(Δψ)·tan(δf)) /
               (L·cos(Δψ) + LH·sin(Δψ)·tan(δf)) )
    """
    sin_d, cos_d = math.sin(dpsi), math.cos(dpsi)
    tan_f = math.tan(delta_f)
    denom = cfg.L * cos_d + cfg.LH * sin_d * tan_f
    if abs(denom) < 1e-6:
        return None
    return math.atan((cfg.L * sin_d - cfg.LH * cos_d * tan_f) / denom)


def _dT_adaptive_range(dpsi: float, cfg: TruckConfig) -> tuple:
    """Adaptive δT limits at the current hitch angle (paper §4.3, Eqs. 15–16).

    Returns (dT_lo, dT_hi): intersection of
      [δT_lb1, δT_ub1]  — range mapped from physical steering limits ±max_steer
      [−DT_LB2, +DT_LB2] — fixed reasonable-trailer-orientation bound
    If the intersection is empty, returns (0.0, 0.0) (only straight ahead).
    """
    lim = cfg.max_steer
    a = _dT_from_dF(+lim, dpsi, cfg)
    b = _dT_from_dF(-lim, dpsi, cfg)
    if a is None or b is None:
        return -DT_LB2, DT_LB2
    dT_lo = max(min(a, b), -DT_LB2)
    dT_hi = min(max(a, b),  DT_LB2)
    if dT_lo > dT_hi:
        mid = (min(a, b) + max(a, b)) / 2
        mid = max(-DT_LB2, min(DT_LB2, mid))
        return mid, mid
    return dT_lo, dT_hi


def _heuristic(xT: float, yT: float, psi2: float, dpsi: float,
                scene: AutoParkScene) -> float:
    euc    = math.hypot(xT - scene.goal_xT, yT - scene.goal_yT)
    d_psi  = abs(_angle_diff(psi2, scene.goal_psi2))
    d_dpsi = abs(_angle_diff(dpsi, scene.goal_dpsi))
    return euc + H_W_PSI * d_psi + H_W_DPSI * d_dpsi


def _is_goal(node: _Node, scene: AutoParkScene) -> bool:
    return (
        abs(node.xT  - scene.goal_xT)                  < GOAL_XY_TOL  and
        abs(node.yT  - scene.goal_yT)                  < GOAL_XY_TOL  and
        abs(_angle_diff(node.psi2, scene.goal_psi2))   < GOAL_PSI_TOL and
        abs(_angle_diff(node.dpsi, scene.goal_dpsi))   < GOAL_DPSI_TOL
    )


def _extract_path(goal_node: _Node) -> list:
    """Walk parent chain, collect segments, flatten into (δf, VR, dt) triples."""
    segments = []
    node = goal_node
    while node.parent is not None:
        segments.append(node.segment)
        node = node.parent
    segments.reverse()
    return [ctrl for seg in segments for ctrl in seg]


# ── Core search ───────────────────────────────────────────────────────────────

def run_hybrid_astar(start_state: TruckTrailerState,
                     scene: AutoParkScene,
                     cfg: TruckConfig,
                     stop_event: threading.Event,
                     n_sub: int = N_SUB) -> list:
    """
    Run Hybrid A* from start_state to scene goal.

    Returns a flat list of (delta_f, vR, dt) triples (one per DT_SUB substep).
    Returns [] if no path found or stop_event is set before completion.
    """
    ik     = InverseKinematics(cfg)
    kin    = TruckTrailerKinematics(cfg)
    dt_sub = DT_PLAN / n_sub

    # Extract start 4D state
    s0     = start_state
    dpsi0  = _normalize(s0.psi1 - s0.psi2)

    # Reject start if already jackknifed
    if abs(dpsi0) > JACKKNIFE_LIM:
        return []

    h0     = _heuristic(s0.xT, s0.yT, s0.psi2, dpsi0, scene)

    counter = 0
    start_node = _Node(f=h0, _count=counter, g=0.0,
                       xT=s0.xT, yT=s0.yT, psi2=s0.psi2, dpsi=dpsi0)

    open_heap = [start_node]
    closed    = set()
    expansions = 0

    while open_heap and expansions < MAX_EXPANSIONS:
        if stop_event.is_set():
            return []

        current = heapq.heappop(open_heap)
        key     = _disc(current.xT, current.yT, current.psi2, current.dpsi)

        if key in closed:
            continue
        closed.add(key)

        if _is_goal(current, scene):
            return _extract_path(current)

        expansions += 1

        # ── Expand: adaptive δT range per node (paper §4.3 Eqs. 15-16) ──────
        dT_lo, dT_hi = _dT_adaptive_range(current.dpsi, cfg)
        dT_values = ([dT_lo] if dT_lo == dT_hi else
                     [dT_lo + (dT_hi - dT_lo) * i / (N_STEER - 1)
                      for i in range(N_STEER)])
        for vT in VT_VALS:
            for dT in dT_values:
                delta_f, vR = ik.solve(dT, vT, current.dpsi)
                if delta_f is None:
                    continue

                # Integrate N_SUB substeps
                s = _reconstruct_state(current.xT, current.yT,
                                        current.psi2, current.dpsi, cfg)
                segment = []
                ok = True
                for _ in range(n_sub):
                    s       = kin.step_rk4(s, delta_f, vR, dt_sub)
                    new_dps = _normalize(s.psi1 - s.psi2)
                    segment.append((delta_f, vR, dt_sub))
                    if _in_collision(s, cfg, scene, new_dps):
                        ok = False
                        break

                if not ok:
                    continue

                new_dps  = _normalize(s.psi1 - s.psi2)
                new_key  = _disc(s.xT, s.yT, s.psi2, new_dps)
                if new_key in closed:
                    continue

                # Cost
                arc   = abs(vR) * DT_PLAN
                extra = (REVERSE_WEIGHT - 1) * arc if vR < 0 else 0.0
                g_new = current.g + arc + extra
                h_new = _heuristic(s.xT, s.yT, s.psi2, new_dps, scene)

                counter += 1
                child = _Node(
                    f=g_new + h_new, _count=counter, g=g_new,
                    xT=s.xT, yT=s.yT, psi2=s.psi2, dpsi=new_dps,
                    parent=current, segment=segment,
                )
                heapq.heappush(open_heap, child)

    return []   # exhausted or aborted


# ── Thread-safe public interface ──────────────────────────────────────────────

class HybridAstarPlanner:
    """Wraps run_hybrid_astar in a background daemon thread.

    Usage::

        planner = HybridAstarPlanner(cfg, scene)
        planner.start(current_state)
        # each frame:
        if planner.is_done:
            path = planner.result   # list or []
    """

    def __init__(self, cfg: TruckConfig, scene: AutoParkScene, n_sub: int = N_SUB):
        self.cfg    = cfg
        self.scene  = scene
        self.n_sub  = n_sub
        self.dt_sub = DT_PLAN / n_sub
        self._stop  = threading.Event()
        self._done  = threading.Event()
        self._result: list | None = None
        self._thread: threading.Thread | None = None

    def start(self, state: TruckTrailerState):
        """Abort any running search, then launch a new planning thread."""
        self.abort()
        self._stop.clear()
        self._done.clear()
        self._result = None
        self._thread = threading.Thread(
            target=self._run, args=(state,), daemon=True, name='HybridAStar')
        self._thread.start()

    def _run(self, state: TruckTrailerState):
        self._result = run_hybrid_astar(state, self.scene, self.cfg, self._stop, self.n_sub)
        self._done.set()

    def abort(self):
        """Signal the planning thread to stop; block briefly until it exits."""
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=0.5)

    @property
    def is_done(self) -> bool:
        return self._done.is_set()

    @property
    def result(self) -> list | None:
        """None while running. Empty list = no path found. Otherwise (δf,VR,dt) list."""
        return self._result


# ── Utility: replay path to list of TruckTrailerState (for visualisation) ────

def replay_path(start_state: TruckTrailerState,
                path: list,
                cfg: TruckConfig,
                sample_every: int = 5,
                dt_sim: float = 1.0 / 60) -> list:
    """
    Re-simulate the planned path at simulation frame rate (dt_sim ≈ 0.017 s).

    Each planned step (df, vr, step_dt) is subdivided into fine RK4 sub-steps
    of dt_sim so the ghost path matches the actual execution trajectory exactly.
    Returns a list of TruckTrailerState sampled every `sample_every` planned steps.
    """
    kin    = TruckTrailerKinematics(cfg)
    state  = start_state
    states = [state]
    for i, (df, vr, step_dt) in enumerate(path):
        t_rem = step_dt
        while t_rem > 1e-9:
            dt    = min(dt_sim, t_rem)
            state = kin.step_rk4(state, df, vr, dt)
            t_rem -= dt
        if (i + 1) % sample_every == 0:
            states.append(state)
    return states
