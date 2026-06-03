import sys
import math
import enum
import pygame
from pygame.locals import QUIT, KEYDOWN, K_r, K_ESCAPE, K_p, K_w, K_s, K_a, K_d

from config import TruckConfig, SimConfig
from kinematics import TruckTrailerKinematics, TruckTrailerState, initial_state
from input_handler import InputHandler
from renderer import Renderer
from hud import HUD
from settings_screen import run_settings
from parking import ParkingManager
from autopark_scene import build_scene
from hybrid_astar import HybridAstarPlanner, replay_path
from auto_park_controller import AutoParkController


# ── Auto-park state machine ───────────────────────────────────────────────────

class APMode(enum.Enum):
    MANUAL    = 'manual'
    PLANNING  = 'planning'
    EXECUTING = 'executing'
    DONE      = 'done'
    FAILED    = 'failed'


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_initial_state(scene, cfg: TruckConfig) -> TruckTrailerState:
    """Place truck at scene start position with trailer aligned behind."""
    psi1 = scene.initial_psi1
    xR   = scene.initial_xR
    yR   = scene.initial_yR
    xH   = xR - cfg.LH * math.cos(psi1)
    yH   = yR - cfg.LH * math.sin(psi1)
    xT   = xH - cfg.LT * math.cos(psi1)
    yT   = yH - cfg.LT * math.sin(psi1)
    return TruckTrailerState(xR=xR, yR=yR, psi1=psi1, xT=xT, yT=yT, psi2=psi1)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    sim_cfg = SimConfig()

    pygame.init()
    pygame.display.set_caption("EuroTruck 3.0 — 2D Kinematic Trailer Sim")
    screen = pygame.display.set_mode((sim_cfg.window_w, sim_cfg.window_h))

    # Settings screen returns (TruckConfig, parking_enabled, autopark_enabled)
    truck_cfg, parking_enabled, autopark_enabled = run_settings(screen, TruckConfig())

    clock   = pygame.time.Clock()
    kin     = TruckTrailerKinematics(truck_cfg)
    handler = InputHandler(truck_cfg)
    ren     = Renderer(screen, truck_cfg, sim_cfg)
    hud     = HUD(screen)

    # ── Auto-park setup ───────────────────────────────────────────────────────
    scene = build_scene(truck_cfg) if autopark_enabled else None
    if autopark_enabled:
        assert scene is not None
        planner: HybridAstarPlanner | None = HybridAstarPlanner(truck_cfg, scene)
    else:
        planner = None
    ap_ctrl: AutoParkController | None = None
    ap_mode = APMode.MANUAL
    ap_fail_timer  = 0.0
    ap_path_states = []   # ghost waypoints for visualisation

    # ── Initial vehicle state ─────────────────────────────────────────────────
    if autopark_enabled:
        state = _make_initial_state(scene, truck_cfg)
    else:
        state = initial_state(truck_cfg)

    parking = ParkingManager(truck_cfg) if parking_enabled else None

    # ── Main loop ─────────────────────────────────────────────────────────────
    running = True
    while running:
        dt = clock.tick(sim_cfg.fps) / 1000.0
        dt = min(dt, 0.05)

        # ── Events ───────────────────────────────────────────────────────────
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False

            if event.type == pygame.MOUSEWHEEL:
                ren.adjust_zoom(event.y)

            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    if ap_mode in (APMode.PLANNING, APMode.EXECUTING):
                        if planner:
                            planner.abort()
                        ap_mode  = APMode.MANUAL
                        ap_ctrl  = None
                    else:
                        running = False

                elif event.key == K_r:
                    if autopark_enabled:
                        state = _make_initial_state(scene, truck_cfg)
                    else:
                        state = initial_state(truck_cfg)
                    handler.reset()
                    if ap_mode != APMode.MANUAL:
                        if planner:
                            planner.abort()
                        ap_mode  = APMode.MANUAL
                        ap_ctrl  = None
                        ap_path_states = []

                elif event.key == K_p and autopark_enabled and planner is not None:
                    if ap_mode == APMode.MANUAL:
                        planner.start(state)
                        ap_mode = APMode.PLANNING
                        ap_path_states = []
                    elif ap_mode in (APMode.DONE, APMode.FAILED):
                        assert scene is not None
                        state = _make_initial_state(scene, truck_cfg)
                        handler.reset()
                        planner.start(state)
                        ap_mode = APMode.PLANNING
                        ap_ctrl = None
                        ap_path_states = []

        # ── WASD abort during auto-execution ─────────────────────────────────
        keys = pygame.key.get_pressed()
        if ap_mode == APMode.EXECUTING:
            if any(keys[k] for k in (K_w, K_s, K_a, K_d)):
                ap_ctrl = None
                ap_mode = APMode.MANUAL

        # ── State machine transitions ─────────────────────────────────────────
        if ap_mode == APMode.PLANNING and planner and planner.is_done:
            path = planner.result
            if path:
                ap_ctrl = AutoParkController(path, state, kin)
                ap_path_states = replay_path(state, path, truck_cfg, sample_every=5)
                ap_mode = APMode.EXECUTING
            else:
                ap_mode = APMode.FAILED
                ap_fail_timer = 4.0

        if ap_mode == APMode.FAILED:
            ap_fail_timer -= dt
            if ap_fail_timer <= 0:
                ap_mode = APMode.MANUAL

        if ap_mode == APMode.EXECUTING and ap_ctrl and ap_ctrl.is_finished:
            ap_ctrl = None
            ap_mode = APMode.DONE

        # ── Control input + Physics ───────────────────────────────────────────
        if ap_mode == APMode.EXECUTING and ap_ctrl is not None:
            # Controller owns state: pre-computed RK4 states, no Euler drift
            state, delta_f, vR = ap_ctrl.update(dt)
        else:
            delta_f, vR = handler.update(keys, dt)
            state = kin.step(state, delta_f, vR, dt)

        hitch_deg = state.hitch_angle_deg
        jk_warn   = abs(hitch_deg) >= sim_cfg.jackknife_warn_deg
        jk_limit  = abs(hitch_deg) >= sim_cfg.jackknife_limit_deg
        hud.update(dt)

        if parking is not None:
            parking.update(state, vR, dt)

        # ── Render ────────────────────────────────────────────────────────────
        ren.draw(state, delta_f, vR, jk_warn, jk_limit)

        # Auto-park scene
        if scene is not None:
            ren.draw_scene_walls(scene)
            if ap_path_states:
                ren.draw_planned_path(ap_path_states, truck_cfg)
            ren.draw_parking_spot(scene.spot,
                                   is_parked=(ap_mode == APMode.DONE),
                                   inside=False)

        # Random parking challenge
        if parking is not None:
            inside = parking.vehicle_inside(state)
            ren.draw_parking_spot(parking.spot, parking.is_parked, inside)
            if not parking.is_parked:
                ren.draw_parking_arrow(parking.spot, parking.distance_to(state))

        # ── HUD ───────────────────────────────────────────────────────────────
        hud.draw(vR, math.degrees(delta_f), hitch_deg, jk_warn, jk_limit, ren.ppm)

        if autopark_enabled:
            if ap_mode == APMode.PLANNING:
                hud.draw_autopark_overlay()
                hud.draw_autopark_hud('planning')
            elif ap_mode == APMode.EXECUTING and ap_ctrl:
                hud.draw_autopark_hud('executing',
                                       ap_ctrl.current_step,
                                       ap_ctrl.total_steps)
            elif ap_mode == APMode.DONE:
                hud.draw_autopark_hud('done')
            elif ap_mode == APMode.FAILED:
                hud.draw_autopark_hud('failed')

        if parking is not None:
            hud.draw_parking_hud(parking.distance_to(state), parking.success_count)
            if parking.is_parked:
                hud.draw_parking_success(parking.success_count,
                                          parking.success_timer,
                                          ParkingManager.SUCCESS_HOLD)

        if autopark_enabled and ap_mode == APMode.MANUAL:
            # Remind user how to trigger planning
            hint_surf = hud.font_sm.render('Press P to start auto-park',
                                            True, (160, 160, 160))
            W = screen.get_width()
            screen.blit(hint_surf,
                        hint_surf.get_rect(center=(W // 2, screen.get_height() - 26)))

        pygame.display.flip()

    # ── Cleanup ───────────────────────────────────────────────────────────────
    if planner:
        planner.abort()
    pygame.quit()
    sys.exit()


if __name__ == '__main__':
    main()
