import sys
import math
import pygame
from pygame.locals import QUIT, KEYDOWN, K_ESCAPE

from config import TruckConfig, SimConfig
from kinematics import TruckTrailerKinematics, initial_state
from input_handler import InputHandler
from renderer import Renderer
from hud import HUD
from settings_screen import run_settings
from parking import ParkingManager
from autopark_scene import build_scene
from autopark_state import (
    APMode, AutoParkState,
    init_autopark, make_initial_state,
    handle_autopark_key, handle_wasd_abort,
    update as ap_update, get_control,
)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    sim_cfg = SimConfig()

    pygame.init()
    pygame.display.set_caption("EuroTruck 3.0 — 2D Kinematic Trailer Sim")
    screen = pygame.display.set_mode((sim_cfg.window_w, sim_cfg.window_h))

    truck_cfg, parking_enabled, autopark_enabled, ap_n_sub, ap_parallel = run_settings(screen, TruckConfig())

    clock   = pygame.time.Clock()
    kin     = TruckTrailerKinematics(truck_cfg)
    handler = InputHandler(truck_cfg)
    ren     = Renderer(screen, truck_cfg, sim_cfg)
    hud     = HUD(screen)

    if autopark_enabled:
        if ap_parallel:
            from parallel_park_scene import build_parallel_scene
            scene = build_parallel_scene(truck_cfg)
        else:
            scene = build_scene(truck_cfg)
        aps: AutoParkState | None = init_autopark(truck_cfg, scene, ap_n_sub)
        state = make_initial_state(scene, truck_cfg)
    else:
        scene = None
        aps   = None
        state = initial_state(truck_cfg)
    parking = ParkingManager(truck_cfg) if parking_enabled else None

    # ── Main loop ─────────────────────────────────────────────────────────────
    running = True
    while running:
        dt = min(clock.tick(sim_cfg.fps) / 1000.0, 0.05)

        # ── Events ───────────────────────────────────────────────────────────
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False

            elif event.type == pygame.MOUSEWHEEL:
                ren.adjust_zoom(event.y)

            elif event.type == KEYDOWN:
                if aps is not None and scene is not None:
                    state, quit_requested = handle_autopark_key(
                        aps, event.key, state, truck_cfg, scene, handler, autopark_enabled)
                    if quit_requested:
                        running = False
                elif event.key == K_ESCAPE:
                    running = False

        # ── Auto-park frame update ────────────────────────────────────────────
        keys = pygame.key.get_pressed()
        if aps is not None:
            handle_wasd_abort(aps, keys)
            ap_update(aps, state, truck_cfg, dt)

        # ── Control + Physics ─────────────────────────────────────────────────
        delta_f, vR = get_control(aps, handler, keys, dt)

        hitch_deg = state.hitch_angle_deg
        jk_warn   = abs(hitch_deg) >= sim_cfg.jackknife_warn_deg
        jk_limit  = abs(hitch_deg) >= sim_cfg.jackknife_limit_deg

        phys_dt = (aps.planner.dt_sub
                   if aps and aps.mode == APMode.EXECUTING and aps.ctrl and aps.planner
                   else dt)
        state = kin.step_rk4(state, delta_f, vR, phys_dt)
        hud.update(dt)
        if parking is not None:
            parking.update(state, vR, dt)

        # ── Render ────────────────────────────────────────────────────────────
        ren.draw_frame(state, delta_f, vR, jk_warn, jk_limit,
                       aps=aps, scene=scene, parking=parking)
        hud.draw_frame(vR, math.degrees(delta_f), hitch_deg, jk_warn, jk_limit, ren.ppm,
                       aps=aps, autopark_enabled=autopark_enabled,
                       parking=parking, state=state)
        pygame.display.flip()

    # ── Cleanup ───────────────────────────────────────────────────────────────
    if aps and aps.planner:
        aps.planner.abort()
    pygame.quit()
    sys.exit()


if __name__ == '__main__':
    main()
