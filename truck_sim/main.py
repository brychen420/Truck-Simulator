import sys
import math
import pygame
from pygame.locals import QUIT, KEYDOWN, K_r, K_ESCAPE

from config import TruckConfig, SimConfig
from kinematics import TruckTrailerKinematics, initial_state
from input_handler import InputHandler
from renderer import Renderer
from hud import HUD
from settings_screen import run_settings
from parking import ParkingManager


def main():
    sim_cfg = SimConfig()

    pygame.init()
    pygame.display.set_caption("EuroTruck 3.0 — 2D Kinematic Trailer Sim")
    screen = pygame.display.set_mode((sim_cfg.window_w, sim_cfg.window_h))

    # Settings screen: configure vehicle and optional parking challenge
    truck_cfg, parking_enabled = run_settings(screen, TruckConfig())

    clock   = pygame.time.Clock()
    state   = initial_state(truck_cfg)
    handler = InputHandler(truck_cfg)
    kin     = TruckTrailerKinematics(truck_cfg)
    ren     = Renderer(screen, truck_cfg, sim_cfg)
    hud     = HUD(screen)

    parking = ParkingManager(truck_cfg) if parking_enabled else None

    running = True
    while running:
        dt = clock.tick(sim_cfg.fps) / 1000.0
        dt = min(dt, 0.05)

        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            if event.type == pygame.MOUSEWHEEL:
                ren.adjust_zoom(event.y)
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False
                if event.key == K_r:
                    state = initial_state(truck_cfg)
                    handler.reset()

        keys = pygame.key.get_pressed()
        delta_f, vR = handler.update(keys, dt)

        hitch_deg = state.hitch_angle_deg
        jk_warn   = abs(hitch_deg) >= sim_cfg.jackknife_warn_deg
        jk_limit  = abs(hitch_deg) >= sim_cfg.jackknife_limit_deg

        # if jk_limit:
        #     vR = 0.0

        state = kin.step(state, delta_f, vR, dt)
        hud.update(dt)

        # Update parking manager
        if parking is not None:
            parking.update(state, vR, dt)

        # ── Render ────────────────────────────────────────────────────────────
        ren.draw(state, delta_f, vR, jk_warn, jk_limit)

        if parking is not None:
            inside = parking.vehicle_inside(state)
            ren.draw_parking_spot(parking.spot, parking.is_parked, inside)
            if not parking.is_parked:
                ren.draw_parking_arrow(parking.spot, parking.distance_to(state))

        hud.draw(vR, math.degrees(delta_f), hitch_deg, jk_warn, jk_limit, ren.ppm)

        if parking is not None:
            hud.draw_parking_hud(parking.distance_to(state), parking.success_count)
            if parking.is_parked:
                hud.draw_parking_success(parking.success_count,
                                         parking.success_timer,
                                         ParkingManager.SUCCESS_HOLD)

        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == '__main__':
    main()
