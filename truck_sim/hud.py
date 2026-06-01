import math
import pygame

# Color palette
WHITE   = (255, 255, 255)
GREEN   = (60,  210, 80)
YELLOW  = (255, 220, 0)
ORANGE  = (255, 140, 0)
RED     = (220, 50,  50)
GRAY    = (160, 160, 160)
DARK    = (20,  20,  20)


class HUD:
    """Overlay UI: speed/steer/hitch gauges and status text."""

    PANEL_W = 270
    PANEL_H = 150

    def __init__(self, screen: pygame.Surface):
        self.screen = screen
        self.font_md = pygame.font.SysFont('consolas', 19)
        self.font_lg = pygame.font.SysFont('consolas', 22, bold=True)
        self.font_sm = pygame.font.SysFont('consolas', 16)
        self._blink_t   = 0.0
        self._blink_vis = True
        # Pre-build semi-transparent panel surface
        self._panel = pygame.Surface((self.PANEL_W, self.PANEL_H), pygame.SRCALPHA)
        self._panel.fill((0, 0, 0, 170))

    def update(self, dt: float):
        self._blink_t += dt
        if self._blink_t >= 0.5:
            self._blink_t = 0.0
            self._blink_vis = not self._blink_vis

    def _hitch_color(self, deg: float) -> tuple:
        a = abs(deg)
        if a < 40:
            return GREEN
        if a < 60:
            return YELLOW
        if a < 80:
            return ORANGE
        return RED

    def draw(self, vR: float, steer_deg: float, hitch_deg: float,
             jackknife_warn: bool, jackknife_limit: bool):
        sx, sy = 12, 12
        self.screen.blit(self._panel, (sx, sy))

        px, py = sx + 10, sy + 8

        # Speed
        surf = self.font_md.render(f"Speed : {vR:+6.2f} m/s", True, WHITE)
        self.screen.blit(surf, (px, py))
        py += 26

        # Steer
        surf = self.font_md.render(f"Steer : {steer_deg:+6.1f} deg", True, WHITE)
        self.screen.blit(surf, (px, py))
        py += 26

        # Hitch angle
        hc = self._hitch_color(hitch_deg)
        surf = self.font_md.render(f"Hitch : {hitch_deg:+6.1f} deg", True, hc)
        self.screen.blit(surf, (px, py))
        py += 28

        # Status
        if jackknife_limit:
            if self._blink_vis:
                surf = self.font_lg.render("!!! JACKKNIFE !!!", True, RED)
                self.screen.blit(surf, (px, py))
        elif jackknife_warn:
            surf = self.font_lg.render("WARNING", True, ORANGE)
            self.screen.blit(surf, (px, py))
        else:
            surf = self.font_lg.render("NORMAL", True, GREEN)
            self.screen.blit(surf, (px, py))

        # Controls hint at bottom
        hint = "W/S: Throttle   A/D: Steer   R: Reset   ESC: Quit"
        surf = self.font_sm.render(hint, True, GRAY)
        self.screen.blit(surf, (10, self.screen.get_height() - 26))
