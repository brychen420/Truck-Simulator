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
    PANEL_H = 175

    def __init__(self, screen: pygame.Surface):
        self.screen = screen
        self.font_md     = pygame.font.SysFont('consolas', 19)
        self.font_lg     = pygame.font.SysFont('consolas', 22, bold=True)
        self.font_sm     = pygame.font.SysFont('consolas', 16)
        self.font_paused = pygame.font.SysFont('consolas', 26, bold=True)
        self._blink_t   = 0.0
        self._blink_vis = True
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
             jackknife_warn: bool, jackknife_limit: bool, ppm: float = 40.0):
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

        # Zoom level
        zoom_x = ppm / 40.0
        surf = self.font_md.render(f"Zoom  :  {zoom_x:5.2f}x  (scroll)", True, GRAY)
        self.screen.blit(surf, (px, py))
        py += 26

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

    # ── Auto-park overlay ──────────────────────────────────────────────────

    def draw_autopark_overlay(self):
        """Dark full-screen overlay with 'PLANNING…' text during A* search."""
        W, H = self.screen.get_width(), self.screen.get_height()
        overlay = pygame.Surface((W, H), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 120))
        self.screen.blit(overlay, (0, 0))
        dots = '.' * (1 + int(self._blink_t * 6) % 4)
        surf = self.font_lg.render(f'PLANNING PATH{dots}', True, (255, 220, 70))
        self.screen.blit(surf, surf.get_rect(center=(W // 2, H // 2 - 18)))
        hint = self.font_sm.render('Press Esc to cancel', True, (180, 180, 180))
        self.screen.blit(hint, hint.get_rect(center=(W // 2, H // 2 + 26)))

    def draw_autopark_hud(self, mode: str, step: int = 0, total: int = 0):
        """Bottom-right status panel for auto-park mode.

        mode: 'planning' | 'executing' | 'done' | 'failed'
        """
        W, H = self.screen.get_width(), self.screen.get_height()
        pw, ph = 360, 64
        px = W - pw - 12
        py = H - ph - 38

        panel = pygame.Surface((pw, ph), pygame.SRCALPHA)
        panel.fill((0, 0, 0, 165))
        self.screen.blit(panel, (px, py))

        if mode == 'planning':
            text = 'Auto-Park: planning…'
            col  = (255, 220, 70)
        elif mode == 'executing':
            pct  = int(100 * step / max(total, 1))
            text = f'Auto-Park: {step}/{total}  ({pct}%)'
            col  = (80, 210, 255)
        elif mode == 'done':
            text = 'Parked!  L: replay   P: re-plan'
            col  = (80, 255, 80)
        elif mode == 'failed':
            text = 'No path found — reposition'
            col  = (255, 80, 80)
        else:
            return

        surf = self.font_sm.render(text, True, col)
        self.screen.blit(surf, (px + 8, py + 8))

        if mode == 'executing' and total > 0:
            bar_x = px + 8
            bar_y = py + 36
            bar_w = pw - 16
            bar_h = 10
            pygame.draw.rect(self.screen, (40, 70, 110),
                             (bar_x, bar_y, bar_w, bar_h), border_radius=5)
            filled = max(1, int(bar_w * step / total))
            pygame.draw.rect(self.screen, (80, 190, 255),
                             (bar_x, bar_y, filled, bar_h), border_radius=5)

        if mode == 'done':
            hint = self.font_sm.render('L: replay   P: re-plan   WASD: manual', True, GRAY)
            self.screen.blit(hint, (px + 8, py + 36))
        elif mode == 'failed':
            hint = self.font_sm.render('P: re-plan   WASD: manual', True, GRAY)
            self.screen.blit(hint, (px + 8, py + 36))

    def draw_paused_overlay(self):
        W, H = self.screen.get_size()
        overlay = pygame.Surface((W, 40), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 160))
        self.screen.blit(overlay, (0, H // 2 - 20))
        surf = self.font_paused.render('PAUSED  —  Space to resume', True, (255, 220, 60))
        self.screen.blit(surf, surf.get_rect(center=(W // 2, H // 2)))

    def draw_frame(self, vR: float, steer_deg: float, hitch_deg: float,
                   jk_warn: bool, jk_limit: bool, ppm: float,
                   aps=None, autopark_enabled: bool = False,
                   paused: bool = False):
        """Single HUD call per frame — gauges + all conditional overlays.

        aps: AutoParkState | None
        """
        from autopark_state import APMode  # local import to avoid circular dependency

        self.draw(vR, steer_deg, hitch_deg, jk_warn, jk_limit, ppm)

        if autopark_enabled and aps is not None:
            if aps.mode == APMode.PLANNING:
                self.draw_autopark_overlay()
                self.draw_autopark_hud('planning')
            elif aps.mode == APMode.EXECUTING and aps.ctrl is not None:
                self.draw_autopark_hud('executing',
                                       aps.ctrl.current_step,
                                       aps.ctrl.total_steps)
            elif aps.mode == APMode.DONE:
                self.draw_autopark_hud('done')
            elif aps.mode == APMode.FAILED:
                self.draw_autopark_hud('failed')

            if aps.mode == APMode.MANUAL:
                hint_surf = self.font_sm.render('Press P to start auto-park',
                                                True, (160, 160, 160))
                W = self.screen.get_width()
                self.screen.blit(hint_surf,
                                 hint_surf.get_rect(center=(W // 2,
                                                            self.screen.get_height() - 26)))

        if paused:
            self.draw_paused_overlay()
