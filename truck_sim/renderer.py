import math
import pygame
from config import TruckConfig, SimConfig
from kinematics import TruckTrailerState, hitch_point

# ── Color palette ──────────────────────────────────────────────
BG_COLOR          = (38,  38,  38)
GRID_COLOR        = (58,  58,  58)
GRID_AXIS_COLOR   = (85,  85,  85)
TRUCK_FILL        = (55,  110, 200)
TRUCK_BORDER      = (180, 200, 255)
TRUCK_CAB_FILL    = (35,  75,  160)
TRAILER_FILL      = (55,  155, 90)
TRAILER_BORDER    = (160, 230, 160)
TRAILER_WARN_FILL = (200, 120, 40)
TRAILER_JK_FILL   = (200, 50,  50)
HITCH_LINE        = (240, 200, 60)
HITCH_DOT_OUTER   = (255, 255, 200)
HITCH_DOT_INNER   = (240, 200, 60)
AXLE_DOT          = (255, 200, 50)
WHEEL_COLOR       = (255, 240, 80)
WHITE             = (255, 255, 255)
DARK              = (20,  20,  20)


def _rect_corners(cx: float, cy: float, yaw: float, length: float, width: float) -> list:
    """Four world-space corners of a rectangle, centred at (cx, cy), rotated by yaw."""
    hl, hw = length / 2, width / 2
    local = [(-hl, -hw), (-hl, hw), (hl, hw), (hl, -hw)]
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    return [
        (cx + x * cos_y - y * sin_y,
         cy + x * sin_y + y * cos_y)
        for x, y in local
    ]


class Renderer:
    _PPM_MIN = 5.0
    _PPM_MAX = 300.0
    _ZOOM_STEP = 1.15   # multiply/divide per scroll tick

    def __init__(self, screen: pygame.Surface, truck_cfg: TruckConfig, sim_cfg: SimConfig):
        self.screen    = screen
        self.tcfg      = truck_cfg
        self.scfg      = sim_cfg
        self.cam_x     = 0.0
        self.cam_y     = 0.0
        self.screen_cx = sim_cfg.window_w / 2
        self.screen_cy = sim_cfg.window_h / 2
        self.ppm       = sim_cfg.pixels_per_m   # mutable zoom level (px per metre)

    def adjust_zoom(self, scroll_y: int):
        """scroll_y > 0 = wheel up = zoom in; < 0 = wheel down = zoom out."""
        if scroll_y > 0:
            self.ppm = min(self.ppm * self._ZOOM_STEP, self._PPM_MAX)
        elif scroll_y < 0:
            self.ppm = max(self.ppm / self._ZOOM_STEP, self._PPM_MIN)

    # ── Coordinate conversion ───────────────────────────────────

    def w2s(self, xw: float, yw: float) -> tuple:
        """World → screen coordinates (y-axis flipped)."""
        sx  = (xw - self.cam_x) * self.ppm + self.screen_cx
        sy  = self.screen_cy - (yw - self.cam_y) * self.ppm
        return int(sx), int(sy)

    def _update_camera(self, state: TruckTrailerState):
        self.cam_x = state.xR
        self.cam_y = state.yR

    # ── Sub-draw routines ───────────────────────────────────────

    def _draw_grid(self):
        ppm     = self.ppm
        spacing = self.scfg.grid_spacing
        W, H    = self.scfg.window_w, self.scfg.window_h

        left_w  = self.cam_x - self.screen_cx / ppm
        right_w = self.cam_x + self.screen_cx / ppm
        bot_w   = self.cam_y - self.screen_cy / ppm
        top_w   = self.cam_y + self.screen_cy / ppm

        x = math.floor(left_w / spacing) * spacing
        while x <= right_w + spacing:
            sx, _ = self.w2s(x, 0)
            is_ax = abs(x) < spacing * 0.01
            pygame.draw.line(self.screen, GRID_AXIS_COLOR if is_ax else GRID_COLOR,
                             (sx, 0), (sx, H), 2 if is_ax else 1)
            x += spacing

        y = math.floor(bot_w / spacing) * spacing
        while y <= top_w + spacing:
            _, sy = self.w2s(0, y)
            is_ax = abs(y) < spacing * 0.01
            pygame.draw.line(self.screen, GRID_AXIS_COLOR if is_ax else GRID_COLOR,
                             (0, sy), (W, sy), 2 if is_ax else 1)
            y += spacing

    def _draw_body(self, corners_world: list, fill: tuple, border: tuple, bw: int = 2):
        pts = [self.w2s(x, y) for x, y in corners_world]
        pygame.draw.polygon(self.screen, fill,   pts)
        pygame.draw.polygon(self.screen, border, pts, bw)

    def _draw_trailer(self, state: TruckTrailerState, warn: bool, limit: bool):
        cfg = self.tcfg
        # Centre of trailer body relative to trailer axle
        offset = cfg.LT - cfg.trailer_length / 2
        cx = state.xT + offset * math.cos(state.psi2)
        cy = state.yT + offset * math.sin(state.psi2)
        corners = _rect_corners(cx, cy, state.psi2, cfg.trailer_length, cfg.trailer_width)

        fill = TRAILER_JK_FILL if limit else (TRAILER_WARN_FILL if warn else TRAILER_FILL)
        self._draw_body(corners, fill, TRAILER_BORDER)

        # Rear reflector strip
        rear_local_x = -(cfg.trailer_length / 2 - cfg.LT)  # = -(LT - trailer_length/2) ... let me recalc
        # Actually, trailer rear is at LT - trailer_length from axle direction (behind axle)
        # In local coords of trailer center: rear is at -trailer_length/2
        # In world coords: rear = center - (trailer_length/2) * direction
        rear_x = cx - (cfg.trailer_length / 2) * math.cos(state.psi2)
        rear_y = cy - (cfg.trailer_length / 2) * math.sin(state.psi2)
        half_w = cfg.trailer_width / 2
        perp_x = -math.sin(state.psi2) * half_w * 0.8
        perp_y =  math.cos(state.psi2) * half_w * 0.8
        r1 = self.w2s(rear_x + perp_x, rear_y + perp_y)
        r2 = self.w2s(rear_x - perp_x, rear_y - perp_y)
        pygame.draw.line(self.screen, (240, 80, 60), r1, r2, 3)

    def _draw_truck(self, state: TruckTrailerState):
        cfg = self.tcfg
        # Centre of truck body relative to rear axle
        offset = cfg.truck_length / 2 - cfg.LH
        cx = state.xR + offset * math.cos(state.psi1)
        cy = state.yR + offset * math.sin(state.psi1)
        corners = _rect_corners(cx, cy, state.psi1, cfg.truck_length, cfg.truck_width)
        self._draw_body(corners, TRUCK_FILL, TRUCK_BORDER)

        # Cab/windshield overlay at truck front (~1 m deep)
        cab_depth = 1.0
        front_dist = cfg.truck_length / 2  # from body centre to front
        cab_cx = cx + (front_dist - cab_depth / 2) * math.cos(state.psi1)
        cab_cy = cy + (front_dist - cab_depth / 2) * math.sin(state.psi1)
        cab_corners = _rect_corners(cab_cx, cab_cy, state.psi1, cab_depth, cfg.truck_width)
        pygame.draw.polygon(self.screen, TRUCK_CAB_FILL,
                            [self.w2s(x, y) for x, y in cab_corners])
        pygame.draw.polygon(self.screen, TRUCK_BORDER,
                            [self.w2s(x, y) for x, y in cab_corners], 2)

        # Rear axle dot
        pygame.draw.circle(self.screen, AXLE_DOT, self.w2s(state.xR, state.yR), 5)

    def _draw_hitch(self, state: TruckTrailerState):
        cfg = self.tcfg
        xH, yH = hitch_point(state, cfg)
        s_rear    = self.w2s(state.xR, state.yR)
        s_hitch   = self.w2s(xH, yH)
        s_trailer = self.w2s(state.xT, state.yT)

        pygame.draw.line(self.screen, HITCH_LINE, s_rear,    s_hitch,   3)
        pygame.draw.line(self.screen, HITCH_LINE, s_hitch,   s_trailer, 3)
        pygame.draw.circle(self.screen, HITCH_DOT_OUTER, s_hitch,   7)
        pygame.draw.circle(self.screen, HITCH_DOT_INNER, s_hitch,   4)
        pygame.draw.circle(self.screen, AXLE_DOT,        s_trailer, 5)

    def _draw_wheel_indicator(self, state: TruckTrailerState, delta_f: float):
        cfg = self.tcfg
        # Front axle position
        fax = state.xR + cfg.L * math.cos(state.psi1)
        fay = state.yR + cfg.L * math.sin(state.psi1)

        # Wheel direction arrow
        wheel_yaw = state.psi1 + delta_f
        arrow_len = 1.4   # m
        tip_x = fax + arrow_len * math.cos(wheel_yaw)
        tip_y = fay + arrow_len * math.sin(wheel_yaw)

        s0 = self.w2s(fax, fay)
        s1 = self.w2s(tip_x, tip_y)
        pygame.draw.line(self.screen, WHEEL_COLOR, s0, s1, 3)
        pygame.draw.circle(self.screen, WHEEL_COLOR, s0, 5)

    # ── Public API ──────────────────────────────────────────────

    def draw(self, state: TruckTrailerState, delta_f: float, vR: float,
             jackknife_warn: bool, jackknife_limit: bool):
        self._update_camera(state)

        # Layer 1: background
        self.screen.fill(BG_COLOR)

        # Layer 2: ground grid
        self._draw_grid()

        # Layer 3: trailer (drawn below truck)
        self._draw_trailer(state, jackknife_warn, jackknife_limit)

        # Layer 4: hitch linkage
        self._draw_hitch(state)

        # Layer 5: truck (drawn on top of trailer)
        self._draw_truck(state)

        # Layer 6: front wheel direction indicator
        self._draw_wheel_indicator(state, delta_f)
