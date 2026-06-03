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
        self._park_font = pygame.font.SysFont('consolas', 22, bold=True)
        self._dist_font = pygame.font.SysFont('consolas', 14, bold=True)

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

    # ── Parking ─────────────────────────────────────────────────

    def _dashed_line(self, p1, p2, color, dash=8, gap=6, lw=2):
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        dist = math.hypot(dx, dy)
        if dist < 1:
            return
        step = dash + gap
        steps = int(dist / step) + 1
        for i in range(steps):
            t1 = min((i * step) / dist, 1.0)
            t2 = min((i * step + dash) / dist, 1.0)
            s1 = (int(p1[0] + dx * t1), int(p1[1] + dy * t1))
            s2 = (int(p1[0] + dx * t2), int(p1[1] + dy * t2))
            pygame.draw.line(self.screen, color, s1, s2, lw)

    def draw_parking_spot(self, spot, is_parked: bool, inside: bool):
        """Draw the parking spot rectangle with status colour coding."""
        corners_s = [self.w2s(x, y) for x, y in spot.corners()]

        # Semi-transparent fill via a temporary surface
        if is_parked:
            fill_col, alpha = (40, 200, 40),  80
            border_col = (80, 255, 80)
        elif inside:
            fill_col, alpha = (220, 200, 30), 55
            border_col = (255, 230, 50)
        else:
            fill_col, alpha = (220, 210, 50), 18
            border_col = (220, 210, 60)

        xs = [p[0] for p in corners_s]
        ys = [p[1] for p in corners_s]
        bx, by = min(xs), min(ys)
        bw = max(1, max(xs) - bx + 1)
        bh = max(1, max(ys) - by + 1)
        tmp = pygame.Surface((bw, bh), pygame.SRCALPHA)
        tmp.fill((0, 0, 0, 0))
        shifted = [(p[0] - bx, p[1] - by) for p in corners_s]
        pygame.draw.polygon(tmp, (*fill_col, alpha), shifted)
        self.screen.blit(tmp, (bx, by))

        # Dashed border
        n = len(corners_s)
        for i in range(n):
            self._dashed_line(corners_s[i], corners_s[(i + 1) % n],
                              border_col, dash=10, gap=6, lw=2)

        # Corner markers
        for cx, cy in corners_s:
            pygame.draw.circle(self.screen, border_col, (cx, cy), 4)

        # Centre "P" label
        sx, sy = self.w2s(spot.x, spot.y)
        s = self._park_font.render('P', True, border_col)
        self.screen.blit(s, (sx - s.get_width() // 2, sy - s.get_height() // 2))

    def draw_parking_arrow(self, spot, dist_m: float):
        """Screen-edge red triangle + distance text when the spot is off-screen."""
        W, H = self.scfg.window_w, self.scfg.window_h
        sx, sy = self.w2s(spot.x, spot.y)

        MARGIN = 60
        on_screen = (MARGIN <= sx <= W - MARGIN and MARGIN <= sy <= H - MARGIN)
        if on_screen:
            return

        # Direction from screen centre toward spot's screen position
        cx, cy = W / 2, H / 2
        dx, dy = sx - cx, sy - cy
        d = math.hypot(dx, dy)
        if d < 1:
            return
        ndx, ndy = dx / d, dy / d

        # Clamp arrow tip to screen edge (with margin)
        ts = []
        if ndx > 0:  ts.append((W - MARGIN - cx) / ndx)
        elif ndx < 0: ts.append((MARGIN - cx) / ndx)
        if ndy > 0:  ts.append((H - MARGIN - cy) / ndy)
        elif ndy < 0: ts.append((MARGIN - cy) / ndy)
        if not ts:
            return
        t = min(t for t in ts if t > 0)
        ax, ay = int(cx + ndx * t), int(cy + ndy * t)

        # Triangle
        ARR = 20
        px, py = -ndy, ndx   # perpendicular
        tip  = (ax, ay)
        bl   = (int(ax - ndx * ARR + px * 10), int(ay - ndy * ARR + py * 10))
        br   = (int(ax - ndx * ARR - px * 10), int(ay - ndy * ARR - py * 10))
        pygame.draw.polygon(self.screen, (210, 35, 35),   [tip, bl, br])
        pygame.draw.polygon(self.screen, (255, 140, 140), [tip, bl, br], 2)

        # Distance label next to arrow
        label = f'{dist_m:.0f} m'
        s = self._dist_font.render(label, True, (255, 180, 180))
        # Offset text away from the screen edge
        off_x = int(-ndx * (ARR + 10 + s.get_width() // 2))
        off_y = int(-ndy * (ARR + 10 + s.get_height() // 2))
        self.screen.blit(s, (ax + off_x - s.get_width() // 2,
                              ay + off_y - s.get_height() // 2))

    # ── Auto-park scene ─────────────────────────────────────────

    def draw_scene_walls(self, scene):
        """Draw the alley walls as solid dark-gray filled rectangles."""
        WALL_FILL   = (72, 72, 72)
        WALL_BORDER = (130, 130, 130)

        hw        = scene.spot.width / 2
        lwall_x   = -(hw + 0.5)
        rwall_x   =  (hw + 0.5)
        bwall_y   = scene.back_wall_y
        road_y    = 0.0
        FAR_X     = 42.0    # extends the wall visually beyond the vehicle path

        def draw_rect_w(wx1, wy1, wx2, wy2):
            s1 = self.w2s(wx1, wy1)
            s2 = self.w2s(wx2, wy2)
            rx = min(s1[0], s2[0])
            ry = min(s1[1], s2[1])
            rw = max(1, abs(s2[0] - s1[0]))
            rh = max(1, abs(s2[1] - s1[1]))
            import pygame as _pg
            r = _pg.Rect(rx, ry, rw, rh)
            _pg.draw.rect(self.screen, WALL_FILL,   r)
            _pg.draw.rect(self.screen, WALL_BORDER, r, 2)

        # Left wall block
        draw_rect_w(-FAR_X, bwall_y, lwall_x, road_y)
        # Right wall block
        draw_rect_w(rwall_x, bwall_y, FAR_X, road_y)
        # Back wall strip (thin, decorative)
        draw_rect_w(lwall_x, bwall_y - 0.5, rwall_x, bwall_y)

    def draw_planned_path(self, path_states: list, cfg):
        """Draw ghost truck+trailer outlines at sampled waypoints along the path.

        Truck outline : blue   (100, 180, 255)
        Trailer outline: orange (255, 160,  50)
        Hitch connector: thin line  rear-axle → hitch → trailer-axle
        """
        if not path_states:
            return
        import pygame as _pg
        GHOST_T  = (100, 180, 255)   # truck  — blue
        GHOST_TR = (255, 160,  50)   # trailer — orange
        n = len(path_states)
        for i, state in enumerate(path_states):
            alpha = int(30 + 120 * i / max(n - 1, 1))
            t_col = (
                int(GHOST_T[0]  * alpha / 150),
                int(GHOST_T[1]  * alpha / 150),
                min(255, int(GHOST_T[2]  * alpha / 150)),
            )
            r_col = (
                min(255, int(GHOST_TR[0] * alpha / 150)),
                int(GHOST_TR[1] * alpha / 150),
                int(GHOST_TR[2] * alpha / 150),
            )

            # Truck outline (blue)
            tc_off = cfg.truck_length / 2 - cfg.LH
            tcx = state.xR + tc_off * math.cos(state.psi1)
            tcy = state.yR + tc_off * math.sin(state.psi1)
            tk_c = [self.w2s(x, y)
                    for x, y in _rect_corners(tcx, tcy, state.psi1,
                                               cfg.truck_length, cfg.truck_width)]
            _pg.draw.polygon(self.screen, t_col, tk_c, 1)

            # Trailer outline (orange)
            tl_off = cfg.LT - cfg.trailer_length / 2
            tlx = state.xT + tl_off * math.cos(state.psi2)
            tly = state.yT + tl_off * math.sin(state.psi2)
            tl_c = [self.w2s(x, y)
                    for x, y in _rect_corners(tlx, tly, state.psi2,
                                               cfg.trailer_length, cfg.trailer_width)]
            _pg.draw.polygon(self.screen, r_col, tl_c, 1)

            # Hitch connector: rear-axle → hitch point → trailer-axle
            xH = state.xR - cfg.LH * math.cos(state.psi1)
            yH = state.yR - cfg.LH * math.sin(state.psi1)
            p_rear   = self.w2s(state.xR, state.yR)
            p_hitch  = self.w2s(xH, yH)
            p_trail  = self.w2s(state.xT, state.yT)
            _pg.draw.line(self.screen, t_col, p_rear,  p_hitch, 1)
            _pg.draw.line(self.screen, r_col, p_hitch, p_trail, 1)
            _pg.draw.circle(self.screen, (220, 220, 220), p_hitch, 2)

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

    def draw_frame(self, state: TruckTrailerState, delta_f: float, vR: float,
                   jk_warn: bool, jk_limit: bool,
                   aps=None, scene=None, parking=None):
        """Single render call per frame — vehicle + optional scene/parking overlays.

        aps: AutoParkState | None
        scene: AutoParkScene | None
        parking: ParkingManager | None
        """
        from autopark_state import APMode  # local import to avoid circular dependency

        self.draw(state, delta_f, vR, jk_warn, jk_limit)

        # Auto-park scene
        if scene is not None:
            self.draw_scene_walls(scene)
            if aps is not None and aps.path_states:
                self.draw_planned_path(aps.path_states, self.tcfg)
            self.draw_parking_spot(scene.spot,
                                   is_parked=(aps is not None and aps.mode == APMode.DONE),
                                   inside=False)

        # Random parking challenge
        if parking is not None:
            inside = parking.vehicle_inside(state)
            self.draw_parking_spot(parking.spot, parking.is_parked, inside)
            if not parking.is_parked:
                self.draw_parking_arrow(parking.spot, parking.distance_to(state))
