"""Pre-simulation settings screen with interactive sliders and live preview."""

import math
import pygame
from config import TruckConfig

# ── Colour palette ────────────────────────────────────────────────────────────
C = {
    'bg':        (28,  28,  28),
    'sec':       (130, 190, 255),
    'label':     (205, 205, 205),
    'trk_bg':    (65,  65,  65),
    'trk_fill':  (58,  125, 215),
    'handle':    (175, 218, 255),
    'handle_b':  (90,  155, 220),
    'value':     (255, 222, 75),
    'white':     (255, 255, 255),
    'gray':      (125, 125, 125),
    'btn_rst':   (88,  52,  52),
    'btn_rst_h': (130, 72,  72),
    'btn_go':    (38,  95,  58),
    'btn_go_h':  (55,  148, 88),
    'prv_bg':    (34,  34,  42),
    'prv_bdr':   (72,  72,  72),
    'truck_f':   (55,  110, 200),
    'truck_b':   (150, 188, 255),
    'truck_c':   (35,  72,  155),
    'trail_f':   (50,  152, 85),
    'trail_b':   (140, 212, 140),
    'hitch':     (238, 196, 55),
    'axle':      (255, 196, 46),
    'dim':       (190, 190, 68),
    'dim_t':     (218, 218, 88),
}


# ── Slider ────────────────────────────────────────────────────────────────────

class Slider:
    _TRACK_H = 6
    _HANDLE_R = 11

    def __init__(self, tx: int, ty: int, tw: int, th: int,
                 lo: float, hi: float, default: float,
                 label: str, unit: str, fmt: str = '.2f'):
        self.tx, self.ty, self.tw, self.th = tx, ty, tw, th
        self.lo, self.hi = lo, hi
        self.value   = default
        self.default = default
        self.label   = label
        self.unit    = unit
        self.fmt     = fmt
        self._drag   = False
        self._cx     = 0
        self._refresh_cx()

    def _refresh_cx(self):
        t = max(0.0, min(1.0, (self.value - self.lo) / (self.hi - self.lo)))
        self._cx = int(self.tx + t * self.tw)

    def _apply_mx(self, mx: int):
        t = max(0.0, min(1.0, (mx - self.tx) / self.tw))
        self.value = self.lo + t * (self.hi - self.lo)
        self._refresh_cx()

    def handle_event(self, ev) -> bool:
        prev = self.value
        cy = self.ty + self.th // 2
        if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
            mx, my = ev.pos
            on_handle = math.hypot(mx - self._cx, my - cy) <= self._HANDLE_R + 6
            on_track  = (self.tx <= mx <= self.tx + self.tw and abs(my - cy) <= 18)
            if on_handle or on_track:
                self._drag = True
                self._apply_mx(mx)
        elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
            self._drag = False
        elif ev.type == pygame.MOUSEMOTION and self._drag:
            self._apply_mx(ev.pos[0])
        return self.value != prev

    def reset(self):
        self.value = self.default
        self._refresh_cx()

    def draw(self, surf: pygame.Surface,
             f_lbl: pygame.font.Font, f_val: pygame.font.Font,
             lbl_x: int, val_x: int):
        cy = self.ty + self.th // 2
        # Label
        s = f_lbl.render(self.label, True, C['label'])
        surf.blit(s, (lbl_x, cy - s.get_height() // 2))
        # Track
        tr = pygame.Rect(self.tx, cy - self._TRACK_H // 2, self.tw, self._TRACK_H)
        pygame.draw.rect(surf, C['trk_bg'],   tr, border_radius=3)
        fw = max(0, self._cx - self.tx)
        if fw:
            fr = pygame.Rect(self.tx, cy - self._TRACK_H // 2, fw, self._TRACK_H)
            pygame.draw.rect(surf, C['trk_fill'], fr, border_radius=3)
        # Handle
        pygame.draw.circle(surf, C['handle'],   (self._cx, cy), self._HANDLE_R)
        pygame.draw.circle(surf, C['handle_b'], (self._cx, cy), self._HANDLE_R, 2)
        # Value
        s = f_val.render(f"{self.value:{self.fmt}} {self.unit}", True, C['value'])
        surf.blit(s, (val_x, cy - s.get_height() // 2))


# ── Button ────────────────────────────────────────────────────────────────────

class _Btn:
    def __init__(self, x, y, w, h, text, col, hov):
        self.rect = pygame.Rect(x, y, w, h)
        self.text = text
        self.col, self.hov = col, hov

    def draw(self, surf, font, mpos):
        c = self.hov if self.rect.collidepoint(mpos) else self.col
        pygame.draw.rect(surf, c, self.rect, border_radius=10)
        pygame.draw.rect(surf, (185, 185, 185), self.rect, 2, border_radius=10)
        s = font.render(self.text, True, C['white'])
        surf.blit(s, s.get_rect(center=self.rect.center))

    def clicked(self, ev) -> bool:
        return (ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1
                and self.rect.collidepoint(ev.pos))


# ── Checkbox ─────────────────────────────────────────────────────────────────

class Checkbox:
    SIZE = 22

    def __init__(self, x: int, y: int, label: str, checked: bool = False):
        self.rect    = pygame.Rect(x, y, self.SIZE, self.SIZE)
        self.label   = label
        self.checked = checked

    def handle_event(self, ev):
        if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
            if self.rect.collidepoint(ev.pos):
                self.checked = not self.checked

    def draw(self, surf: pygame.Surface, font: pygame.font.Font,
             col=(210, 210, 210)):
        # Box
        pygame.draw.rect(surf, (52, 52, 52), self.rect)
        pygame.draw.rect(surf, (155, 155, 155), self.rect, 2, border_radius=3)
        # Checkmark
        if self.checked:
            m = 4
            pts = [
                (self.rect.x + m,              self.rect.y + self.SIZE // 2),
                (self.rect.x + self.SIZE // 3, self.rect.y + self.SIZE - m),
                (self.rect.x + self.SIZE - m,  self.rect.y + m),
            ]
            pygame.draw.lines(surf, (80, 215, 80), False, pts, 3)
        # Label
        s = font.render(self.label, True, col)
        surf.blit(s, (self.rect.right + 10,
                      self.rect.y + (self.SIZE - s.get_height()) // 2))


# ── Settings screen ───────────────────────────────────────────────────────────

class SettingsScreen:
    # Layout
    LBL_X = 20     # left edge of label text
    TRK_X = 258    # left edge of slider track
    TRK_W = 555    # slider track pixel width
    VAL_X = 824    # left edge of value text
    ROW_H = 42     # vertical pixels per slider row

    # Right preview panel
    PRV_X = 950
    PRV_Y = 52
    PRV_W = 310
    PRV_H = 615

    def __init__(self, screen: pygame.Surface, default_cfg: TruckConfig):
        self.screen = screen
        self.f_ttl = pygame.font.SysFont('consolas', 25, bold=True)
        self.f_sec = pygame.font.SysFont('consolas', 15, bold=True)
        self.f_lbl = pygame.font.SysFont('consolas', 14)
        self.f_val = pygame.font.SysFont('consolas', 14)
        self.f_btn = pygame.font.SysFont('consolas', 16, bold=True)
        self.f_dim = pygame.font.SysFont('consolas', 11)

        tx, tw, rh = self.TRK_X, self.TRK_W, self.ROW_H

        def S(y, lo, hi, dv, lbl, unit, fmt='.2f'):
            return Slider(tx, y, tw, rh, lo, hi, dv, lbl, unit, fmt)

        c = default_cfg
        y = 70
        self._sec_labels = []   # [(y, text), ...]

        # ── Truck section ──────────────────────────────────────────────────────
        self._sec_labels.append((y, '▸ Truck'))
        y += 25
        self.s_L   = S(y, 1.0, 5.0, c.L,            'Wheelbase  L',          'm'); y += rh
        self.s_LH  = S(y, 0.2, 2.5, c.LH,           'Rear Axle → Hitch  LH', 'm'); y += rh
        self.s_TrL = S(y, 3.0, 6.0, c.truck_length, 'Truck Length',           'm'); y += rh
        self.s_TrW = S(y, 1.5, 4.0, c.truck_width,  'Truck Width',            'm'); y += rh
        y += 14

        # ── Trailer / Cargo section ────────────────────────────────────────────
        self._sec_labels.append((y, '▸ Trailer / Cargo'))
        y += 25
        self.s_LT  = S(y, 0.5, 4.5, c.LT,             'Trailer Axle → Hitch LT', 'm'); y += rh
        self.s_TlL = S(y, 2.0, 15.0, c.trailer_length, 'Trailer Length',           'm'); y += rh
        self.s_TlW = S(y, 1.5, 4.0, c.trailer_width,  'Trailer Width',            'm'); y += rh
        y += 14

        # ── Physics section ────────────────────────────────────────────────────
        self._sec_labels.append((y, '▸ Physics'))
        y += 25
        self.s_steer = S(y, 0.2, 1.2, c.max_steer, 'Max Steer Angle', 'rad'); y += rh
        self.s_speed = S(y, 2.0, 15., c.max_speed,  'Max Speed',       'm/s'); y += rh

        self.sliders = [
            self.s_L, self.s_LH, self.s_TrL, self.s_TrW,
            self.s_LT, self.s_TlL, self.s_TlW,
            self.s_steer, self.s_speed,
        ]

        # ── Mode checkboxes ────────────────────────────────────────────────────
        y += 16
        self.cb_parking  = Checkbox(tx - 10, y,
                                    'Enable Parking Challenge',
                                    checked=False)
        y += Checkbox.SIZE + 8
        self.cb_autopark = Checkbox(tx - 10, y,
                                    'Enable Auto-Park Scene  (Hybrid A*)',
                                    checked=False)
        y += Checkbox.SIZE + 8
        self.cb_autopark_par = Checkbox(tx - 10, y,
                                        'Enable Auto-Park (Parallel)  (Hybrid A*)',
                                        checked=False)
        y += Checkbox.SIZE + 10

        # ── A* time step slider (visible only when autopark is checked) ────────
        self._ap_slider_y = y
        self.s_dt_sub = Slider(tx, y, tw, rh,
                               lo=1.0 / 60, hi=0.200, default=1.0 / 60,
                               label='A* Time Step', unit='s', fmt='.4f')
        self._btn_y_base = y + 10           # buttons when slider hidden
        self._btn_y_ap   = y + rh + 18     # buttons when slider visible

        # Buttons (position updated dynamically in _draw)
        self.btn_rst = _Btn(tx,            self._btn_y_base, 215, 44, 'Reset Defaults',
                            C['btn_rst'],  C['btn_rst_h'])
        self.btn_go  = _Btn(tx + tw - 270, self._btn_y_base, 270, 44, 'Start Simulation ▶',
                            C['btn_go'],   C['btn_go_h'])

    # ── Config readout ────────────────────────────────────────────────────────

    def _build_n_sub(self) -> int:
        return max(5, min(60, round(1.0 / self.s_dt_sub.value)))

    def _build_cfg(self) -> TruckConfig:
        return TruckConfig(
            L=self.s_L.value,   LH=self.s_LH.value,  LT=self.s_LT.value,
            truck_length=self.s_TrL.value,   truck_width=self.s_TrW.value,
            trailer_length=self.s_TlL.value, trailer_width=self.s_TlW.value,
            max_steer=self.s_steer.value,    max_speed=self.s_speed.value,
        )

    # ── Preview panel ─────────────────────────────────────────────────────────

    def _draw_preview(self, cfg: TruckConfig):
        PX, PY, PW, PH = self.PRV_X, self.PRV_Y, self.PRV_W, self.PRV_H

        # Panel background
        pygame.draw.rect(self.screen, C['prv_bg'],  (PX, PY, PW, PH), border_radius=8)
        pygame.draw.rect(self.screen, C['prv_bdr'], (PX, PY, PW, PH), 2, border_radius=8)

        # Header
        s = self.f_sec.render('Live Preview', True, C['sec'])
        self.screen.blit(s, (PX + PW // 2 - s.get_width() // 2, PY + 8))

        # World bounding box of truck+trailer (psi=0, rear axle at world origin)
        x_lo  = -(cfg.LH + cfg.trailer_length)
        x_hi  = cfg.truck_length - cfg.LH
        y_ext = max(cfg.truck_width, cfg.trailer_width) / 2

        mg     = 30
        hdr_h  = 28
        avail_w = PW - mg * 2
        avail_h = PH - hdr_h - mg * 2
        span_x  = max(x_hi - x_lo, 0.01)
        span_y  = max(2 * y_ext, 0.01)
        scale   = min(avail_w / span_x, avail_h / span_y)

        # Screen position of world origin (truck rear axle)
        ox = PX + mg + (-x_lo) * scale
        oy = PY + hdr_h + mg + (PH - hdr_h - mg * 2) / 2

        def w2p(wx, wy):
            return int(ox + wx * scale), int(oy - wy * scale)

        # Dashed centre axis
        for xi in range(PX + mg, PX + PW - mg, 9):
            pygame.draw.line(self.screen, (52, 52, 52),
                             (xi, int(oy)), (xi + 5, int(oy)), 1)

        def draw_box(cx, cy, length, width, fill, border):
            hl, hw = length / 2, width / 2
            pts = [w2p(cx + dx, cy + dy)
                   for dx, dy in [(-hl, -hw), (-hl, hw), (hl, hw), (hl, -hw)]]
            pygame.draw.polygon(self.screen, fill,   pts)
            pygame.draw.polygon(self.screen, border, pts, 2)

        # Trailer
        t_ax_x = -(cfg.LH + cfg.LT)
        t_cx   = t_ax_x + (cfg.LT - cfg.trailer_length / 2)
        draw_box(t_cx, 0, cfg.trailer_length, cfg.trailer_width,
                 C['trail_f'], C['trail_b'])

        # Hitch linkage
        pygame.draw.line(self.screen, C['hitch'], w2p(0, 0),      w2p(-cfg.LH, 0), 2)
        pygame.draw.line(self.screen, C['hitch'], w2p(-cfg.LH, 0), w2p(t_ax_x, 0), 2)
        pygame.draw.circle(self.screen, C['hitch'], w2p(-cfg.LH, 0), 5)
        pygame.draw.circle(self.screen, C['white'], w2p(-cfg.LH, 0), 5, 1)

        # Truck body
        draw_box(cfg.truck_length / 2 - cfg.LH, 0,
                 cfg.truck_length, cfg.truck_width, C['truck_f'], C['truck_b'])

        # Windshield shade at front of truck
        front_x = cfg.truck_length - cfg.LH
        cab_d   = min(1.2, cfg.truck_length * 0.22)
        draw_box(front_x - cab_d / 2, 0, cab_d, cfg.truck_width, C['truck_c'], C['truck_b'])

        # Axle dots
        for wx in (0, cfg.L, t_ax_x):
            pygame.draw.circle(self.screen, C['axle'], w2p(wx, 0), 4)

        # Dimension annotations
        ann_above = y_ext + 0.22
        ann_below = -(y_ext + 0.22)

        def dim_arrow(wx1, wx2, wy, text):
            s1, s2 = w2p(wx1, wy), w2p(wx2, wy)
            mid = ((s1[0] + s2[0]) // 2, (s1[1] + s2[1]) // 2)
            pygame.draw.line(self.screen, C['dim'], s1, s2, 1)
            for p in (s1, s2):
                pygame.draw.line(self.screen, C['dim'], (p[0], p[1]-4), (p[0], p[1]+4), 1)
            ts = self.f_dim.render(text, True, C['dim_t'])
            above = wy > 0
            self.screen.blit(ts, (mid[0] - ts.get_width() // 2,
                                  mid[1] + (-ts.get_height() - 2 if above else 4)))

        # Draw annotations only if scale is large enough to be readable (> 5px per char)
        if scale > 3:
            dim_arrow(0, cfg.L, ann_above, f'L={cfg.L:.2f}m')
            dim_arrow(-cfg.LH, 0, ann_below, f'LH={cfg.LH:.2f}m')
            dim_arrow(t_ax_x, -cfg.LH, ann_below - 0.45, f'LT={cfg.LT:.2f}m')

    # ── Main draw ─────────────────────────────────────────────────────────────

    def _draw(self):
        mpos = pygame.mouse.get_pos()
        self.screen.fill(C['bg'])

        # Title
        s = self.f_ttl.render('EuroTruck 3.0  —  Vehicle Setup', True, C['white'])
        self.screen.blit(s, (self.TRK_X, 20))

        # Section headers
        for y_h, text in self._sec_labels:
            s = self.f_sec.render(text, True, C['sec'])
            self.screen.blit(s, (self.TRK_X - 10, y_h))

        # Sliders
        for sl in self.sliders:
            sl.draw(self.screen, self.f_lbl, self.f_val, self.LBL_X, self.VAL_X)

        # Mode checkboxes
        self.cb_parking.draw(self.screen, self.f_lbl)
        self.cb_autopark.draw(self.screen, self.f_lbl)
        self.cb_autopark_par.draw(self.screen, self.f_lbl)

        # A* time step slider — only when either autopark mode is checked
        ap_checked = self.cb_autopark.checked or self.cb_autopark_par.checked
        if ap_checked:
            self.s_dt_sub.draw(self.screen, self.f_lbl, self.f_val, self.LBL_X, self.VAL_X)
            n = self._build_n_sub()
            info = self.f_dim.render(
                f'  N_SUB = {n}  |  DT_SUB = {1.0/n:.4f} s  |  ~{n/5:.1f}x slower than N=5',
                True, C['gray'])
            self.screen.blit(info, (self.TRK_X, self._ap_slider_y + self.ROW_H + 2))

        # Reposition buttons depending on whether slider is visible
        btn_y = self._btn_y_ap if ap_checked else self._btn_y_base
        self.btn_rst.rect.y = btn_y
        self.btn_go.rect.y  = btn_y

        # Buttons
        self.btn_rst.draw(self.screen, self.f_btn, mpos)
        self.btn_go.draw(self.screen, self.f_btn, mpos)

        # Hint
        hint_y = btn_y + 54
        hint = self.f_lbl.render(
            'Drag sliders to adjust   ·   Enter or ▶ to start   ·   Esc to quit',
            True, C['gray'])
        self.screen.blit(hint, (self.TRK_X, hint_y))

        # Live preview
        self._draw_preview(self._build_cfg())
        pygame.display.flip()

    # ── Event loop ────────────────────────────────────────────────────────────

    def _return_values(self) -> tuple:
        ap_enabled  = self.cb_autopark.checked or self.cb_autopark_par.checked
        ap_parallel = self.cb_autopark_par.checked
        return (self._build_cfg(), self.cb_parking.checked,
                ap_enabled, self._build_n_sub(), ap_parallel)

    def run(self) -> tuple:
        clock = pygame.time.Clock()
        while True:
            clock.tick(60)
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    pygame.quit()
                    raise SystemExit
                if ev.type == pygame.KEYDOWN:
                    if ev.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                        return self._return_values()
                    if ev.key == pygame.K_ESCAPE:
                        pygame.quit()
                        raise SystemExit
                if self.btn_go.clicked(ev):
                    return self._return_values()
                if self.btn_rst.clicked(ev):
                    for sl in self.sliders:
                        sl.reset()
                    self.s_dt_sub.reset()
                # Checkboxes — handle then enforce 3-way mutual exclusion
                prev_park = self.cb_parking.checked
                prev_auto = self.cb_autopark.checked
                prev_par  = self.cb_autopark_par.checked
                self.cb_parking.handle_event(ev)
                self.cb_autopark.handle_event(ev)
                self.cb_autopark_par.handle_event(ev)
                if self.cb_parking.checked and not prev_park:
                    self.cb_autopark.checked = False
                    self.cb_autopark_par.checked = False
                elif self.cb_autopark.checked and not prev_auto:
                    self.cb_parking.checked = False
                    self.cb_autopark_par.checked = False
                elif self.cb_autopark_par.checked and not prev_par:
                    self.cb_parking.checked = False
                    self.cb_autopark.checked = False
                for sl in self.sliders:
                    sl.handle_event(ev)
                if self.cb_autopark.checked or self.cb_autopark_par.checked:
                    self.s_dt_sub.handle_event(ev)
            self._draw()


def run_settings(screen: pygame.Surface, default_cfg: TruckConfig) -> tuple:
    """Show the settings screen and return (TruckConfig, parking_enabled, autopark_enabled, n_sub, ap_parallel)."""
    return SettingsScreen(screen, default_cfg).run()
