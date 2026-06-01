# 2D 卡車拖車駕駛模擬——專案規劃

## 一、專案概述

本專案目標是實作一個基於運動學的 2D 卡車拖車即時駕駛模擬，玩家可透過 WASD 鍵盤操控車頭的油門與轉向，系統依據論文所推導的運動學方程式（Cao et al., 2026）在每個 tick 更新車頭與拖車的完整姿態，並以 pygame 即時渲染。

**技術棧**：Python 3.11 · pygame 2.x · NumPy（選用，加速向量運算）

**核心特性**：

- 基於論文方程式 (1)–(6) 的 kinematic vehicle-trailer 前向積分
- 鉸接角即時顯示，超過警告閾值時顯示視覺提示
- 可調整車頭與拖車參數（LH、LT、軸距等）
- 地面網格 + 座標軸，幫助感知位移與轉向

---

## 二、運動學模型回顧

### 2.1 狀態向量

系統擁有 6 個狀態變數：

```
state = (xR, yR, ψ₁, xT, yT, ψ₂)
```

| 符號 | 意義 |
|---|---|
| `xR, yR` | 車頭後軸中心位置 (m) |
| `ψ₁` | 車頭偏航角 (rad) |
| `xT, yT` | 拖車軸心位置 (m) |
| `ψ₂` | 拖車偏航角 (rad) |
| `Δψ = ψ₁ − ψ₂` | 鉸接角（折刀指標） |

### 2.2 控制輸入

```
inputs = (δf, VR)
```

| 符號 | 意義 | 玩家對應鍵 |
|---|---|---|
| `δf` | 前輪轉角 (rad) | A（左）/ D（右） |
| `VR` | 後軸速度 (m/s，正=前進，負=後退) | W（前進）/ S（後退） |

### 2.3 前向運動學方程式（直接來自論文）

車頭動態（equations 1–3）：

```
ẋR = VR · cos(ψ₁)
ẏR = VR · sin(ψ₁)
ψ̇₁ = VR / L · tan(δf)
```

拖車動態（equations 4–6）：

```
ẋT = VR · cos(ψ₂) · [cos(Δψ) + (LH/L) · sin(Δψ) · tan(δf)]
ẏT = VR · sin(ψ₂) · [cos(Δψ) + (LH/L) · sin(Δψ) · tan(δf)]
ψ̇₂ = VR / LT · [sin(Δψ) − (LH/L) · cos(Δψ) · tan(δf)]
```

### 2.4 折刀條件

```
|Δψ| ≥ 90°  →  系統進入不可控的折刀狀態
```

建議設定軟性警告閾值（如 60°）提前顯示警示。

---

## 三、參考參數

來自論文 Table 2 及 Table 5，可作為預設值：

| 參數 | 符號 | 預設值 |
|---|---|---|
| 車頭軸距 | L | 2.896 m |
| 後軸至鉸接距離 | LH | 1.159 m |
| 拖車軸至鉸接距離 | LT | 2.693 m |
| 車頭長度 | — | 5.046 m |
| 拖車長度 | — | 3.84 m |
| 車頭寬度 | — | 1.935 m |
| 拖車寬度 | — | 1.63 m |
| 最大前輪轉角 | δf,max | 0.75 rad（≈ 43°） |
| 初始速度 | VR | 0 m/s |

---

## 四、架構與模組分工

```
truck_sim/
├── main.py               # 入口：初始化 pygame，執行主迴圈
├── config.py             # TruckConfig、SimConfig dataclass
├── kinematics.py         # TruckTrailerState + 前向積分器
├── input_handler.py      # WASD 鍵盤輸入 → (δf, VR)
├── renderer.py           # pygame 渲染：地面、車體、HUD
├── hud.py                # 儀表板文字資訊（速度、鉸接角、轉角）
└── TRUCK_SIM_PLAN.md     # 本規劃文件
```

### 4.1 `config.py`

```python
@dataclass
class TruckConfig:
    L:   float = 2.896    # 軸距 (m)
    LH:  float = 1.159    # 後軸至鉸接 (m)
    LT:  float = 2.693    # 拖車軸至鉸接 (m)
    truck_length: float = 5.046
    truck_width:  float = 1.935
    trailer_length: float = 3.84
    trailer_width:  float = 1.63
    max_steer: float = 0.75   # rad
    max_speed: float = 8.0    # m/s
    accel:     float = 3.0    # m/s² (油門加速度)
    steer_rate: float = 1.2   # rad/s（轉向速率）

@dataclass
class SimConfig:
    fps:          int   = 60
    dt:           float = 1 / 60      # 積分步長（s）
    pixels_per_m: float = 40.0        # 渲染比例
    grid_spacing: float = 5.0         # 地面網格間距 (m)
    jackknife_warn_deg:  float = 60.0
    jackknife_limit_deg: float = 85.0
    window_w: int = 1280
    window_h: int = 720
```

### 4.2 `kinematics.py`

**核心職責**：維護狀態向量，每 tick 呼叫 `step()` 前向積分。

```python
@dataclass
class TruckTrailerState:
    xR: float;  yR: float;  psi1: float  # 車頭後軸
    xT: float;  yT: float;  psi2: float  # 拖車軸心

    @property
    def hitch_angle_deg(self) -> float:
        return math.degrees(self.psi1 - self.psi2)

class TruckTrailerKinematics:
    def __init__(self, cfg: TruckConfig): ...

    def step(self, state: TruckTrailerState,
             delta_f: float, vR: float, dt: float) -> TruckTrailerState:
        """
        Euler 積分，一次 dt。
        傳入的 delta_f 已 clamp 至 [-max_steer, max_steer]。
        """
        dpsi1 = vR / L * math.tan(delta_f)
        dxR   = vR * math.cos(state.psi1)
        dyR   = vR * math.sin(state.psi1)

        dpsi  = state.psi1 - state.psi2
        dxT   = vR * math.cos(state.psi2) * (
                    math.cos(dpsi) + (LH/L) * math.sin(dpsi) * math.tan(delta_f))
        dyT   = vR * math.sin(state.psi2) * (
                    math.cos(dpsi) + (LH/L) * math.sin(dpsi) * math.tan(delta_f))
        dpsi2 = vR / LT * (
                    math.sin(dpsi) - (LH/L) * math.cos(dpsi) * math.tan(delta_f))

        return TruckTrailerState(
            xR  = state.xR  + dxR   * dt,
            yR  = state.yR  + dyR   * dt,
            psi1= state.psi1 + dpsi1 * dt,
            xT  = state.xT  + dxT   * dt,
            yT  = state.yT  + dyT   * dt,
            psi2= state.psi2 + dpsi2 * dt,
        )
```

**積分精度說明**：對於低速停車場景（≤ 10 m/s），Euler 積分在 dt = 1/60 s 下已足夠精確。若未來需要更高速或更長時間模擬，可升級為 RK4：

```python
def step_rk4(self, state, delta_f, vR, dt) -> TruckTrailerState:
    # k1
    k1 = self._derivatives(state, delta_f, vR)
    # k2
    s2 = self._apply(state, k1, dt/2)
    k2 = self._derivatives(s2, delta_f, vR)
    # k3
    s3 = self._apply(state, k2, dt/2)
    k3 = self._derivatives(s3, delta_f, vR)
    # k4
    s4 = self._apply(state, k3, dt)
    k4 = self._derivatives(s4, delta_f, vR)
    # combine
    return self._apply(state, {
        k: (k1[k] + 2*k2[k] + 2*k3[k] + k4[k]) / 6
        for k in k1
    }, dt)
```

### 4.3 `input_handler.py`

**職責**：把 pygame key state 轉換成連續的 `(δf, VR)`，模擬真實車輛的慣性感。

```python
class InputHandler:
    def __init__(self, cfg: TruckConfig):
        self._speed   = 0.0   # 當前速度（帶慣性）
        self._steer   = 0.0   # 當前轉角（帶慣性）

    def update(self, keys, dt: float) -> tuple[float, float]:
        # 油門：W/S 加速，放開時自然減速
        if keys[K_w]:
            self._speed = min(self._speed + cfg.accel * dt, cfg.max_speed)
        elif keys[K_s]:
            self._speed = max(self._speed - cfg.accel * dt, -cfg.max_speed)
        else:
            # 滑行摩擦：速度朝 0 收斂
            self._speed *= (1 - 2.0 * dt)
            if abs(self._speed) < 0.05:
                self._speed = 0.0

        # 轉向：A/D 旋轉，放開時回正
        if keys[K_a]:
            self._steer = max(self._steer - cfg.steer_rate * dt, -cfg.max_steer)
        elif keys[K_d]:
            self._steer = min(self._steer + cfg.steer_rate * dt, cfg.max_steer)
        else:
            # 自動回正（模擬方向盤回正力矩）
            self._steer *= (1 - 5.0 * dt)
            if abs(self._steer) < 0.01:
                self._steer = 0.0

        return self._steer, self._speed
```

**設計考量**：
- 速度帶慣性而非瞬間切換，避免模擬出「瞬間剎車/起步」的不真實感
- 轉向加入回正力矩，讓直線行駛時不需要刻意保持中立
- 兩者的時間常數（`2.0` 和 `5.0`）可在 `TruckConfig` 中公開為可調參數

### 4.4 `renderer.py`

渲染以**世界座標系**為主，攝影機跟隨車頭（camera follow），保持車頭始終在畫面中央附近。

```
world_to_screen(xw, yw):
    sx = (xw - cam_x) * ppm + screen_cx
    sy = screen_cy - (yw - cam_y) * ppm   # y 軸翻轉（世界 +y 向上，螢幕 +y 向下）
    return (sx, sy)
```

渲染層次（由下至上）：

1. **地面網格**：以灰色細線繪製，間距 `grid_spacing` 公尺
2. **拖車本體**：以後軸中心 + 偏航角計算四個角點，繪製矩形
3. **鉸接連桿**：從車頭後軸到拖車軸的線段，中間標出鉸接點 H（圓點）
4. **車頭本體**：同上，疊在拖車上方
5. **前輪轉角指示**：在車頭前方繪製一個小的方向箭頭，可視化 δf
6. **HUD 疊加層**：速度計、鉸接角錶、警示文字

**車體角點計算（世界座標）**：

```python
def rect_corners(cx, cy, yaw, length, width):
    """以 (cx, cy) 為軸心，計算車體四角世界座標。"""
    half_l = length / 2
    half_w = width / 2
    corners_local = [
        (-half_l, -half_w), (-half_l, half_w),
        ( half_l, half_w),  ( half_l, -half_w),
    ]
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    return [
        (cx + x * cos_y - y * sin_y,
         cy + x * sin_y + y * cos_y)
        for x, y in corners_local
    ]
```

注意：車頭的「軸心」是後軸中心 `(xR, yR)`，車頭往前延伸 `LF`，往後延伸 `LR`（`LF + LR = truck_length`，`LH` 在後軸之後，需從幾何上分開計算）。

### 4.5 `hud.py`

左上角儀表板顯示：

| 欄位 | 說明 |
|---|---|
| Speed | 當前 VR（m/s），正/負顯示前進/後退 |
| Steer δf | 前輪轉角（度），向左負向右正 |
| Hitch Δψ | 鉸接角（度），顏色隨大小變化 |
| State | NORMAL / WARNING（>60°）/ JACKKNIFE（>85°）|

鉸接角儀錶顏色映射：

```
|Δψ| < 40°   → 綠色
|Δψ| < 60°   → 黃色
|Δψ| < 80°   → 橙色
|Δψ| ≥ 80°  → 紅色閃爍（每 0.5s toggle）
```

---

## 五、主迴圈設計（`main.py`）

```python
def main():
    pygame.init()
    screen = pygame.display.set_mode((cfg.window_w, cfg.window_h))
    clock  = pygame.time.Clock()

    state   = initial_state()       # 初始化在畫面中央，朝右（ψ₁ = 0）
    handler = InputHandler(truck_cfg)
    kin     = TruckTrailerKinematics(truck_cfg)
    ren     = Renderer(screen, truck_cfg, sim_cfg)

    running = True
    while running:
        dt = clock.tick(sim_cfg.fps) / 1000.0   # 實際幀時間（s）
        dt = min(dt, 0.05)                       # 防止 lag spike 導致積分發散

        for event in pygame.event.get():
            if event.type == QUIT: running = False
            if event.type == KEYDOWN:
                if event.key == K_r: state = initial_state()   # R 重置
                if event.key == K_ESCAPE: running = False

        keys = pygame.key.get_pressed()
        delta_f, vR = handler.update(keys, dt)

        # 折刀防護：若已超過硬限制，鎖定速度為 0
        if abs(state.hitch_angle_deg) >= sim_cfg.jackknife_limit_deg:
            vR = 0.0

        state = kin.step(state, delta_f, vR, dt)

        ren.draw(state, delta_f, vR)
        pygame.display.flip()

    pygame.quit()
```

**折刀防護**：一旦 `|Δψ|` 超過硬限制，自動鎖速為 0，視覺上呈現「卡住」的狀態，並顯示 JACKKNIFE 警示，需玩家主動反向修正（放開 W/S）才能恢復。

---

## 六、座標系約定

與論文一致：

- **世界座標**：+x 向右，+y 向上，角度 θ 以 +x 軸為 0°，逆時針為正
- **螢幕座標**：+x 向右，+y 向下（pygame 預設）
- 轉換：`screen_y = screen_center_y - world_y * ppm`
- 車頭初始姿態：`(xR=0, yR=0, ψ₁=0)` → 朝向畫面右方

---

## 七、初始狀態計算

拖車初始位置需由車頭初始位置和鉸接幾何反向計算，確保兩者一開始就連接在一起（Δψ = 0）：

```python
def initial_state(cfg: TruckConfig, psi1_init=0.0) -> TruckTrailerState:
    xR, yR = 0.0, 0.0
    psi1   = psi1_init

    # 鉸接點 H 在車頭後軸往後 LH
    xH = xR - cfg.LH * math.cos(psi1)
    yH = yR - cfg.LH * math.sin(psi1)

    # 拖車軸在 H 往後 LT（初始 Δψ = 0，故拖車與車頭同向）
    xT = xH - cfg.LT * math.cos(psi1)
    yT = yH - cfg.LT * math.sin(psi1)
    psi2 = psi1

    return TruckTrailerState(xR, yR, psi1, xT, yT, psi2)
```

---

## 八、鉸接點 H 的即時計算（渲染用）

H 點不是狀態變數，但渲染時需要繪製鉸接連桿：

```python
def hitch_point(state: TruckTrailerState, cfg: TruckConfig):
    xH = state.xR - cfg.LH * math.cos(state.psi1)
    yH = state.yR - cfg.LH * math.sin(state.psi1)
    return xH, yH
```

---

## 九、實作順序（建議）

遵循「先跑起來再完善」的原則：

| 階段 | 目標 | 驗收標準 |
|---|---|---|
| **Phase 1** | 車頭單獨行駛 | WASD 可控制車頭移動，pygame 視窗顯示矩形 |
| **Phase 2** | 加入拖車 | 拖車跟隨車頭移動，鉸接點可見 |
| **Phase 3** | 完整 HUD | 速度、鉸接角、轉角即時顯示 |
| **Phase 4** | 折刀警示 | |Δψ| 大時顏色警示，超限時鎖速 |
| **Phase 5** | 攝影機跟隨 | 車頭不會離開畫面 |
| **Phase 6** | 參數調整 UI | 可在 Settings 視窗調整 LH、LT 等參數 |

---

## 十、已知邊界情況與處理方式

| 情況 | 問題 | 處理方式 |
|---|---|---|
| `cos(Δψ) + (LH/L)·sin(Δψ)·tan(δf) ≈ 0` | 方程式分母趨近於零（奇異點） | 加 epsilon guard；此情況對應 ψ̇₂ 趨向無窮，即折刀臨界點，應在此之前就已觸發折刀鎖速 |
| `dt` 過大（如視窗失焦後恢復） | 積分一步飛出 | `dt = min(dt, 0.05)` 硬性限制，對應最大每幀 20 fps 等效積分步長 |
| 速度非常小時的數值噪聲 | `ψ̇` 積累誤差 | 設 `if abs(vR) < 0.01: vR = 0.0` 死區，靜止時完全停止積分 |
| `psi1`、`psi2` 繞圈 | 角度超過 2π | 每步後 `psi = (psi + π) % (2π) - π` 歸一化至 `[-π, π]` |

---

## 十一、Dependencies

```
# requirements.txt
pygame>=2.0
numpy>=1.21   # 選用：加速批次座標轉換
```

安裝：

```bash
conda activate car   # 沿用既有環境
pip install -r requirements.txt
```

---

## 十二、未來擴充方向

本次 Phase 6 之後可考慮的擴充（不在本次 scope 內）：

- **逆運動學模式**：切換為「控制拖車虛擬轉角 δT」的操作方式，作為 RL 訓練環境的人工示範資料收集工具
- **路徑錄製與回放**：記錄狀態序列，用於 RL 的 imitation learning
- **障礙物與停車格**：加入靜態障礙物碰撞偵測，銜接 `trailer_hybrid_astar` 的地圖格式
- **多拖車串接**：將運動學推廣至 n 節拖車（每節加一個方程組）
- **3D 渲染升級**：以 OpenGL 或 Pygame-CE 加入俯視透視效果
