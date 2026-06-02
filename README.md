# EuroTruck 3.0 — 2D Kinematic Truck-Trailer Simulator

A real-time 2D driving simulation of an articulated truck and trailer, implemented in Python and pygame. Vehicle dynamics are derived directly from the kinematic model in Cao et al. (2026), using forward Euler integration to update the full 6-DOF state at every tick.

---

## Features

- **Paper-accurate kinematics** — equations (1)–(6) govern truck heading, trailer heading, and hitch-point constraint
- **Pre-simulation setup screen** — adjust all vehicle geometry with sliders; a live top-down preview updates in real time; optional parking challenge toggle
- **WASD driving** — throttle and steering with realistic inertia and steering auto-centering
- **Mouse scroll wheel zoom** — zoom in/out during simulation; current scale shown in HUD
- **Jackknife detection** — color-coded HUD: warning at 60°, speed lock at 85° articulation angle
- **Parking challenge mode** — randomly spawned parking spot with screen-edge direction arrow and success detection

---

## Kinematic Model

State vector (6 variables):

```
(xR, yR, ψ₁, xT, yT, ψ₂)
```

| Symbol | Meaning |
|--------|---------|
| `xR, yR` | Truck rear-axle position (m) |
| `ψ₁` | Truck yaw angle (rad) |
| `xT, yT` | Trailer axle position (m) |
| `ψ₂` | Trailer yaw angle (rad) |
| `Δψ = ψ₁ − ψ₂` | Articulation (hitch) angle |

Control inputs `(δf, VR)` map directly to WASD.

**Truck dynamics (eqs. 1–3):**
```
ẋR  = VR · cos(ψ₁)
ẏR  = VR · sin(ψ₁)
ψ̇₁ = VR / L · tan(δf)
```

**Trailer dynamics (eqs. 4–6):**
```
ẋT  = VR · cos(ψ₂) · [cos(Δψ) + (LH/L) · sin(Δψ) · tan(δf)]
ẏT  = VR · sin(ψ₂) · [cos(Δψ) + (LH/L) · sin(Δψ) · tan(δf)]
ψ̇₂ = VR / LT · [sin(Δψ) − (LH/L) · cos(Δψ) · tan(δf)]
```

Integration uses Euler (dt = 1/60 s). A drop-in `step_rk4()` is available in `kinematics.py` for higher accuracy.

---

## Default Vehicle Parameters

From paper Table 2 / Table 5:

| Parameter | Symbol | Default |
|-----------|--------|---------|
| Wheelbase | L | 2.896 m |
| Rear axle to hitch | LH | 1.159 m |
| Trailer axle to hitch | LT | 2.693 m |
| Truck length | — | 5.046 m |
| Truck width | — | 1.935 m |
| Trailer length | — | 3.840 m |
| Trailer width | — | 1.630 m |
| Max steer angle | δf,max | 0.75 rad (~43°) |
| Max speed | VR,max | 8.0 m/s |

All parameters are adjustable in the settings screen.

---

## Installation

```bash
pip install -r requirements.txt
```

Requires **Python 3.11+** and **pygame >= 2.0**.

---

## Running

```bash
cd truck_sim
python main.py
```

The settings screen opens first. Adjust sliders, optionally enable the parking challenge, then press **Enter** or **Start Simulation ▶**.

---

## Controls

### Settings Screen

| Action | Effect |
|--------|--------|
| Drag slider | Adjust parameter; live preview updates instantly |
| Click anywhere on track | Jump slider to that value |
| `[ ] Enable Parking Challenge` | Toggle parking mode for this run |
| `Reset Defaults` | Restore all sliders to paper values |
| `Enter` / **▶** | Start simulation |
| `Esc` | Quit |

### Simulation

| Key / Input | Action |
|-------------|--------|
| `W` | Accelerate forward |
| `S` | Accelerate backward |
| `A` | Steer left |
| `D` | Steer right |
| Scroll wheel ↑ / ↓ | Zoom in / out (5–300 px/m) |
| `R` | Reset vehicle to initial state |
| `Esc` | Quit |

Releasing `W`/`S` applies rolling friction. Releasing `A`/`D` auto-centers the steering wheel.

---

## HUD Indicators

### Main panel (top-left)

| Field | Description |
|-------|-------------|
| Speed | `VR` in m/s; positive = forward, negative = reverse |
| Steer | Front-wheel angle `δf` in degrees |
| Hitch | Articulation angle `Δψ` in degrees, color-coded |
| Zoom | Current scale relative to default (1.00×) |
| State | `NORMAL` / `WARNING` / `JACKKNIFE` |

Hitch angle color scale:

| Range | Color |
|-------|-------|
| \|Δψ\| < 40° | Green |
| 40° – 60° | Yellow |
| 60° – 80° | Orange |
| ≥ 80° | Red (blinking) |

### Parking panel (top-right, parking mode only)

| Field | Description |
|-------|-------------|
| Park | Distance to parking spot centre (m) |
| Parked | Number of successful parks this session |

---

## Parking Challenge

Enable **Parking Challenge** in the settings screen to activate this mode.

**Spot generation**
- Spawns 30–65 m from the vehicle at a random angle, snapped to the 5 m world grid
- Orientation is 0° or 90° (axis-aligned), chosen randomly each spawn
- Size: `(truck_length + trailer_length + 2.5 m) × (max_width + 1.5 m)`

**Direction indicator**
- When the spot is **off-screen**: a red triangle arrow appears on the screen edge pointing toward the spot, with the distance in metres
- When the spot is **on-screen**: the arrow disappears; the spot outline is directly visible

**Parking spot colours**

| State | Border | Fill |
|-------|--------|------|
| Default | Dashed yellow | Faint yellow |
| Vehicle partially inside | Bright yellow | Light yellow |
| Parked (success) | Solid green | Semi-transparent green |

**Success condition**
All 8 body corners of the truck+trailer are inside the spot **and** speed ≤ 0.5 m/s.

On success, a **PARKED!** banner appears at the screen centre for 3 seconds, then a new spot spawns.

---

## Project Structure

```
EuroTruck3.0/
├── requirements.txt
├── TRUCK_SIM_PLAN.md          # Original design document
└── truck_sim/
    ├── main.py                # Entry point and main loop
    ├── config.py              # TruckConfig / SimConfig dataclasses
    ├── kinematics.py          # State vector, Euler and RK4 integrators
    ├── input_handler.py       # WASD input with inertia model
    ├── settings_screen.py     # Pre-simulation slider UI + parking toggle
    ├── renderer.py            # pygame rendering: grid, bodies, parking, zoom
    ├── hud.py                 # Overlay dashboard and parking banners
    └── parking.py             # Parking spot geometry, spawning, success detection
```

---

## Coordinate Convention

Consistent with the reference paper:

- World frame: +x right, +y up, angles counter-clockwise from +x
- Screen frame: +x right, +y down (pygame default)
- Conversion: `screen_y = screen_center_y − world_y × ppm`
- Initial pose: truck rear axle at world origin, heading along +x

---

## Reference

Cao et al. (2026). *Kinematic modeling and hybrid A\* path planning for truck-trailer parking.*
