# EuroTruck 3.0 — 2D Kinematic Truck-Trailer Simulator

A real-time 2D driving simulation of an articulated truck and trailer, implemented in Python and pygame. The vehicle dynamics are derived directly from the kinematic vehicle-trailer model presented in Cao et al. (2026), using forward Euler integration to update the full 6-DOF state at every simulation tick.

---

## Features

- **Paper-accurate kinematics** — equations (1)–(6) from the reference paper govern truck heading, trailer heading, and hitch-point constraint
- **Interactive settings screen** — configure vehicle and cargo geometry with sliders before each run; a live top-down preview reflects changes in real time
- **WASD driving controls** — throttle and steering with realistic inertia and auto-centering
- **Jackknife detection** — color-coded HUD warning at 60°, speed lock at 85° articulation angle
- **Camera follow** — the viewport tracks the truck's rear axle so the vehicle always stays on screen
- **RK4 integrator included** — `step_rk4()` is available in `kinematics.py` as a drop-in replacement for higher accuracy at speed

---

## Kinematic Model

The system state is a 6-vector:

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

Control inputs `(δf, VR)` map directly to WASD keys.

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

---

## Default Vehicle Parameters

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

All parameters are adjustable in the settings screen before each simulation run.

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

The settings screen opens first. Adjust sliders as desired, then press **Enter** or click **Start Simulation**.

---

## Controls

| Key | Action |
|-----|--------|
| `W` | Accelerate forward |
| `S` | Accelerate backward |
| `A` | Steer left |
| `D` | Steer right |
| `R` | Reset to initial state |
| `Esc` | Quit |

Releasing `W`/`S` applies friction that coasts the speed to zero. Releasing `A`/`D` auto-centers the steering.

---

## Project Structure

```
EuroTruck3.0/
├── requirements.txt
├── TRUCK_SIM_PLAN.md          # Design document (Chinese)
└── truck_sim/
    ├── main.py                # Entry point and main loop
    ├── config.py              # TruckConfig / SimConfig dataclasses
    ├── kinematics.py          # State vector and Euler/RK4 integrator
    ├── input_handler.py       # WASD input with inertia
    ├── renderer.py            # pygame rendering (grid, bodies, hitch, HUD)
    ├── hud.py                 # Overlay dashboard (speed, steer, hitch angle)
    └── settings_screen.py     # Pre-simulation slider-based configuration UI
```

---

## HUD Indicators

| Field | Description |
|-------|-------------|
| Speed | Current `VR` in m/s; positive = forward, negative = reverse |
| Steer | Front-wheel angle `δf` in degrees |
| Hitch | Articulation angle `Δψ` in degrees, color-coded by severity |
| State | `NORMAL` / `WARNING` (>60°) / `JACKKNIFE` (>85°, speed locked) |

Hitch angle color scale:

| Range | Color |
|-------|-------|
| \|Δψ\| < 40° | Green |
| 40° – 60° | Yellow |
| 60° – 80° | Orange |
| ≥ 80° | Red (blinking) |

---

## Coordinate Convention

Consistent with the reference paper:

- World frame: +x right, +y up, angles measured counter-clockwise from +x
- Screen frame: +x right, +y down (pygame default)
- Conversion: `screen_y = screen_center_y − world_y × pixels_per_m`
- Initial pose: truck rear axle at origin, heading along +x (facing right)

---

## Reference

Cao et al. (2026). *Kinematic modeling and hybrid A\* path planning for truck-trailer parking.*
