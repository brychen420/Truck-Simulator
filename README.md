# EuroTruck 3.0 — 2D Kinematic Truck-Trailer Simulator

A real-time 2D driving simulation of an articulated truck and trailer, implemented in Python and pygame. Vehicle dynamics follow the kinematic model in Cao et al., using RK4 integration. Includes a full Hybrid A* auto-parking planner with inverse kinematics.

---

## Features

- **Paper-accurate kinematics** — equations (1)–(6) govern truck heading, trailer heading, and hitch-point constraint; integrated with RK4
- **Pre-simulation setup screen** — adjust all vehicle geometry with sliders; live top-down preview scales with window width
- **Resizable window** — drag to any size during both settings and simulation; minimum 1000 × 600
- **WASD driving** — throttle and steering with realistic inertia and auto-centering
- **Mouse scroll wheel zoom** — zoom in/out during simulation; current scale shown in HUD
- **Pause** — `Space` freezes simulation and overlays a banner; all other keys are suppressed
- **Jackknife detection** — color-coded HUD: warning at 60°, limit at 85° articulation angle
- **Parking challenge mode** — randomly spawned parking spot with screen-edge direction arrow and success detection
- **Hybrid A\* auto-parking (perpendicular)** — reverse into a fixed alley slot; ghost path visualization
- **Hybrid A\* auto-parking (parallel)** — roadside parallel parking; goal condition requires all 8 body corners inside the spot

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

Control inputs `(δf, VR)` — front-wheel steer angle and rear-axle speed.

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

Integration uses RK4 (dt = 1/60 s for simulation; dt = 0.2 s per sub-step during planning).

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

The settings screen opens first. Adjust sliders, optionally enable a mode, then press **Enter** or **Start Simulation ▶**.

---

## Controls

### Settings Screen

| Action | Effect |
|--------|--------|
| Drag slider | Adjust parameter; live preview updates instantly |
| Click anywhere on track | Jump slider to that value |
| `[ ] Enable Parking Challenge` | Toggle random parking challenge mode |
| `[ ] Enable Auto-Park Scene (Hybrid A*)` | Toggle perpendicular auto-parking |
| `[ ] Enable Auto-Park (Parallel) (Hybrid A*)` | Toggle parallel roadside auto-parking |
| `Reset Defaults` | Restore all sliders to paper values |
| `Enter` / **▶** | Start simulation |
| `Esc` | Quit |

### Simulation — Manual

| Key / Input | Action |
|-------------|--------|
| `W` | Accelerate forward |
| `S` | Accelerate backward |
| `A` | Steer left |
| `D` | Steer right |
| `Space` | Pause / resume (freezes physics and input; zoom and resize still work) |
| Scroll wheel ↑ / ↓ | Zoom in / out (5–300 px/m) |
| `R` | Reset vehicle to initial position |
| `Esc` | Quit |

Releasing `W`/`S` applies rolling friction. Releasing `A`/`D` auto-centers the steering wheel.

### Simulation — Auto-Park Mode

| Key | Action |
|-----|--------|
| `P` | Start planning (from MANUAL or DONE/FAILED state) |
| `Space` | Pause / resume (freezes execution; zoom and resize still work) |
| `Esc` | Cancel planning / abort execution → return to manual |
| `W` / `S` / `A` / `D` | Abort execution mid-path → return to manual |
| `L` | Replay last planned path from the start position |
| `R` | Reset vehicle to scene start position |

---

## HUD Indicators

### Main panel (top-left)

| Field | Description |
|-------|-------------|
| Speed | `VR` in m/s; positive = forward, negative = reverse |
| Steer | Front-wheel angle `δf` in degrees |
| Hitch | Articulation angle `Δψ` in degrees, color-coded |
| Zoom | Current scale relative to default (1.00×) |
| State | `NORMAL` / `WARNING` / `!!! JACKKNIFE !!!` |

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

### Auto-park panel (bottom-right, auto-park mode only)

| State | Display |
|-------|---------|
| Planning | Yellow text + animated dots; full-screen dark overlay |
| Executing | Blue text with step counter and progress bar |
| Done | Green "Parked!" text; `L: replay  P: re-plan` hints |
| Failed | Red "No path found" text |

---

## Parking Challenge

Enable **Parking Challenge** in the settings screen.

- Spot spawns 30–65 m away at a random angle, snapped to the 5 m world grid
- When the spot is off-screen: a red triangle arrow appears on the screen edge with distance in metres
- **Success condition**: all 8 body corners inside the spot **and** speed ≤ 0.5 m/s
- On success, a **PARKED!** banner appears for 3 seconds, then a new spot spawns

---

## Auto-Park Mode (Hybrid A*)

Two scenes are available; select one in the settings screen before starting.

Press `P` to run the planner in a background thread. The simulation remains interactive during planning. When planning completes, the vehicle is snapped back to the exact start state before execution begins (eliminates drift from input-handler inertia during the planning phase).

### Scenes

| Scene | Setting | Starting pose | Goal condition |
|-------|---------|---------------|----------------|
| **Perpendicular** | Enable Auto-Park Scene | Rear axle at (−10, 8), heading east | Trailer axle within 0.8 m of goal, ψ₂ within 15°, Δψ within 10° |
| **Parallel** | Enable Auto-Park (Parallel) | East of slot, heading east, in travel lane | All 8 body corners inside the parking spot |

### Planner

| Property | Value |
|----------|-------|
| State space | 4D trailer-centric `(xT, yT, ψ₂, Δψ)` |
| Controls | Virtual trailer steer δT × forward/reverse, converted to `(δf, VR)` via inverse kinematics |
| Integration | RK4, 5 sub-steps per node (DT_SUB = 0.2 s, horizon = 1.0 s) |
| Jackknife limit | \|Δψ\| ≤ 55° during planning |
| Adaptive δT range | Per-node feasible steer range derived from paper §4.3 Eqs. 15–16 |
| Max expansions | 100 000 nodes |

### Planning Time Step

The planner uses a fixed sub-step of `DT_SUB = 0.2 s` (`N_SUB = 5` per node, total horizon 1.0 s per expansion). Path execution integrates each planned step with the same `DT_SUB`, so the replayed trajectory exactly matches the planned one.

### Ghost Path Visualization

Once a path is found, ghost outlines are drawn along the planned trajectory:
- **Blue** outlines — truck
- **Orange** outlines — trailer
- Thin connector lines show the hitch linkage at each sampled pose

---

## Project Structure

```
EuroTruck3.0/
├── requirements.txt
└── truck_sim/
    ├── main.py                  # Entry point — thin main loop, delegates to modules
    ├── config.py                # TruckConfig / SimConfig dataclasses
    ├── kinematics.py            # State vector, RK4 integrator, hitch geometry
    ├── input_handler.py         # WASD input with inertia model
    ├── settings_screen.py       # Pre-simulation slider UI (resizable; preview panel scales)
    ├── renderer.py              # pygame rendering: grid, bodies, ghost path, parking
    ├── hud.py                   # Overlay dashboard, auto-park status, parking banners, pause overlay
    ├── parking.py               # Parking spot geometry, spawning, success detection
    ├── autopark_scene.py        # Perpendicular alley scene geometry and goal state
    ├── parallel_park_scene.py   # Parallel (roadside) parking scene geometry and goal state
    ├── autopark_state.py        # APMode enum, AutoParkState, state machine logic
    ├── hybrid_astar.py          # Hybrid A* planner (background thread)
    ├── inverse_kinematics.py    # Virtual trailer steer → physical (δf, VR)
    └── auto_park_controller.py  # Path replay controller (supplies control inputs per frame)
```

---

## Coordinate Convention

- World frame: +x right (east), +y up (north), angles counter-clockwise from +x
- Screen frame: +x right, +y down (pygame default)
- Conversion: `screen_y = screen_center_y − world_y × ppm`

| Scene | Initial truck rear-axle position | Heading |
|-------|----------------------------------|---------|
| Perpendicular | (−10, 8) | East (ψ = 0) |
| Parallel | (~16, ~3.5) — east of slot | East (ψ = 0) |

---

## Reference

Cao et al. *Hybrid A\*-Based Reverse Path-Planning of a Vehicle with Single Trailer.*
