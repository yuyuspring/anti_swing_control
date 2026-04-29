# Anti-Swing Control

Modular C++ project for pendulum angle observation and LQR-based anti-swing control of slung-load systems.

## Project Structure

```
├── CMakeLists.txt
├── README.md
├── run_lqr.sh          # One-click closed-loop LQR pipeline
├── run_mpc.sh          # One-click MPC pipeline
├── run_replay.sh       # One-click observer replay validation
├── include/            # Public headers
│   ├── observer/       # Pendulum observer (legacy C core + C++ wrapper)
│   ├── controller/     # LQR controller
│   ├── dynamics/       # Slung-load dynamics simulation
│   ├── sensor/         # IMU sensor model
│   ├── simulation/     # Simulation engines
│   └── utils/          # CSV logger, math utilities
├── src/                # Implementations
│   ├── apps/           # Executable entry points
│   ├── observer/
│   ├── controller/
│   ├── dynamics/
│   ├── sensor/
│   ├── simulation/
│   └── utils/
├── scripts/            # Python scripts
│   ├── design/         # LQR gain design
│   ├── simulation/     # MPC simulation
│   ├── plot/           # Plotting scripts
│   └── analysis/       # Comparison & analysis
├── data/               # Input CSV data (crane IMU recordings)
├── results/            # Output CSV / plots / animations (gitignored)
├── docs/               # Documentation
└── build/              # CMake build directory (gitignored)
```

## Build

```bash
cd /home/hcy/work_space/pend_observer_test
mkdir -p build && cd build
cmake .. && make -j4
```

Three executables will be generated:
- `run_open_loop_sim`
- `run_csv_replay`
- `run_closed_loop_lqr`

---

## Quick Start Scripts

### `run_lqr.sh` — Closed-Loop LQR Simulation Pipeline

Runs the full simulation pipeline: compute LQR gains → build → run all 5 modes → generate plots/animation.

```bash
./run_lqr.sh                    # Full pipeline (all plots + animation)
./run_lqr.sh --comparison  -c   # Only comparison.png
./run_lqr.sh --brake       -b   # Only brake_phase.png
./run_lqr.sh --animation   -m   # Only lqr_brake_animation.mp4
./run_lqr.sh --help        -h   # Show usage
```

Outputs (in `results/lqr/`):
- `comparison.png` — 5-mode full-trajectory comparison
- `brake_phase.png` — 5-mode brake-phase comparison
- `lqr_brake_animation.mp4` — Animated brake-phase swing visualization

### `run_replay.sh` — Observer CSV Replay Validation

Replays recorded IMU data through the observer and validates against recorded outputs.

```bash
./run_replay.sh                    # Default input: data/crane_imu_obs_debug.csv
./run_replay.sh my_data.csv        # Specify custom input
```

Outputs (in `results/`):
- `replay_validation.csv` — Recorded vs replay comparison data
- `replay_validation.png` — 4-subplot validation visualization

---

## 1. Closed-Loop LQR Simulation (New)

Drone moves from position A to position B while suppressing payload swing using observer feedback and LQR control.

### Run

```bash
cd build
./run_closed_loop_lqr
```

Default parameters:
- Start position: `0 m`
- Target position: `50 m`
- Rope length: `15 m`
- Initial swing angle: `0.1 rad` (~5.7°)
- Max horizontal acceleration: `±2 m/s²`
- Total duration: `40 s`

### Custom Parameters

```bash
./run_closed_loop_lqr <mode> [initial_theta] [rope_length] [duration]

# Modes: 0=Full, 1=Shortest, 2=MinSwing, 3=VelocityOmega, 4=PayloadVelocity, 5=MinEnergy
# Example: MinEnergy mode, initial angle 5° (0.087 rad), rope 10m, 20s
./run_closed_loop_lqr 5 0.087 10.0 20.0
```

### Plot Results

```bash
python3 ../scripts/plot/plot_closed_loop.py closed_loop_results.csv
```

The plot window will stay open. **Close it manually when you are done.**

---

## 2. Open-Loop Simulation (Legacy)

Simulates three scenarios (pure pitch, pure roll, mixed) where a platform cruises at constant speed then decelerates, causing the suspended payload to swing. Observer runs on synthetic sensor data.

### Run

```bash
cd build
./run_open_loop_sim
```

Generates (in `results/`):
- `simulation_results.csv`        (pure pitch)
- `simulation_results_roll.csv`   (pure roll)
- `simulation_results_mixed.csv`  (mixed pitch + roll)

### Plot Results

```bash
python3 ../scripts/plot/plot_results.py results/simulation_results.csv results/simulation_results.png
```

The plot window will stay open. **Close it manually when you are done.**

---

## 3. CSV Replay (Legacy)

Replays recorded flight/crane IMU data through the observer and validates against recorded observer outputs.

### Run

```bash
cd build
./run_csv_replay ../data/crane_imu_obs_debug.csv
```

Generates (in `results/`):
- `replay_validation.csv`

### Plot Results

```bash
python3 ../scripts/plot/plot_replay_results.py results/replay_validation.csv results/replay_validation.png
```

The plot window will stay open. **Close it manually when you are done.**

---

## LQR Gain Tuning

Edit `scripts/design/compute_lqr_gain.py` to adjust LQR weights, then regenerate the gain header:

```bash
cd /home/hcy/work_space/pend_observer_test
python3 scripts/design/compute_lqr_gain.py --q-theta 30.0 --q-omega 10.0 --r-ax 1.0
```

Recompile:

```bash
cd build && make -j4
```

---

## Key Design Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `Ts` | `0.02 s` | Observer / control period |
| `g` | `9.81 m/s²` | Gravity |
| `L` | `15 m` | Rope length |
| `ζ` | `0.06` | Damping ratio |
| `a_max` | `±2 m/s²` | Drone acceleration limit |

## Coordinate Frames

- **FRD**: Forward-Right-Down (body frame, IMU measurements)
- **NEU**: North-East-Up (navigation frame, velocity/position)
- **Euler angles**: `[yaw, pitch, roll]` in radians (Z-Y-X order)
- **Angular rates**: `[roll_rate, pitch_rate, yaw_rate]` in rad/s

> **Note**: The legacy observer interface `pend_observer_iterate_2` expects gyro input in **deg/s** and yaw input in **deg**. The C++ wrapper `PendulumObserver` handles all SI-unit conversions internally.
