# Anti-Swing Control

Modular C++ project for pendulum angle observation and LQR-based anti-swing control of slung-load systems.

## Project Structure

```
├── CMakeLists.txt
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
├── scripts/            # Python scripts (LQR gain design, plotting)
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
- Max horizontal acceleration: `±3 m/s²`
- Total duration: `40 s`

### Custom Parameters

```bash
./run_closed_loop_lqr <target_position> <initial_theta_rad> <rope_length> <duration>

# Example: 0m -> 30m, initial angle 5° (0.087 rad), rope 10m, 20s
./run_closed_loop_lqr 30.0 0.087 10.0 20.0
```

### Plot Results

```bash
python3 ../scripts/plot_closed_loop.py closed_loop_results.csv
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

Generates:
- `simulation_results.csv`        (pure pitch)
- `simulation_results_roll.csv`   (pure roll)
- `simulation_results_mixed.csv`  (mixed pitch + roll)

### Plot Results

```bash
python3 ../scripts/plot_results.py simulation_results.csv simulation_results.png
```

The plot window will stay open. **Close it manually when you are done.**

---

## 3. CSV Replay (Legacy)

Replays recorded flight/crane IMU data through the observer and validates against recorded observer outputs.

### Run

```bash
cd build
./run_csv_replay ../crane_imu_obs_debug.csv
```

Generates:
- `replay_validation.csv`

### Plot Results

```bash
python3 ../scripts/plot_replay_results.py replay_validation.csv replay_validation.png
```

The plot window will stay open. **Close it manually when you are done.**

---

## LQR Gain Tuning

Edit `scripts/compute_lqr_gain.py` to adjust LQR weights, then regenerate the gain header:

```bash
cd /home/hcy/work_space/pend_observer_test
python3 scripts/compute_lqr_gain.py --q-theta 30.0 --q-omega 10.0 --r-ax 1.0
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
| `a_max` | `±3 m/s²` | Drone acceleration limit |

## Coordinate Frames

- **FRD**: Forward-Right-Down (body frame, IMU measurements)
- **NEU**: North-East-Up (navigation frame, velocity/position)
- **Euler angles**: `[yaw, pitch, roll]` in radians (Z-Y-X order)
- **Angular rates**: `[roll_rate, pitch_rate, yaw_rate]` in rad/s

> **Note**: The legacy observer interface `pend_observer_iterate_2` expects gyro input in **deg/s** and yaw input in **deg**. The C++ wrapper `PendulumObserver` handles all SI-unit conversions internally.
