# CSV Replay Reference

This document records the current field, unit, coordinate-frame, and conversion assumptions used by `replay_csv.cpp` when replaying `crane_imu_obs_debug.csv`.

It is intended to be the single reference for later code changes. If a field meaning or unit assumption changes, update this file together with the replay code.

## 1. Replay Input Columns

The replay path reads five groups of columns as actual observer inputs:

| CSV column | Unit in CSV | Frame / meaning | Handling in replay |
| --- | --- | --- | --- |
| `time_stamp` | `s` | sample timestamp | converted to relative time `time_s` for output only |
| `w_meas[0]` | `rad/s` | body angular rate x | multiplied by `R2D`, then passed to observer interface |
| `w_meas[1]` | `rad/s` | body angular rate y | multiplied by `R2D`, then passed to observer interface |
| `w_meas[2]` | `rad/s` | body angular rate z | multiplied by `R2D`, then passed to observer interface |
| `a_meas[0]` | `m/s^2` | FRD specific force / apparent acceleration x | passed to observer directly |
| `a_meas[1]` | `m/s^2` | FRD specific force / apparent acceleration y | passed to observer directly |
| `a_meas[2]` | `m/s^2` | FRD specific force / apparent acceleration z | passed to observer directly |
| `v_meas[0]` | `m/s` | navigation velocity north | passed to observer directly |
| `v_meas[1]` | `m/s` | navigation velocity east | passed to observer directly |
| `v_meas[2]` | `m/s` | navigation velocity up | passed to observer directly |
| `yaw` | `rad` | heading / yaw angle | used directly for forward-axis synthesis; multiplied by `R2D` before passing to observer |

## 2. Recorded Columns Used Only For Replay Validation

These columns are not replay inputs. They are read only to compare recorded values against replayed observer outputs.

| CSV column | Unit in CSV | Meaning | Comparison target |
| --- | --- | --- | --- |
| `theta[1]` | `rad` | recorded pitch estimate | `obs.theta[1]` |
| `theta[2]` | `rad` | recorded roll estimate | `obs.theta[2]` |
| `w_est[0]` | `rad/s` | recorded roll-rate estimate | `obs.w_est[0]` after converting both to `deg/s` for plotting and error output |
| `w_est[1]` | `rad/s` | recorded pitch-rate estimate | `obs.w_est[1]` after converting both to `deg/s` for plotting and error output |
| `v_hat[0]` | `m/s` | recorded velocity estimate north | combined with `v_hat[1]` and `yaw` into recorded forward velocity |
| `v_hat[1]` | `m/s` | recorded velocity estimate east | combined with `v_hat[0]` and `yaw` into recorded forward velocity |
| `a_hat[0]` | `m/s^2` | recorded acceleration estimate north | combined with `a_hat[1]` and `yaw` into recorded forward acceleration |
| `a_hat[1]` | `m/s^2` | recorded acceleration estimate east | combined with `a_hat[0]` and `yaw` into recorded forward acceleration |
| `acc_modify[0]` | `m/s^2` | recorded internal correction x | compared with replay `debug_data[1]` |
| `acc_modify[1]` | `m/s^2` | recorded internal correction y | compared with replay `debug_data[2]` |
| `acc_modify[2]` | `m/s^2` | recorded internal correction z | compared with replay `debug_data[3]` |
| `Rg[0]` | `m/s^2` | recorded gravity projection x | compared with replay `obs.Rg[0]` |
| `Rg[1]` | `m/s^2` | recorded gravity projection y | compared with replay `obs.Rg[1]` |
| `Rg[2]` | `m/s^2` | recorded gravity projection z | compared with replay `obs.Rg[2]` |

## 3. Current Coordinate Assumptions

The replay code currently assumes the following frames and meanings:

- `a_meas[*]` is in FRD, with axes `x = forward`, `y = right`, `z = down`.
- `a_meas[*]` is interpreted as specific force / apparent acceleration, not pure inertial linear acceleration.
- `v_meas[*]`, `v_hat[*]`, and `a_hat[*]` are in navigation frame NEU, with axes `north`, `east`, `up`.
- `yaw` is a navigation heading angle in radians.
- `theta[1]` is pitch and `theta[2]` is roll in radians.

## 4. Interface Unit Expectations In `pend_observer_iterate_2`

The replay code converts some CSV fields because the observer interface does not use the same units as the CSV:

- `w_meas[*]` in CSV is treated as `rad/s`.
- `pend_observer_iterate_2` expects `w_meas[*]` in `deg/s` because the function internally divides by `R2D`.
- `yaw` in CSV is treated as `rad`.
- `pend_observer_iterate_2` expects `ahrs_input[0]` in `deg` because the function internally divides by `R2D`.

This means the replay code intentionally does:

```cpp
w_meas[i] *= R2D;
ahrs_input[0] = yaw_rad * R2D;
```

## 5. Forward-Axis Synthesis Rule

Forward velocity and forward acceleration are synthesized from navigation-frame north/east components using the current yaw convention:

```cpp
forward = cos(yaw_rad) * north + sin(yaw_rad) * east;
```

This matches the yaw rotation convention currently used inside `pend_observer.cpp` for the first row of `R_yaw`.

## 6. Scope Note

This document describes the current replay implementation and its present assumptions. It does not prove that the recorded CSV is physically correct. It only records the assumptions needed to make replay behavior reproducible and reviewable.