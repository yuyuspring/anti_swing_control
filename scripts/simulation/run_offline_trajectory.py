#!/usr/bin/env python3
"""
离线规划轨迹仿真：Jerk-Ramp 三段式（无恒定段）

轨迹形状：
  第0段: ax = -j*t,           0 <= t < t0   (jerk ramp 到 theta_max)
  第1段: 无 (t1=0)
  第2段: ax 从 -j*t0 释放到 0, t0 <= t < t0+t2

t0 由 theta_max 事件决定（数值搜索）
t2 由 vx=0 约束解析计算

用法:
    python3 run_offline_trajectory.py [--jerk J] [--brake-start T]

参数:
    --jerk      第0段 jerk 大小 (m/s³), 默认 0.5
    --brake-start  刹车开始时间 (s), 默认 40.0
"""

import numpy as np
import csv
import os
import sys
import argparse

# ---------------------- 物理参数 ----------------------
g = 9.81
L = 15.0
PAYLOAD_MASS = 150.0
DRAG_COEFF = 1.0
DRAG_AREA = 0.5
AIR_DENSITY = 1.225
LINEAR_DAMPING = 0.15
VX_MAX = 15.0
AX_MAX = 2.0
JERK_MAX = 2.0

# 仿真参数
DT_TRUTH = 0.005
T_FINAL = 60.0
CRUISE_SPEED = 15.0
INITIAL_THETA = 0.1
INITIAL_THETA_DOT = 0.0


def compute_derivative(state, ax):
    px, vx, theta, theta_dot = state
    st = np.sin(theta)
    ct = np.cos(theta)

    dpx = vx
    dvx = ax
    dtheta = theta_dot

    gravity_torque = g * st
    inertial_torque = ax * ct
    linear_drag = LINEAR_DAMPING * theta_dot

    vx_pay = vx + L * theta_dot * ct
    vz_pay = L * theta_dot * st
    v_abs_sq = vx_pay**2 + vz_pay**2
    v_abs = np.sqrt(v_abs_sq)

    quad_drag = 0.0
    if v_abs > 1e-6:
        drag_force = 0.5 * AIR_DENSITY * DRAG_COEFF * DRAG_AREA * v_abs_sq
        f_drag_x = -drag_force * vx_pay / v_abs
        f_drag_z = -drag_force * vz_pay / v_abs
        f_tangential = f_drag_x * ct + f_drag_z * st
        quad_drag = f_tangential / (PAYLOAD_MASS * L)

    dtheta_dot = -(gravity_torque + inertial_torque) / L - linear_drag + quad_drag
    return np.array([dpx, dvx, dtheta, dtheta_dot])


def rk4_step(state, ax, dt):
    k1 = compute_derivative(state, ax)
    k2 = compute_derivative(state + 0.5 * dt * k1, ax)
    k3 = compute_derivative(state + 0.5 * dt * k2, ax)
    k4 = compute_derivative(state + dt * k3, ax)
    return state + dt * (k1 + 2*k2 + 2*k3 + k4) / 6.0


def find_theta_max(j, t_guess=12.0):
    """用短仿真找到 theta_max 时刻"""
    T = t_guess + 2.0

    def dyn(t, y):
        th, om, vx, px = y
        ax = -j * t if t < t_guess else 0.0
        return [om, -(g/L)*np.sin(th) - (ax/L)*np.cos(th), ax, vx]

    sol = rk4_solve(dyn, [0, T], [0.0, 0.0, 15.0, 0.0], dt=0.01)

    for i in range(1, len(sol['t'])-1):
        if sol['y'][1][i-1] > 0 and sol['y'][1][i] <= 0:
            return sol['t'][i]
    return None


def rk4_solve(dyn_func, t_span, y0, dt=0.005):
    """简单的 RK4 积分器，返回 dict"""
    t0, tf = t_span
    state = np.array(y0, dtype=float)
    t = t0
    ts = [t]
    ys = [state.copy()]
    while t < tf - 1e-9:
        step = min(dt, tf - t)
        k1 = np.array(dyn_func(t, state))
        k2 = np.array(dyn_func(t + 0.5*step, state + 0.5*step*k1))
        k3 = np.array(dyn_func(t + 0.5*step, state + 0.5*step*k2))
        k4 = np.array(dyn_func(t + step, state + step*k3))
        state = state + step * (k1 + 2*k2 + 2*k3 + k4) / 6.0
        t += step
        ts.append(t)
        ys.append(state.copy())
    return {'t': np.array(ts), 'y': np.array(ys).T}


def run_simulation(jerk, brake_start):
    """主仿真"""
    omega_n = np.sqrt(g / L)

    # ---- Step 1: 找 theta_max 时刻 t0 ----
    # 先用解析估算: t0 ≈ 2π/ω_n = T
    t0_guess = 2 * np.pi / omega_n
    t0 = find_theta_max(jerk, t_guess=t0_guess + 2.0)
    if t0 is None:
        print("Error: Could not find theta_max")
        return None

    ax_at_t0 = -jerk * t0

    # ---- Step 2: 计算 t2 (vx=0) ----
    # Δv = -0.5*j*t0² - 0.5*j*t0*t2 = -15
    t2 = (CRUISE_SPEED - 0.5 * jerk * t0**2) / (0.5 * jerk * t0)
    if t2 < 0.1:
        print(f"Warning: t2={t2:.3f}s is too small, vx may not reach 0")
        t2 = 0.1

    print(f"[Jerk-Ramp Offline Trajectory]")
    print(f"  jerk = {jerk:.3f} m/s³")
    print(f"  Phase 0 (jerk ramp): 0 ~ {t0:.3f}s, ax@{t0:.3f}s = {ax_at_t0:.3f} m/s²")
    print(f"  Phase 1: None (t1=0)")
    print(f"  Phase 2 (release):   {t0:.3f} ~ {t0+t2:.3f}s")
    print(f"  Total brake time:    {t0+t2:.3f}s")

    # ---- Step 3: 完整仿真 ----
    state = np.array([0.0, 0.0, INITIAL_THETA, INITIAL_THETA_DOT])
    accel_phase_end = CRUISE_SPEED / AX_MAX
    ax_cmd = 0.0

    log = []
    t = 0.0

    print(f"\n  Accel phase : 0 ~ {accel_phase_end:.1f} s")
    print(f"  Cruise phase: {accel_phase_end:.1f} ~ {brake_start:.1f} s")
    print(f"  Brake phase : {brake_start:.1f} ~ {brake_start + t0 + t2:.1f} s")

    while t <= T_FINAL + 1e-9:
        if t < accel_phase_end:
            ax_target = AX_MAX
        elif t < brake_start:
            ax_target = 0.0
        else:
            t_rel = t - brake_start
            if t_rel < t0:
                ax_target = -jerk * t_rel
            elif t_rel < t0 + t2:
                ax_target = -jerk * t0 * (1.0 - (t_rel - t0) / t2)
            else:
                ax_target = 0.0

        # Apply jerk limit (与 LQR 一致)
        j = (ax_target - ax_cmd) / DT_TRUTH
        if j > JERK_MAX:
            ax_cmd += JERK_MAX * DT_TRUTH
        elif j < -JERK_MAX:
            ax_cmd -= JERK_MAX * DT_TRUTH
        else:
            ax_cmd = ax_target

        log.append({
            "time_s": t,
            "px_truth_m": state[0],
            "vx_truth_m_s": state[1],
            "theta_truth_rad": state[2],
            "theta_dot_truth_rad_s": state[3],
            "theta_est_rad": state[2],
            "omega_est_rad_s": state[3],
            "ax_cmd_m_s2": ax_target,
            "ax_applied_m_s2": ax_cmd,
        })

        state = rk4_step(state, ax_cmd, DT_TRUTH)
        state[1] = np.clip(state[1], -VX_MAX, VX_MAX)
        t += DT_TRUTH

    results_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..", "results", "trajectory"
    )
    os.makedirs(results_dir, exist_ok=True)
    out_path = os.path.join(results_dir, "offline_three_phase.csv")

    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=log[0].keys())
        writer.writeheader()
        writer.writerows(log)

    print(f"\nDone. Saved: {out_path}")
    return out_path, jerk, t0, t2


def main():
    parser = argparse.ArgumentParser(description="Offline jerk-ramp trajectory simulation")
    parser.add_argument("--jerk", type=float, default=0.3, help="Jerk magnitude (m/s³), default=0.3")
    parser.add_argument("--brake-start", type=float, default=40.0, help="Brake start time (s), default=40.0")
    args = parser.parse_args()

    run_simulation(args.jerk, args.brake_start)


if __name__ == "__main__":
    main()
