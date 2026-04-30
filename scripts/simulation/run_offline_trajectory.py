#!/usr/bin/env python3
"""
离线规划轨迹仿真：Jerk-Ramp 三段式（无恒定段）

支持两种无人机模型：
  --model simple:  直接加速度控制 (ax 为控制量)
  --model pitch:   俯仰角控制 (pitch 为控制量, 拉力沿体轴z, 垂向稳定)

轨迹形状（刹车段）：
  第0段: 控制量线性变化到最大值, 0 <= t < t0 (到 theta_max)
  第1段: 无 (t1=0)
  第2段: 控制量从当前值释放到 0, t0 <= t < t0+t2

用法:
    python3 run_offline_trajectory.py [--model {simple,pitch}] [--jerk J] [--brake-start T]
"""

import numpy as np
import csv
import os
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

# 无人机质量 (用于俯仰角模型)
DRONE_MASS = 20.0  # kg

# 俯仰角限制
PITCH_MAX = np.arctan(AX_MAX / g)  # ≈ 11.5°
PITCH_RATE_MAX = np.radians(30.0)   # 30°/s
PITCH_ACC_MAX = np.radians(100.0)   # 100°/s²

# 仿真参数
DT_TRUTH = 0.005
T_FINAL = 60.0
CRUISE_SPEED = 15.0
INITIAL_THETA = 0.1
INITIAL_THETA_DOT = 0.0


def compute_derivative_simple(state, ax):
    """简单模型: ax 直接作为控制输入"""
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


def compute_derivative_pitch(state):
    """俯仰角模型: 状态包含 pitch 和 pitch_rate"""
    px, vx, theta, theta_dot, pitch, pitch_rate = state
    st = np.sin(theta)
    ct = np.cos(theta)
    sp = np.sin(pitch)
    cp = np.cos(pitch)

    # 拉力 (垂向稳定: T*cos(pitch) = m*g)
    thrust = DRONE_MASS * g / cp if abs(cp) > 1e-6 else 0.0

    # 水平加速度 (拉力水平分量)
    ax = thrust * sp / DRONE_MASS  # = g * tan(pitch)

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
    dpitch = pitch_rate
    dpitch_rate = 0.0  # 由外部伺服更新

    return np.array([dpx, dvx, dtheta, dtheta_dot, dpitch, dpitch_rate])


def rk4_step_simple(state, ax, dt):
    k1 = compute_derivative_simple(state, ax)
    k2 = compute_derivative_simple(state + 0.5 * dt * k1, ax)
    k3 = compute_derivative_simple(state + 0.5 * dt * k2, ax)
    k4 = compute_derivative_simple(state + dt * k3, ax)
    return state + dt * (k1 + 2*k2 + 2*k3 + k4) / 6.0


def rk4_step_pitch(state, dt):
    k1 = compute_derivative_pitch(state)
    k2 = compute_derivative_pitch(state + 0.5 * dt * k1)
    k3 = compute_derivative_pitch(state + 0.5 * dt * k2)
    k4 = compute_derivative_pitch(state + dt * k3)
    return state + dt * (k1 + 2*k2 + 2*k3 + k4) / 6.0


def update_pitch_servo(pitch, pitch_rate, pitch_cmd, dt):
    """俯仰角伺服更新 (带速率、加速度、角度限制)"""
    # 速率指令
    rate_target = (pitch_cmd - pitch) / dt
    if rate_target > PITCH_RATE_MAX:
        rate_target = PITCH_RATE_MAX
    elif rate_target < -PITCH_RATE_MAX:
        rate_target = -PITCH_RATE_MAX

    # 加速度限制
    acc = (rate_target - pitch_rate) / dt
    if acc > PITCH_ACC_MAX:
        new_rate = pitch_rate + PITCH_ACC_MAX * dt
    elif acc < -PITCH_ACC_MAX:
        new_rate = pitch_rate - PITCH_ACC_MAX * dt
    else:
        new_rate = rate_target

    new_pitch = pitch + new_rate * dt
    # 角度限制
    if new_pitch > PITCH_MAX:
        new_pitch = PITCH_MAX
        new_rate = 0.0
    elif new_pitch < -PITCH_MAX:
        new_pitch = -PITCH_MAX
        new_rate = 0.0
    return new_pitch, new_rate


def find_theta_max(jerk_or_pitch_rate, model, t_guess=12.0):
    """用短仿真找到 theta_max 时刻"""
    T = t_guess + 2.0

    if model == "simple":
        def dyn(t, y):
            th, om, vx, px = y
            ax = -jerk_or_pitch_rate * t if t < t_guess else 0.0
            return [om, -(g/L)*np.sin(th) - (ax/L)*np.cos(th), ax, vx]
        y0 = [0.0, 0.0, 15.0, 0.0]
    else:
        def dyn(t, y):
            th, om, vx, px, p, pr = y
            # pitch_cmd 线性变化
            pc = -jerk_or_pitch_rate * t if t < t_guess else 0.0
            if pc > PITCH_MAX:
                pc = PITCH_MAX
            elif pc < -PITCH_MAX:
                pc = -PITCH_MAX
            p, pr = update_pitch_servo(p, pr, pc, 0.01)
            ax = g * np.tan(p)
            return [om, -(g/L)*np.sin(th) - (ax/L)*np.cos(th), ax, vx, pr, 0.0]
        y0 = [0.0, 0.0, 15.0, 0.0, 0.0, 0.0]

    sol = rk4_solve(dyn, [0, T], y0, dt=0.01)

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


def run_simulation(jerk, brake_start, model):
    """主仿真"""
    omega_n = np.sqrt(g / L)

    # ---- Step 1: 找 theta_max 时刻 t0 ----
    t0_guess = 2 * np.pi / omega_n
    t0 = find_theta_max(jerk, model, t_guess=t0_guess + 2.0)
    if t0 is None:
        print("Error: Could not find theta_max")
        return None

    # ---- Step 2: 计算 t2 (vx=0) ----
    # 速度变化 = 控制量曲线下的面积
    if model == "simple":
        # ax(t) = -j*t, 面积 = -0.5*j*t0²
        ax_at_t0 = -jerk * t0
        # 第2段面积 = -0.5 * |ax_at_t0| * t2
        t2 = (CRUISE_SPEED - 0.5 * jerk * t0**2) / (0.5 * abs(ax_at_t0))
    else:
        # pitch 模型: 先做一次短仿真获取 t0 时刻的 pitch
        def short_dyn(t, y):
            th, om, vx, px, p, pr = y
            pc = -jerk * t
            if pc > PITCH_MAX:
                pc = PITCH_MAX
            elif pc < -PITCH_MAX:
                pc = -PITCH_MAX
            p, pr = update_pitch_servo(p, pr, pc, 0.01)
            return [0, 0, 0, 0, 0, 0]  # 只关心 pitch
        # 用数值积分估算
        pitch_t0 = 0.0
        pr = 0.0
        dt = 0.01
        t = 0.0
        while t < t0:
            pc = -jerk * t
            if pc > PITCH_MAX:
                pc = PITCH_MAX
            elif pc < -PITCH_MAX:
                pc = -PITCH_MAX
            pitch_t0, pr = update_pitch_servo(pitch_t0, pr, pc, dt)
            t += dt
        ax_at_t0 = g * np.tan(pitch_t0)
        # 解析近似: 第0段平均 ax ≈ 0.5*ax_at_t0, 面积 ≈ 0.5*ax_at_t0*t0
        # 第2段平均 ax ≈ 0.5*ax_at_t0, 面积 ≈ 0.5*ax_at_t0*t2
        area0 = 0.5 * abs(ax_at_t0) * t0
        if area0 < CRUISE_SPEED:
            t2 = (CRUISE_SPEED - area0) / (0.5 * abs(ax_at_t0))
        else:
            t2 = 0.1

    if t2 < 0.1:
        print(f"Warning: t2={t2:.3f}s is too small")
        t2 = 0.1

    if model == "simple":
        print(f"[Jerk-Ramp Offline Trajectory — Simple Model]")
        print(f"  jerk = {jerk:.3f} m/s³")
        print(f"  Phase 0: 0 ~ {t0:.3f}s, ax@{t0:.3f}s = {ax_at_t0:.3f} m/s²")
    else:
        print(f"[Jerk-Ramp Offline Trajectory — Pitch Model]")
        print(f"  jerk = {jerk:.3f} rad/s³ (pitch cmd rate)")
        print(f"  Phase 0: 0 ~ {t0:.3f}s, pitch@{t0:.3f}s = {np.degrees(pitch_t0):.2f}°")
        print(f"           ax@{t0:.3f}s = {ax_at_t0:.3f} m/s²")
        print(f"           thrust = {DRONE_MASS*g/np.cos(pitch_t0):.1f} N")

    print(f"  Phase 1: None (t1=0)")
    print(f"  Phase 2: {t0:.3f} ~ {t0+t2:.3f}s")
    print(f"  Total brake time: {t0+t2:.3f}s")

    # ---- Step 3: 完整仿真 ----
    if model == "simple":
        state = np.array([0.0, 0.0, INITIAL_THETA, INITIAL_THETA_DOT])
    else:
        state = np.array([0.0, 0.0, INITIAL_THETA, INITIAL_THETA_DOT, 0.0, 0.0])

    accel_phase_end = CRUISE_SPEED / AX_MAX
    ax_cmd = 0.0
    pitch = 0.0
    pitch_rate = 0.0

    log = []
    t = 0.0

    print(f"\n  Accel phase : 0 ~ {accel_phase_end:.1f} s")
    print(f"  Cruise phase: {accel_phase_end:.1f} ~ {brake_start:.1f} s")
    print(f"  Brake phase : {brake_start:.1f} ~ {brake_start + t0 + t2:.1f} s")

    while t <= T_FINAL + 1e-9:
        if t < accel_phase_end:
            ax_target = AX_MAX
            pitch_cmd = np.arctan(AX_MAX / g) if model == "pitch" else 0.0
        elif t < brake_start:
            ax_target = 0.0
            pitch_cmd = 0.0
        else:
            t_rel = t - brake_start
            if t_rel < t0:
                if model == "simple":
                    ax_target = -jerk * t_rel
                else:
                    pitch_cmd = -jerk * t_rel
            elif t_rel < t0 + t2:
                if model == "simple":
                    ax_target = -jerk * t0 * (1.0 - (t_rel - t0) / t2)
                else:
                    pitch_cmd = -jerk * t0 * (1.0 - (t_rel - t0) / t2)
            else:
                ax_target = 0.0
                pitch_cmd = 0.0

        # Pitch limit
        if model == "pitch":
            if pitch_cmd > PITCH_MAX:
                pitch_cmd = PITCH_MAX
            elif pitch_cmd < -PITCH_MAX:
                pitch_cmd = -PITCH_MAX

        if model == "simple":
            # Jerk limit for ax
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
                "pitch_cmd_rad": 0.0,
                "pitch_applied_rad": 0.0,
                "thrust_n": 0.0,
            })

            state = rk4_step_simple(state, ax_cmd, DT_TRUTH)
            state[1] = np.clip(state[1], -VX_MAX, VX_MAX)
        else:
            # Pitch servo
            pitch, pitch_rate = update_pitch_servo(pitch, pitch_rate, pitch_cmd, DT_TRUTH)
            ax_applied = g * np.tan(pitch)
            thrust = DRONE_MASS * g / np.cos(pitch) if abs(np.cos(pitch)) > 1e-6 else 0.0

            log.append({
                "time_s": t,
                "px_truth_m": state[0],
                "vx_truth_m_s": state[1],
                "theta_truth_rad": state[2],
                "theta_dot_truth_rad_s": state[3],
                "theta_est_rad": state[2],
                "omega_est_rad_s": state[3],
                "ax_cmd_m_s2": g * np.tan(pitch_cmd) if abs(pitch_cmd) < PITCH_MAX else 0.0,
                "ax_applied_m_s2": ax_applied,
                "pitch_cmd_rad": pitch_cmd,
                "pitch_applied_rad": pitch,
                "thrust_n": thrust,
            })

            state = rk4_step_pitch(state, DT_TRUTH)
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
    return out_path


def main():
    parser = argparse.ArgumentParser(description="Offline jerk-ramp trajectory simulation")
    parser.add_argument("--model", choices=["simple", "pitch"], default="simple",
                        help="Drone model: simple (ax control) or pitch (pitch-angle control)")
    parser.add_argument("--jerk", type=float, default=0.3,
                        help="Jerk magnitude (m/s³ for simple, rad/s³ for pitch), default=0.3")
    parser.add_argument("--brake-start", type=float, default=40.0,
                        help="Brake start time (s), default=40.0")
    args = parser.parse_args()

    run_simulation(args.jerk, args.brake_start, args.model)


if __name__ == "__main__":
    main()
