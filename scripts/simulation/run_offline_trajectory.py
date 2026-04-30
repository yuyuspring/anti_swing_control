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
JERK_MAX = 10.0

# 无人机质量 (用于俯仰角模型)
DRONE_MASS = 20.0  # kg

# 俯仰角限制
PITCH_MAX = np.pi / 2 - 0.01  # ≈ 89.4°，几乎不限制 (伺服硬限幅)
PRE_TILT_MAX = np.radians(12.0)  # 预调阶段最大抬头角 12°
SYNC_GAIN = 0.5  # 同步阶段比例系数 pitch = 0.5 * theta
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


def compute_derivative_pitch(state, pitch):
    """俯仰角模型: pitch 作为外部参数传入"""
    px, vx, theta, theta_dot = state[:4]
    st = np.sin(theta)
    ct = np.cos(theta)
    cp = np.cos(pitch)

    # 水平加速度 (来自推力倾斜, 垂向稳定: T*cos(pitch) = m*g)
    # 抬头(pitch>0) → 推力向后 → 减速(ax<0)
    # 低头(pitch<0) → 推力向前 → 加速(ax>0)
    ax = -g * np.tan(pitch) if abs(cp) > 1e-6 else 0.0

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


def rk4_step_simple(state, ax, dt):
    k1 = compute_derivative_simple(state, ax)
    k2 = compute_derivative_simple(state + 0.5 * dt * k1, ax)
    k3 = compute_derivative_simple(state + 0.5 * dt * k2, ax)
    k4 = compute_derivative_simple(state + dt * k3, ax)
    return state + dt * (k1 + 2*k2 + 2*k3 + k4) / 6.0


def rk4_step_pitch(state, pitch, dt):
    k1 = compute_derivative_pitch(state, pitch)
    k2 = compute_derivative_pitch(state + 0.5 * dt * k1, pitch)
    k3 = compute_derivative_pitch(state + 0.5 * dt * k2, pitch)
    k4 = compute_derivative_pitch(state + dt * k3, pitch)
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


def find_critical_point(jerk_or_pitch_rate, model, mode="theta_max", initial_state=None, t_guess=12.0):
    """用短仿真找到关键时刻
    mode="theta_max": theta 达到最大 (theta_dot=0)
    mode="omega_max": theta_dot 达到最大 (theta_ddot=0)
    mode="zero_cross": theta 穿过 0 点（从初始符号变号）
    """
    T = t_guess + 4.0  # 给足够时间找到 zero_cross

    if initial_state is None:
        if model == "simple":
            initial_state = [0.0, 0.0, 15.0, 0.0]
        else:
            initial_state = [0.0, 0.0, 15.0, 0.0, 0.0, 0.0]

    th0 = initial_state[0]

    if model == "simple":
        def dyn(t, y):
            th, om, vx, px = y
            ax = -jerk_or_pitch_rate * t if t < t_guess else 0.0
            return [om, -(g/L)*np.sin(th) - (ax/L)*np.cos(th), ax, vx]
        y0 = initial_state[:4]
    else:
        def dyn(t, y):
            th, om, vx, px, p, pr = y
            pc = -jerk_or_pitch_rate * t if t < t_guess else 0.0
            if pc > PITCH_MAX:
                pc = PITCH_MAX
            elif pc < -PITCH_MAX:
                pc = -PITCH_MAX
            p, pr = update_pitch_servo(p, pr, pc, 0.01)
            ax = -g * np.tan(p)
            return [om, -(g/L)*np.sin(th) - (ax/L)*np.cos(th), ax, vx, pr, 0.0]
        y0 = initial_state[:6]

    sol = rk4_solve(dyn, [0, T], y0, dt=0.005)
    t_arr = sol['t']
    theta = sol['y'][0]
    omega = sol['y'][1]

    if mode == "theta_max":
        # theta_dot 从正变负
        for i in range(1, len(t_arr)-1):
            if omega[i-1] > 0 and omega[i] <= 0:
                return t_arr[i]
    elif mode == "omega_max":
        # theta_ddot 从正变负（theta_dot 达到最大）
        ax_arr = np.where(t_arr < t_guess, -jerk_or_pitch_rate * t_arr, 0.0)
        theta_ddot = -(g/L)*np.sin(theta) - (ax_arr/L)*np.cos(theta)
        for i in range(1, len(t_arr)-1):
            if theta_ddot[i-1] > 0 and theta_ddot[i] <= 0:
                return t_arr[i]
    elif mode == "zero_cross":
        # theta 穿过 0 点
        for i in range(1, len(t_arr)-1):
            if th0 >= 0:
                # 从正到负
                if theta[i-1] > 0 and theta[i] <= 0:
                    return t_arr[i]
            else:
                # 从负到正
                if theta[i-1] < 0 and theta[i] >= 0:
                    return t_arr[i]
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


def run_simulation(jerk, brake_start, model, t0_mode="theta_max"):
    """主仿真"""
    omega_n = np.sqrt(g / L)

    t1 = 0.0  # 初始化 t1
    ax_peak = 0.0  # 初始化峰值加速度

    # ---- 预仿真到刹车开始，获取初始状态 (用于 zero_cross 模式) ----
    pre_state = np.array([0.0, 0.0, INITIAL_THETA, INITIAL_THETA_DOT])
    accel_phase_end = CRUISE_SPEED / AX_MAX
    t_pre = 0.0
    ax_cmd_pre = 0.0
    while t_pre < brake_start - 1e-9:
        if t_pre < accel_phase_end:
            ax_target_pre = AX_MAX
        else:
            ax_target_pre = 0.0
        # Jerk limit
        j_pre = (ax_target_pre - ax_cmd_pre) / DT_TRUTH
        if j_pre > JERK_MAX:
            ax_cmd_pre += JERK_MAX * DT_TRUTH
        elif j_pre < -JERK_MAX:
            ax_cmd_pre -= JERK_MAX * DT_TRUTH
        else:
            ax_cmd_pre = ax_target_pre
        pre_state = rk4_step_simple(pre_state, ax_cmd_pre, DT_TRUTH)
        pre_state[1] = np.clip(pre_state[1], -VX_MAX, VX_MAX)
        t_pre += DT_TRUTH

    brake_initial_state = [pre_state[2], pre_state[3], pre_state[1], pre_state[0]]

    if model == "simple":
        if t0_mode == "thrust_hold":
            # ---- 新策略: t0 ramp 到 T_max, t1 保持到 omega_max ----
            t0_guess = 2 * np.pi / omega_n
            omega_max_t = find_critical_point(jerk, model, mode="omega_max", t_guess=t0_guess + 2.0)
            if omega_max_t is None:
                print("Error: Could not find omega_max")
                return None
            T_max = AX_MAX / jerk
            if T_max <= omega_max_t:
                t0 = T_max
                t1 = omega_max_t - T_max
                ax_peak = -AX_MAX
                t2 = (CRUISE_SPEED - 0.5 * AX_MAX * t0 - AX_MAX * t1) / (0.5 * AX_MAX)
            else:
                t0 = omega_max_t
                t1 = 0.0
                ax_peak = -jerk * t0
                t2 = (CRUISE_SPEED - 0.5 * jerk * t0**2) / (0.5 * abs(ax_peak))
            if t2 < 0.1:
                t2 = 0.1
            print(f"[Thrust-Hold Offline Trajectory — Simple Model]")
            print(f"  jerk = {jerk:.3f} m/s³")
            print(f"  T_max = {T_max:.3f}s, omega_max = {omega_max_t:.3f}s")
            print(f"  Phase 0 (ramp): 0 ~ {t0:.3f}s, ax@{t0:.3f}s = {ax_peak:.3f} m/s²")
            if t1 > 0:
                print(f"  Phase 1 (hold): {t0:.3f} ~ {t0+t1:.3f}s, ax = {ax_peak:.3f} m/s²")
            else:
                print(f"  Phase 1: None (t1=0)")
            print(f"  Phase 2 (release): {t0+t1:.3f} ~ {t0+t1+t2:.3f}s")
            print(f"  Total brake time: {t0+t1+t2:.3f}s")
        elif t0_mode == "zero_cross":
            # ---- 新策略: t0 到 theta 过 0 点, t1 保持到 omega_max ----
            # a_max 不限制; 用带 jerk 限制的预仿真找到关键时间点
            # 从刹车初始状态继续仿真刹车段
            sim_state = np.array([brake_initial_state[3], brake_initial_state[2],
                                   brake_initial_state[0], brake_initial_state[1]])
            ax_cmd_sim = 0.0
            zero_t = None
            omega_max_t = None
            theta_prev = sim_state[2]
            t_sim = 0.0
            dt_sim = DT_TRUTH
            MAX_SIM_TIME = 10.0
            while t_sim < MAX_SIM_TIME:
                # Phase 0: ramp; Phase 1: hold at ax_peak
                if zero_t is None:
                    ax_target_sim = -jerk * t_sim
                else:
                    ax_target_sim = -jerk * zero_t
                j_sim = (ax_target_sim - ax_cmd_sim) / dt_sim
                if j_sim > JERK_MAX:
                    ax_cmd_sim += JERK_MAX * dt_sim
                elif j_sim < -JERK_MAX:
                    ax_cmd_sim -= JERK_MAX * dt_sim
                else:
                    ax_cmd_sim = ax_target_sim

                # RK4 step
                k1 = compute_derivative_simple(sim_state, ax_cmd_sim)
                k2 = compute_derivative_simple(sim_state + 0.5 * dt_sim * k1, ax_cmd_sim)
                k3 = compute_derivative_simple(sim_state + 0.5 * dt_sim * k2, ax_cmd_sim)
                k4 = compute_derivative_simple(sim_state + dt_sim * k3, ax_cmd_sim)
                sim_state_new = sim_state + dt_sim * (k1 + 2*k2 + 2*k3 + k4) / 6.0

                # 检测 theta 过 0 点
                if zero_t is None:
                    if theta_prev < 0 and sim_state_new[2] >= 0:
                        zero_t = t_sim
                    elif theta_prev > 0 and sim_state_new[2] <= 0:
                        zero_t = t_sim

                # 检测 omega 最大 (dtheta_dot 从正变负，使用完整动力学含阻尼)
                if zero_t is not None and omega_max_t is None:
                    ddot_curr = compute_derivative_simple(sim_state, ax_cmd_sim)[3]
                    ddot_next = compute_derivative_simple(sim_state_new, ax_cmd_sim)[3]
                    if ddot_curr > 0 and ddot_next <= 0:
                        omega_max_t = t_sim

                sim_state = sim_state_new
                theta_prev = sim_state[2]
                t_sim += dt_sim

                if zero_t is not None and omega_max_t is not None:
                    break

            if zero_t is None:
                print("Error: Could not find theta zero cross in pre-sim")
                return None
            if omega_max_t is None:
                # 如果 omega_max 在仿真时间内未找到，使用最大仿真时间
                omega_max_t = zero_t
            t0 = zero_t
            t1 = max(0.0, omega_max_t - t0)
            ax_peak = -jerk * t0
            # 速度平衡: 0.5*j*t0^2 + |ax_peak|*t1 + 0.5*|ax_peak|*t2 = CRUISE_SPEED
            t2 = 2.0 * CRUISE_SPEED / (jerk * t0) - t0 - 2.0 * t1
            if t2 < 0.1:
                t2 = 0.1
            print(f"[Zero-Cross Offline Trajectory — Simple Model]")
            print(f"  jerk = {jerk:.3f} m/s³")
            print(f"  Brake initial: theta={np.degrees(brake_initial_state[0]):.2f}°, omega={np.degrees(brake_initial_state[1]):.2f}°/s")
            print(f"  theta_zero = {t0:.3f}s, omega_max = {omega_max_t:.3f}s")
            print(f"  Phase 0 (ramp): 0 ~ {t0:.3f}s, ax@{t0:.3f}s = {ax_peak:.3f} m/s²")
            if t1 > 0:
                print(f"  Phase 1 (hold): {t0:.3f} ~ {t0+t1:.3f}s, ax = {ax_peak:.3f} m/s²")
            else:
                print(f"  Phase 1: None (t1=0)")
            print(f"  Phase 2 (release): {t0+t1:.3f} ~ {t0+t1+t2:.3f}s")
            print(f"  Total brake time: {t0+t1+t2:.3f}s")
        else:
            # ---- 原策略: theta_max 或 omega_max ----
            t0_guess = 2 * np.pi / omega_n
            t0 = find_critical_point(jerk, model, mode=t0_mode, t_guess=t0_guess + 2.0)
            if t0 is None:
                print(f"Error: Could not find {t0_mode}")
                return None
            ax_peak = -jerk * t0
            t2 = (CRUISE_SPEED - 0.5 * jerk * t0**2) / (0.5 * abs(ax_peak))
            if t2 < 0.1:
                t2 = 0.1
            print(f"[Jerk-Ramp Offline Trajectory — Simple Model]")
            print(f"  jerk = {jerk:.3f} m/s³")
            print(f"  t0 mode: {t0_mode}")
            print(f"  Phase 0: 0 ~ {t0:.3f}s, ax@{t0:.3f}s = {ax_peak:.3f} m/s²")
            print(f"  Phase 1: None (t1=0)")
            print(f"  Phase 2: {t0:.3f} ~ {t0+t2:.3f}s")
            print(f"  Total brake time: {t0+t2:.3f}s")
    else:
        # pitch 闭环模型: 不需要 t0/t2
        t0 = 0.0
        t2 = 0.0
        print(f"[Pitch-Loop Offline Trajectory — Pitch follows Theta]")
        print(f"  Pitch cmd = theta (current pendulum angle)")
        print(f"  No PITCH_MAX limit")

    # ---- Step 3: 完整仿真 ----
    state = np.array([0.0, 0.0, INITIAL_THETA, INITIAL_THETA_DOT])

    ax_cmd = 0.0
    pitch = 0.0
    pitch_rate = 0.0

    # 记录历史最大摆角 (用于 pitch 模型)
    max_theta_hist = abs(INITIAL_THETA)
    # 刹车阶段状态: 0=预调(到-max_theta_hist), 1=同步(pitch=theta)
    brake_phase = 0
    theta_dot_prev = INITIAL_THETA_DOT

    log = []
    t = 0.0

    print(f"\n  Accel phase : 0 ~ {accel_phase_end:.1f} s")
    print(f"  Cruise phase: {accel_phase_end:.1f} ~ {brake_start:.1f} s")
    if model == "simple":
        print(f"  Brake phase : {brake_start:.1f} ~ {brake_start + t0 + t1 + t2:.1f} s")
    else:
        print(f"  Brake phase : {brake_start:.1f} ~ {T_FINAL:.1f} s (two-phase: pre-tilt → sync)")

    while t <= T_FINAL + 1e-9:
        # 更新历史最大摆角 (仅在刹车开始前更新)
        if t < brake_start:
            max_theta_hist = max(max_theta_hist, abs(state[2]))

        if t < accel_phase_end:
            ax_target = AX_MAX
            pitch_cmd = np.arctan(-AX_MAX / g) if model == "pitch" else 0.0
        elif t < brake_start:
            ax_target = 0.0
            pitch_cmd = 0.0
        else:
            t_rel = t - brake_start
            if model == "simple":
                total_phase0 = t0 + t1
                if t_rel < t0:
                    ax_target = -jerk * t_rel
                elif t_rel < total_phase0:
                    ax_target = ax_peak  # 保持 Phase 0 结束时的加速度
                elif t_rel < total_phase0 + t2:
                    ax_target = ax_peak * (1.0 - (t_rel - total_phase0) / t2)
                else:
                    ax_target = 0.0
            else:
                # pitch 两阶段闭环
                if brake_phase == 0:
                    # 预调阶段: pitch 按 jerk 速率 ramp 到 12° (抬头减速)
                    t_rel = t - brake_start
                    pitch_cmd = min(jerk * t_rel, PRE_TILT_MAX)
                    # 检测摆角达到最大 (theta_dot 从正变负)
                    if theta_dot_prev > 0 and state[3] <= 0:
                        brake_phase = 1
                        print(f"    [Sync triggered at t={t:.3f}s, theta={np.degrees(state[2]):.2f}°]")
                else:
                    # 同步阶段: pitch = 0.5 * theta (比例控制，避免正反馈发散)
                    pitch_cmd = SYNC_GAIN * state[2]

        theta_dot_prev = state[3]

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
            ax_applied = -g * np.tan(pitch)
            thrust = DRONE_MASS * g / np.cos(pitch) if abs(np.cos(pitch)) > 1e-6 else 0.0

            log.append({
                "time_s": t,
                "px_truth_m": state[0],
                "vx_truth_m_s": state[1],
                "theta_truth_rad": state[2],
                "theta_dot_truth_rad_s": state[3],
                "theta_est_rad": state[2],
                "omega_est_rad_s": state[3],
                "ax_cmd_m_s2": -g * np.tan(pitch_cmd) if abs(pitch_cmd) < PITCH_MAX else 0.0,
                "ax_applied_m_s2": ax_applied,
                "pitch_cmd_rad": pitch_cmd,
                "pitch_applied_rad": pitch,
                "thrust_n": thrust,
            })

            state = rk4_step_pitch(state, pitch, DT_TRUTH)
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
    parser.add_argument("--t0-mode", choices=["theta_max", "omega_max", "thrust_hold", "zero_cross"], default="theta_max",
                        help="t0 calculation mode: theta_max (default), omega_max, thrust_hold, or zero_cross")
    args = parser.parse_args()

    run_simulation(args.jerk, args.brake_start, args.model, args.t0_mode)


if __name__ == "__main__":
    main()
