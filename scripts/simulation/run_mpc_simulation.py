#!/usr/bin/env python3
"""
MPC 闭环仿真脚本（多模式）

支持两种目标函数：
    --mode min_omega        最小摆速度
    --mode min_system_energy 最小系统能量

约束：
    |ax| <= 2 m/s²
    |ax(k) - ax(k-1)| <= jerk_max * dt
    vx >= 0（不倒退）
    theta 尽量 >= 0（软约束，避免回摆到负）

输出: results/mpc/closed_loop_mpc_<mode>.csv
"""

import numpy as np
from scipy.linalg import solve_discrete_are, expm
import cvxpy as cp
import csv
import os
import sys
import time

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
DT_CONTROL = 0.10
T_FINAL = 60.0
BRAKE_START = 40.0
CRUISE_SPEED = 15.0
INITIAL_THETA = 0.1
INITIAL_THETA_DOT = 0.0

# MPC 参数
N = 40             # 预测时域（4 s = 40 × 0.1 s）
du_max = JERK_MAX * DT_CONTROL  # = 0.20


def get_mpc_config(mode):
    if mode == "min_omega":
        Q = np.diag([1.0, 10.0, 1.0])
        R = 2.0
        label = "minomega"
    elif mode == "min_system_energy":
        #    q_drone   无人机动能惩罚     让无人机速度尽快降到 0
        #    q_pe      吊重势能惩罚       让摆角回到竖直（theta=0）
        #    q_ke      吊重绝对动能惩罚   让吊重相对于地面别乱动（vx + L*omega → 0）
        #    q_omega   角速度正则项       保证 Q 矩阵正定，额外抑制摆速
        q_ke = 0.5
        q_pe = 1000.0
        q_drone = 1.0
        q_omega = 1.0
        Q = np.array([
            [q_drone + q_ke, 0.0, q_ke * L],
            [0.0, q_pe, 0.0],
            [q_ke * L, 0.0, q_ke * L**2 + q_omega]
        ], dtype=float)
        R = 2.0
        label = "minsysenergy"
    else:
        raise ValueError(f"Unknown mode: {mode}")
    return Q, R, label


# ---------------------- 离散化模型 ----------------------
Ac = np.array([
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
    [0.0, -g/L, 0.0]
], dtype=float)
Bc = np.array([[1.0], [0.0], [-1.0/L]], dtype=float)

M_aug = np.block([[Ac, Bc], [np.zeros((1, 3)), np.zeros((1, 1))]])
M_exp = expm(M_aug * DT_CONTROL)
Ad = M_exp[:3, :3]
Bd_col = M_exp[:3, 3:]
Bd = Bd_col.flatten()


# ---------------------- 非线性动力学 (RK4) ----------------------
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


# ---------------------- MPC 求解器 ----------------------
def build_mpc_solver(Q, R):
    P_dare = solve_discrete_are(Ad, Bd_col, Q, np.array([[R]]))
    x0_param = cp.Parameter(3)
    u_prev_param = cp.Parameter()
    U = cp.Variable(N)
    X = cp.Variable((N + 1, 3))

    constraints = [X[0] == x0_param]

    for k in range(N):
        constraints += [X[k + 1] == Ad @ X[k] + Bd * U[k]]
        constraints += [U[k] >= -AX_MAX, U[k] <= AX_MAX]

    # Jerk limit
    constraints += [U[0] - u_prev_param >= -du_max, U[0] - u_prev_param <= du_max]
    for k in range(1, N):
        constraints += [U[k] - U[k - 1] >= -du_max, U[k] - U[k - 1] <= du_max]

    # 代价函数
    cost = cp.quad_form(X[N], P_dare)
    for k in range(N):
        cost += cp.quad_form(X[k], Q) + R * U[k] ** 2

    # 软约束：theta < 0 时大惩罚，避免回摆到负
    theta_penalty = 500.0
    for k in range(1, N + 1):
        cost += theta_penalty * cp.pos(-X[k, 1])

    # 软约束：vx < 0 时惩罚（替代硬约束 vx >= 0）
    vx_penalty = 300.0
    for k in range(1, N + 1):
        cost += vx_penalty * cp.pos(-X[k, 0])

    prob = cp.Problem(cp.Minimize(cost), constraints)
    return prob, x0_param, u_prev_param, U, X


# ---------------------- 主仿真 ----------------------
def run_simulation(mode):
    Q, R, label = get_mpc_config(mode)
    prob, x0_param, u_prev_param, U_var, X_var = build_mpc_solver(Q, R)

    state = np.array([0.0, 0.0, INITIAL_THETA, INITIAL_THETA_DOT])
    ax_cmd = 0.0

    control_steps_per_call = int(round(DT_CONTROL / DT_TRUTH))
    step_counter = 0
    t = 0.0

    accel_phase_end = CRUISE_SPEED / AX_MAX
    log = []
    mpc_solve_times = []
    has_been_positive = False

    print(f"[MPC Simulation] Mode: {mode}")
    print(f"  Accel phase : 0 ~ {accel_phase_end:.1f} s")
    print(f"  Cruise phase: {accel_phase_end:.1f} ~ {BRAKE_START:.1f} s")
    print(f"  Brake phase : {BRAKE_START:.1f} ~ {T_FINAL:.1f} s (MPC)")
    print(f"  N={N}, dt_ctrl={DT_CONTROL}")
    print(f"  Soft theta>=0 penalty: 500, vx>=0 penalty: 300")

    while t <= T_FINAL + 1e-9:
        if t < accel_phase_end:
            ax_target = AX_MAX
        elif t < BRAKE_START:
            ax_target = 0.0
        else:
            if step_counter % control_steps_per_call == 0:
                x0_mpc = np.array([state[1], state[2], state[3]])
                x0_param.value = x0_mpc
                u_prev_param.value = ax_cmd
                t0 = time.time()
                prob.solve(solver=cp.OSQP, warm_start=True, max_iter=10000)
                elapsed = time.time() - t0
                mpc_solve_times.append(elapsed)

                if prob.status in ("optimal", "optimal_inaccurate"):
                    ax_target = float(U_var.value[0])
                elif prob.status == "user_limit" and U_var.value is not None:
                    # OSQP 达到迭代上限但返回了可用近似解
                    ax_target = float(U_var.value[0])
                else:
                    print(f"  ⚠ MPC solve failed at t={t:.3f}: {prob.status}")
                    ax_target = ax_cmd

                if U_var.value is not None:
                    u_ws = np.concatenate([U_var.value[1:], [U_var.value[-1]]])
                    U_var.value = u_ws
            else:
                ax_target = ax_cmd

        # 跟踪 theta 是否曾经为正
        if state[2] > 0.0:
            has_been_positive = True

        log.append({
            "time_s": t,
            "px_truth_m": state[0],
            "vx_truth_m_s": state[1],
            "theta_truth_rad": state[2],
            "theta_dot_truth_rad_s": state[3],
            "theta_est_rad": state[2],
            "omega_est_rad_s": state[3],
            "ax_cmd_m_s2": ax_target,
            "ax_applied_m_s2": ax_target,
        })

        state = rk4_step(state, ax_target, DT_TRUTH)
        state[1] = np.clip(state[1], -VX_MAX, VX_MAX)
        ax_cmd = ax_target
        t += DT_TRUTH
        step_counter += 1

    results_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "results", "mpc")
    os.makedirs(results_dir, exist_ok=True)
    out_path = os.path.join(results_dir, f"closed_loop_mpc_{label}.csv")

    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=log[0].keys())
        writer.writeheader()
        writer.writerows(log)

    avg_solve = np.mean(mpc_solve_times) * 1000 if mpc_solve_times else 0
    print(f"\nDone. Saved: {out_path}")
    print(f"MPC solve time: avg={avg_solve:.2f} ms")
    print(f"MPC calls: {len(mpc_solve_times)}, theta ever positive: {has_been_positive}\n")
    return out_path


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "min_omega"
    if mode not in ("min_omega", "min_system_energy"):
        print(f"Usage: python3 run_mpc_simulation.py [min_omega|min_system_energy]")
        sys.exit(1)
    run_simulation(mode)


if __name__ == "__main__":
    main()
