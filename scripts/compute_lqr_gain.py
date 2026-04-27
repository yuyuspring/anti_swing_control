#!/usr/bin/env python3
"""
LQR 增益离线计算脚本

功能：为 1-D 俯仰吊重制动控制设计离散 LQR 控制器，
      并自动生成 C++ 头文件 include/controller/lqr_gain.hpp。

控制目标：将无人机水平速度降到 0，同时抑制吊重摆动。
状态向量：x = [vx, theta, theta_dot]^T
控制输入：u = ax（无人机水平加速度）

使用方法：
    python3 scripts/compute_lqr_gain.py
    python3 scripts/compute_lqr_gain.py --rope-length 10.0 --dt 0.01
"""

import numpy as np
from scipy.linalg import solve_discrete_are, expm
import argparse
import os


def build_state_space(g: float, L: float):
    """
    建立连续时间状态空间模型（小角度线性化）

    状态 x = [vx, theta, theta_dot]^T
    输入 u = ax

    系统方程：
        vx_dot      = ax
        theta_dot   = omega
        omega_dot   = -(g/L) * theta - (1/L) * ax

    对应矩阵：
        A = [[0,    0,     0],
             [0,    0,     1],
             [0,  -g/L,    0]]

        B = [[ 1],
             [ 0],
             [-1/L]]

    参数:
        g: 重力加速度 [m/s^2]
        L: 绳长 [m]

    返回:
        A: 3x3 连续状态矩阵
        B: 3x1 连续输入矩阵
    """
    A = np.array([
        [0.0, 0.0, 0.0],      # vx_dot = ax
        [0.0, 0.0, 1.0],      # theta_dot = omega
        [0.0, -g / L, 0.0]    # omega_dot = -(g/L)*theta - (1/L)*ax
    ], dtype=float)

    B = np.array([
        [1.0],      # 加速度直接影响速度
        [0.0],      # 加速度不直接影响角度
        [-1.0 / L]  # 加速度通过惯性力影响角加速度
    ], dtype=float)

    return A, B


def c2d(A, B, dt):
    """
    零阶保持（ZOH）离散化

    将连续系统 (A, B) 离散化为 (Ad, Bd)，采样周期为 dt。

    数学原理：
        Ad = exp(A * dt)
        Bd = integral_0^dt exp(A*tau) dt * B

    实现方法：构造增广矩阵 M = [[A, B], [0, 0]]，
              计算矩阵指数 exp(M*dt)，提取左上/右上分块。

    参数:
        A: 连续状态矩阵 (n x n)
        B: 连续输入矩阵 (n x m)
        dt: 采样周期 [s]

    返回:
        Ad: 离散状态矩阵 (n x n)
        Bd: 离散输入矩阵 (n x m)
    """
    n = A.shape[0]      # 状态维度
    m = B.shape[1]      # 输入维度

    # 构造增广矩阵：M = [[A, B], [0, 0]]
    M = np.block([
        [A, B],
        [np.zeros((m, n)), np.zeros((m, m))]
    ])

    # 计算矩阵指数 exp(M * dt)
    M_exp = expm(M * dt)

    # 提取离散化后的 A_d 和 B_d
    Ad = M_exp[:n, :n]
    Bd = M_exp[:n, n:]
    return Ad, Bd


def solve_lqr(Ad, Bd, Q, R_val):
    """
    求解离散 LQR 的最优反馈增益 K

    性能指标：
        J = sum(x_k^T * Q * x_k + u_k^T * R * u_k)

    求解步骤：
        1. 解离散代数 Riccati 方程 (DARE) 得到 P
        2. 计算最优增益 K = (R + Bd^T P Bd)^{-1} Bd^T P Ad

    参数:
        Ad: 离散状态矩阵
        Bd: 离散输入矩阵
        Q: Q 权重矩阵（3x3 numpy array 或对角线列表 [Q_vx, Q_theta, Q_omega]）
        R_val:  R 矩阵的标量值（单输入系统）

    返回:
        K: 1x3 最优状态反馈增益向量 [K_vx, K_theta, K_omega]
    """
    if isinstance(Q, (list, tuple)):
        Q = np.diag(Q)            # 构造对角权重矩阵 Q
    R = np.array([[R_val]])       # 构造输入权重矩阵 R（标量）

    # 解离散代数 Riccati 方程：P = A^T P A - A^T P B (R+B^T P B)^{-1} B^T P A + Q
    P = solve_discrete_are(Ad, Bd, Q, R)

    # 计算最优反馈增益 K
    K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad).flatten()
    return K


def generate_lqr_gains(
    rope_length: float = 15.0,
    gravity: float = 9.81,
    dt_control: float = 0.02,
    output_path: str = None,
):
    """
    生成三种控制模式的 LQR 增益，并输出为 C++ 头文件

    三种模式的设计哲学：

    1. Full（均衡模式）:
       Q = [2, 30, 10], R = 2
       - 速度收敛与摆动抑制兼顾
       - 适合一般工况

    2. Shortest（最短距离模式）:
       Q = [8, 2, 1], R = 3
       - 速度权重较大，摆角权重较小
       - 优先快速减速，对摆动容忍度较高

    3. MinSwing（最小摆动模式）:
       Q = [1, 100, 50], R = 2
       - 摆角和角速度权重很大
       - 温柔减速，优先抑制摆动

    参数:
        rope_length: 绳长 [m]
        gravity:     重力加速度 [m/s^2]
        dt_control:  控制周期 [s]
        output_path: 输出文件路径（默认：../include/controller/lqr_gain.hpp）
    """
    # Step 1: 建立连续时间模型
    A, B = build_state_space(gravity, rope_length)

    # Step 2: ZOH 离散化
    Ad, Bd = c2d(A, B, dt_control)

    # Step 3: 求解四种模式的 LQR 增益
    # Mode 1: Full - 均衡策略
    K_full = solve_lqr(Ad, Bd, [2.0, 30.0, 10.0], 2.0)

    # Mode 2: Shortest - 优先速度收敛（最短刹车距离）
    K_shortest = solve_lqr(Ad, Bd, [8.0, 2.0, 1.0], 3.0)

    # Mode 3: MinSwing - 优先摆动抑制（温柔刹车）
    K_minswing = solve_lqr(Ad, Bd, [1.0, 100.0, 50.0], 2.0)

    # Mode 4: VelocityOmega - 同时抑制速度和角速度
    K_velomega = solve_lqr(Ad, Bd, [10.0, 1.0, 100.0], 2.0)

    # Mode 5: PayloadVelocity - 惩罚 payload 绝对水平速度 (vx + L*omega)
    # 代价函数中增加 q_pay * (vx + L*omega)^2 项
    q_pay = 4.0
    q_theta = 1.0
    q_omega_extra = 0.1
    Q_payload = np.array([
        [q_pay, 0.0, q_pay * rope_length],
        [0.0, q_theta, 0.0],
        [q_pay * rope_length, 0.0, q_pay * rope_length**2 + q_omega_extra]
    ], dtype=float)
    K_payload = solve_lqr(Ad, Bd, Q_payload, 2.0)

    # Step 4: 打印闭环极点，验证稳定性
    print("=" * 60)
    print(f"LQR 增益设计结果（速度目标制动）")
    print(f"绳长 L = {rope_length:.3f} m, 控制周期 Ts = {dt_control:.3f} s")
    print("=" * 60)

    for name, K in [("Full", K_full), ("Shortest", K_shortest), ("MinSwing", K_minswing),
                    ("VelocityOmega", K_velomega), ("PayloadVelocity", K_payload)]:
        # 闭环矩阵 A_cl = Ad - Bd * K
        Acl = Ad - Bd @ K.reshape(1, -1)
        eigs = np.linalg.eigvals(Acl)
        max_eig = max(abs(eigs))

        print(f"\n{name}:")
        print(f"  增益 K = [{K[0]:.6f}, {K[1]:.6f}, {K[2]:.6f}]")
        print(f"  闭环最大特征值 = {max_eig:.4f}", end="")
        if max_eig < 1.0:
            print("  ✓ 稳定")
        else:
            print("  ✗ 不稳定或临界稳定")

    print("=" * 60)

    # Step 5: 确定输出路径
    if output_path is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_path = os.path.join(script_dir, "..", "include", "controller", "lqr_gain.hpp")
        output_path = os.path.abspath(output_path)

    # 确保目标目录存在
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    # Step 6: 生成 C++ 头文件内容
    header_content = f"""#pragma once

// AUTO-GENERATED by scripts/compute_lqr_gain.py
// DO NOT EDIT MANUALLY
// 此文件由 Python 脚本自动生成，请勿手动修改

namespace pendulum {{

enum class LqrMode {{
    kFull,          ///< 均衡模式：速度收敛 + 摆动抑制
    kShortest,      ///< 最短距离模式：优先速度收敛
    kMinSwing,      ///< 最小摆动模式：优先摆动抑制
    kVelocityOmega,   ///< 速度+角速度模式：同时抑制速度和平抑角速度
    kPayloadVelocity  ///< payload 绝对速度模式：直接惩罚吊重水平绝对速度
}};

struct LqrGain {{
    // Full: Q=[2,30,10], R=2
    static constexpr double kFullV     = {K_full[0]:.8f};
    static constexpr double kFullTheta = {K_full[1]:.8f};
    static constexpr double kFullOmega = {K_full[2]:.8f};

    // Shortest: Q=[8,2,1], R=3
    static constexpr double kShortestV     = {K_shortest[0]:.8f};
    static constexpr double kShortestTheta = {K_shortest[1]:.8f};
    static constexpr double kShortestOmega = {K_shortest[2]:.8f};

    // MinSwing: Q=[1,100,50], R=2
    static constexpr double kMinSwingV     = {K_minswing[0]:.8f};
    static constexpr double kMinSwingTheta = {K_minswing[1]:.8f};
    static constexpr double kMinSwingOmega = {K_minswing[2]:.8f};

    // VelocityOmega: Q=[10,1,100], R=2
    static constexpr double kVelocityOmegaV     = {K_velomega[0]:.8f};
    static constexpr double kVelocityOmegaTheta = {K_velomega[1]:.8f};
    static constexpr double kVelocityOmegaOmega = {K_velomega[2]:.8f};

    // PayloadVelocity: Q penalizes (vx + L*omega)^2, R=2
    static constexpr double kPayloadVelocityV     = {K_payload[0]:.8f};
    static constexpr double kPayloadVelocityTheta = {K_payload[1]:.8f};
    static constexpr double kPayloadVelocityOmega = {K_payload[2]:.8f};
}};

}} // namespace pendulum
"""

    with open(output_path, "w") as f:
        f.write(header_content)

    print(f"\n已生成: {output_path}")


def main():
    """
    命令行入口

    支持参数：
        --rope-length: 绳长 [m]，默认 15.0
        --gravity:     重力加速度 [m/s^2]，默认 9.81
        --dt:          控制周期 [s]，默认 0.02
        --output:      输出文件路径，默认 ../include/controller/lqr_gain.hpp
    """
    parser = argparse.ArgumentParser(description="计算制动控制的 LQR 增益")
    parser.add_argument("--rope-length", type=float, default=15.0, help="绳长 [m]")
    parser.add_argument("--gravity", type=float, default=9.81, help="重力加速度 [m/s^2]")
    parser.add_argument("--dt", type=float, default=0.02, help="控制周期 [s]")
    parser.add_argument("--output", type=str, default=None, help="输出文件路径")
    args = parser.parse_args()

    generate_lqr_gains(
        rope_length=args.rope_length,
        gravity=args.gravity,
        dt_control=args.dt,
        output_path=args.output,
    )


if __name__ == "__main__":
    main()
