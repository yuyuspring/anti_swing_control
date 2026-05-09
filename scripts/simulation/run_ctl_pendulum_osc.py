#!/usr/bin/env python3
"""
按照 src/dynamics/ctl_pendulum_osc.m 的传递函数框图，
构建可运行时域仿真模型。

输入：速度指令序列 r(t) [m/s]
输出：v(t), a1(t), a2(t), theta(t), theta_dot(t)

运行示例:
    # 默认：零输入自由响应（复现 .m 中的 lsim(T, 0, t, x0)）
    python3 run_ctl_pendulum_osc.py

    # 从 CSV 读取速度指令
    python3 run_ctl_pendulum_osc.py --csv-input speed_profile.csv
"""

import argparse
import csv
import os

import numpy as np
import control
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


# ============================================================================
# 物理与模型参数
# ============================================================================
G = 9.81
L = 10.0

# filt2 的离散传递函数系数（来自 .m 代码）
_FILT2_NUM_D = [0.000241359, 0.000482718, 0.000241359]
_FILT2_DEN_D = [1.0, -1.95557824, 0.9565436765]
_FILT2_TS = 0.0025

# 通过 Tustin 逆变换得到的 filt2 连续等效
# 验证：sample_system(continuous, _FILT2_TS, method='tustin') 与原离散误差 < 0.001 dB
_FILT2_NUM_C = [157.93961494178433]
_FILT2_DEN_C = [1.0, 17.771531324439593, 157.93969673884737]


def build_continuous_systems():
    """
    按照 .m 代码逐句翻译，在连续域中构建各 SISO 闭环传递函数。
    """
    # MATLAB: s = tf('s')
    s = control.tf([1, 0], [1])

    # --- 旋转回路 ---
    # Gr = 2/s / (1/(2*pi*2)*s+1) = 8*pi / (s^2 + 4*pi*s)
    Gr = control.tf([8 * np.pi], [1, 4 * np.pi, 0])

    # Kr = pid(0.1,0.1,0) * (s/(4π)+1)/(s/(40π)+1) * 100
    #    = 100*(s+1)*(s+4π) / (s*(s+40π))
    num_kr = [100, 100 + 400 * np.pi, 400 * np.pi]
    den_kr = [1, 40 * np.pi, 0]
    Kr = control.tf(num_kr, den_kr)
    Gr_cl = control.feedback(Gr * Kr, 1)

    # --- 高度回路 ---
    # Ge = Gr_cl / s
    Ge = Gr_cl / s
    Ke = 6.0

    # Ge_cl = feedback(Ge*Ke, 1) * filt2
    # 用 Tustin 逆变换得到的连续等效替代原离散 filt2
    filt2 = control.tf(_FILT2_NUM_C, _FILT2_DEN_C)
    Ge_cl = control.feedback(Ge * Ke, 1) * filt2

    # --- 速度回路 ---
    # Kv = pid(.5,.02,0) * (1 + 0.6*s)
    # pid(.5,.02,0) = (0.5s+0.02)/s
    # Kv = (0.5s+0.02)(0.6s+1)/s = (0.3s^2 + 0.512s + 0.02)/s
    # num_kv = [0.3, 0.512, 0.02]
    num_kv = [0.3, 5.12, 0.02]
    den_kv = [1, 0]
    Kv = control.tf(num_kv, den_kv)

    # --- 单摆 ---
    # pend_ss: A=[0 1; -g/l 0], B=[0; 1/l], C=eye(2), D=0
    # pend_tf = -tf(pend_ss)
    # pend_eul = -1/(l*s^2 + g)
    # pend_w   = -s/(l*s^2 + g)
    pend_eul = control.tf([-1.0], [L, 0, G])
    pend_w = control.tf([-1.0, 0], [L, 0, G])

    # --- 完整互联（代数推导） ---
    # P = Kv * Ge_cl   (.m 中 gain 标量抵消)
    P = Kv * Ge_cl

    # 摆动耦合传递函数：
    #   pend_eul 把基座加速度映射为摆角 theta。
    #   物理上，单摆感受到的是机体的实际加速度 a，不是名义加速度 a1。
    #   稳态时 theta = (-1/g) * a，反作用力 a2 = -(m/M) * a。
    #   这里用 PEND_GAIN 表示 m/M，pend_sys 的直流增益为 -PEND_GAIN。
    m = 180 # 摆质量，单位 kg
    M = 120 # 机体质量，单位 kg
    PEND_GAIN = m/M
    pend_sys = control.tf([-PEND_GAIN * G], [L, 0, G])

    Inte = control.tf([1], [1, 0])

    # 闭环推导（单摆输入为实际加速度 a）：
    #
    # %% plant (修正版：单摆输入为实际加速度 a)
    #
    #                                                   
    #                                                 +------------------------------|
    #               a       eul     eul         a1    |                              |
    #  r--->o--->Kv--->gain----->P------>1/gain------>o--->1/s----->y                |
    #       |                                         | a           |                |
    #       ------------------------------------------+--------------                | 
    #                                                 |                              |a2
    #                                                 +--->pend---->w,eul----gain1---+
    #
    #   gain1: w^2*l*eul*m/M
    #   代数关系：
    #   e = r - v
    #   a1 = P * e
    #   a2 = pend_sys * a          # 单摆由实际加速度 a 驱动
    #   a  = a1 + a2 = a1 + pend_sys * a
    #   => a = a1 / (1 - pend_sys)  # 从 a1 到 a 的传递
    #   v  = Inte * a = Inte * a1 / (1 - pend_sys)
    #
    # 从 a1 到 v 的开环：Inte / (1 - pend_sys)
    # => a1 = feedback(P, Inte / (1 - pend_sys)) * r

    T_a_a1 = 1 / (1 - pend_sys)          # 从 a1 到 a
    T_a1 = control.feedback(P, Inte * T_a_a1)
    T_a = T_a_a1 * T_a1
    T_v = Inte * T_a
    T_a2 = pend_sys * T_a
    T_theta = pend_eul * T_a             # 单摆输入是实际加速度 a
    T_omega = pend_w * T_a

    return T_v, T_a1, T_a2, T_theta, T_omega


def run_simulation(systems, t, r, dt=None):
    """
    运行时域仿真。

    Args:
        systems: (T_v, T_a1, T_a2, T_theta, T_omega) 连续 SISO 传递函数
        t: 时间向量 [s]
        r: 输入向量 (速度指令) [m/s]
        dt: 离散化步长 [s]，默认取 t[1]-t[0]

    Returns:
        dict: 包含 time, r, v, a1, a2, theta, theta_dot
    """
    T_v, T_a1, T_a2, T_theta, T_omega = systems

    # 直接对连续系统做 forced_response，避免离散化引入数值不稳定
    resp_v = control.forced_response(T_v, T=t, U=r)
    resp_a1 = control.forced_response(T_a1, T=t, U=r)
    resp_a2 = control.forced_response(T_a2, T=t, U=r)
    resp_theta = control.forced_response(T_theta, T=t, U=r)
    resp_omega = control.forced_response(T_omega, T=t, U=r)

    return {
        'time': resp_v.time,
        'r': r,
        'v': resp_v.outputs,
        'a1': resp_a1.outputs,
        'a2': resp_a2.outputs,
        'theta': resp_theta.outputs,
        'theta_dot': resp_omega.outputs,
    }


def load_csv_reference(csv_path):
    """加载 LQR CSV 参考数据，用于对比绘制。"""
    t_ref, theta_ref, omega_ref = [], [], []
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        hdr = next(reader)
        # 找到对应列索引
        try:
            i_theta = hdr.index('theta_truth_rad')
            i_omega = hdr.index('theta_dot_truth_rad_s')
        except ValueError:
            i_theta = 3
            i_omega = 4
        for row in reader:
            t_ref.append(float(row[0]))
            theta_ref.append(float(row[i_theta]))
            omega_ref.append(float(row[i_omega]))
    return np.array(t_ref), np.array(theta_ref), np.array(omega_ref)


def plot_results(data, save_path=None, ref_data=None, v_ref_data=None):
    """绘制线性模型结果，可选叠加 LQR 参考数据和 LQR 指令速度。"""
    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)

    # Compute cumulative distance by trapezoidal integration
    t = np.array(data['time'])
    v = np.array(data['v'])
    a = np.array(data['a1']) + np.array(data['a2'])
    distance = np.zeros_like(t)
    for i in range(1, len(t)):
        distance[i] = distance[i-1] + 0.5 * (v[i-1] + v[i]) * (t[i] - t[i-1])

    # Detect accel / brake phases by acceleration threshold
    accel_mask = a > 0.25
    brake_mask = a < -0.25

    # Merge adjacent segments with gaps shorter than 0.5s
    def merge_mask(mask, t_arr, max_gap=0.5):
        out = mask.copy()
        segments = []
        i = 0
        while i < len(out):
            if out[i]:
                start = i
                while i < len(out) and out[i]:
                    i += 1
                segments.append((start, i - 1))
            else:
                i += 1
        for j in range(1, len(segments)):
            prev_end = segments[j - 1][1]
            curr_start = segments[j][0]
            if t_arr[curr_start] - t_arr[prev_end] < max_gap:
                out[prev_end:curr_start + 1] = True
        return out

    accel_mask = merge_mask(accel_mask, t, 0.5)
    brake_mask = merge_mask(brake_mask, t, 0.5)

    # Compute accel / brake distances
    accel_dist, brake_dist = 0.0, 0.0
    for i in range(1, len(t)):
        dt = t[i] - t[i-1]
        if accel_mask[i]:
            accel_dist += 0.5 * (v[i-1] + v[i]) * dt
        if brake_mask[i]:
            brake_dist += 0.5 * (v[i-1] + v[i]) * dt

    # Plot velocity: LQR command v_ref (A), LQR actual r (B), model output v
    if v_ref_data is not None:
        t_vref, vref = v_ref_data
        axes[0].plot(t_vref, vref, 'gray', linestyle='-', linewidth=1.0,
                     alpha=0.7, label='v_ref (LQR command A)')
    axes[0].plot(data['time'], data['r'], 'r--', label='r = LQR actual (B)', linewidth=1.5)
    axes[0].plot(data['time'], data['v'], 'b', label='v = TF model output', linewidth=1.5)
    # Shade accel / brake regions
    axes[0].fill_between(t, 0, v, where=accel_mask, alpha=0.15, color='green',
                         label=f'Accel dist={accel_dist:.1f}m')
    axes[0].fill_between(t, 0, v, where=brake_mask, alpha=0.15, color='red',
                         label=f'Brake dist={brake_dist:.1f}m')
    axes[0].set_ylabel('Velocity [m/s]')
    axes[0].legend(loc='upper left', fontsize=8)
    axes[0].grid(True)
    axes[0].set_title(f'r-v  (Accel={accel_dist:.1f}m, Brake={brake_dist:.1f}m, Total={distance[-1]:.1f}m)')

    # Acceleration breakdown: a1 + a2 = a
    a = np.array(data['a1']) + np.array(data['a2'])
    axes[1].plot(data['time'], data['a1'], 'b--', label='a1 (nominal)', linewidth=1.5)
    axes[1].plot(data['time'], data['a2'], 'r:', label='a2 (coupling)', linewidth=1.5)
    axes[1].plot(data['time'], a, 'g-', label='a = a1 + a2', linewidth=1.5)
    axes[1].axhline(0, color='gray', linestyle='-', alpha=0.3)
    axes[1].set_ylabel('Acceleration [m/s²]')
    axes[1].legend()
    axes[1].grid(True)
    axes[1].set_title('Acceleration Breakdown (a = a1 + a2)')

    # 摆角对比
    axes[2].plot(data['time'], np.degrees(data['theta']), 'b',
                 label='TF model theta', linewidth=1.5)
    if ref_data is not None:
        t_ref, theta_ref, _ = ref_data
        axes[2].plot(t_ref, np.degrees(theta_ref), 'g--',
                     label='LQR truth theta', linewidth=1.5)
    axes[2].set_ylabel('Theta [deg]')
    axes[2].legend()
    axes[2].grid(True)
    axes[2].set_title('theta comparison')

    # 角速度对比
    axes[3].plot(data['time'], np.degrees(data['theta_dot']), 'b',
                 label='TF model theta_dot', linewidth=1.5)
    if ref_data is not None:
        t_ref, _, omega_ref = ref_data
        axes[3].plot(t_ref, np.degrees(omega_ref), 'g--',
                     label='LQR truth theta_dot', linewidth=1.5)
    axes[3].set_ylabel('Theta_dot [deg/s]')
    axes[3].set_xlabel('Time [s]')
    axes[3].legend()
    axes[3].grid(True)
    axes[3].set_title('theta_dot comparison')

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"[Plot] Saved to {save_path}")
    else:
        plt.show()


def save_csv(data, filepath):
    """保存结果到 CSV。"""
    dirname = os.path.dirname(filepath)
    if dirname:
        os.makedirs(dirname, exist_ok=True)

    with open(filepath, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time_s', 'r_m_s', 'v_m_s', 'a1_m_s2', 'a2_m_s2',
            'theta_rad', 'theta_dot_rad_s'
        ])
        for i in range(len(data['time'])):
            writer.writerow([
                data['time'][i],
                data['r'][i],
                data['v'][i],
                data['a1'][i],
                data['a2'][i],
                data['theta'][i],
                data['theta_dot'][i],
            ])
    print(f"[CSV] Saved to {filepath}")


def main():
    parser = argparse.ArgumentParser(
        description='时域仿真: ctl_pendulum_osc 闭环模型（r 为速度指令）'
    )
    parser.add_argument(
        '--csv-input', type=str, default=None,
        help='输入 CSV: 两列 time[s], r[m/s]'
    )
    parser.add_argument(
        '--dt', type=float, default=0.01,
        help='仿真步长 [s] (默认 0.01)'
    )
    parser.add_argument(
        '--t-final', type=float, default=10.0,
        help='仿真时长 [s] (默认 10，仅当无 --csv-input 时生效)'
    )
    parser.add_argument(
        '--out-csv', type=str, default=None,
        help='输出 CSV 路径'
    )
    parser.add_argument(
        '--out-png', type=str, default=None,
        help='输出 PNG 路径'
    )
    parser.add_argument(
        '--s-curve', action='store_true',
        help='使用 S 曲线速度输入: jerk=10, a_max=2, v_cruise=15'
    )
    args = parser.parse_args()

    print("[Build] Constructing continuous closed-loop systems...")
    systems = build_continuous_systems()

    v_ref_data = None
    if args.csv_input:
        t_raw = []
        r_raw = []
        vref_raw = []
        with open(args.csv_input, 'r') as f:
            reader = csv.reader(f)
            hdr = next(reader)  # skip header
            # closed_loop_systemenergy.csv: time_s, px_truth_m, vx_truth_m_s, ...
            # r 是速度指令，取 vx_truth_m_s 列（索引 2）
            r_col = 2 if 'vx_truth_m_s' in hdr else 1
            vref_col = hdr.index('v_ref_m_s') if 'v_ref_m_s' in hdr else -1
            if r_col == 1:
                print("[Warn] CSV 未识别到 vx_truth_m_s，回退到第 2 列")
            for row in reader:
                t_raw.append(float(row[0]))
                r_raw.append(float(row[r_col]))
                if vref_col >= 0:
                    vref_raw.append(float(row[vref_col]))
        t_raw = np.array(t_raw)
        r_raw = np.array(r_raw)

        # 插值到均匀步长 dt
        t = np.arange(t_raw[0], t_raw[-1] + args.dt * 0.5, args.dt)
        r = np.interp(t, t_raw, r_raw)
        if vref_col >= 0:
            vref_raw = np.array(vref_raw)
            vref_interp = np.interp(t, t_raw, vref_raw)
            v_ref_data = (t, vref_interp)
        print(f"[Input] Loaded {len(t_raw)} samples from {args.csv_input}, "
              f"interpolated to {len(t)} points @ dt={args.dt}s")
    else:
        t = np.arange(0, args.t_final + args.dt * 0.5, args.dt)
        # .m 代码中 u = sin(2*pi*0*t) = 0
        r = np.zeros_like(t)
        print(f"[Input] Zero input, t=0~{args.t_final}s, dt={args.dt}s")

    print("[Simulate] Running time-domain simulation...")
    data = run_simulation(systems, t, r, dt=args.dt)

    # 加载 LQR 参考数据用于对比绘制
    ref_data = None
    if args.csv_input:
        ref_data = load_csv_reference(args.csv_input)

    # 默认输出路径
    results_dir = os.path.join(
        os.path.dirname(__file__), '..', '..', 'results', 'ctl_pendulum_osc'
    )
    os.makedirs(results_dir, exist_ok=True)

    if args.out_csv is None:
        args.out_csv = os.path.join(results_dir, 'ctl_pendulum_osc.csv')
    if args.out_png is None:
        args.out_png = os.path.join(results_dir, 'ctl_pendulum_osc.png')

    save_csv(data, args.out_csv)
    plot_results(data, save_path=args.out_png, ref_data=ref_data, v_ref_data=v_ref_data)


if __name__ == '__main__':
    main()
