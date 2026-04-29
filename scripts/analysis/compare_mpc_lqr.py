#!/usr/bin/env python3
"""
MPC vs LQR 对比绘图脚本

对比 MPC（最小摆速度）与 LQR VelocityOmega 模式在刹车阶段的表现。
也支持叠加其他 LQR 模式作为参考。

用法:
    python3 scripts/analysis/compare_mpc_lqr.py
"""

import sys
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt

L = 15.0  # rope length

# 颜色定义
COLOR_MPC = "#d62728"   # 红色 — MPC
COLOR_LQR_VO = "#1f77b4"  # 蓝色 — LQR VelocityOmega
COLOR_LQR_FULL = "#2ca02c"  # 绿色
COLOR_LQR_MS = "#9467bd"    # 紫色


def load_and_shift_brake_phase(csv_path, accel_end, cruise_end):
    """读取 CSV，截取刹车阶段并归零时间"""
    df = pd.read_csv(csv_path)
    brake = df[(df["time_s"] >= cruise_end)].copy()
    brake["t_rel"] = brake["time_s"] - cruise_end
    brake["px_rel"] = brake["px_truth_m"] - brake["px_truth_m"].iloc[0]
    return brake


def compute_brake_metrics(df, accel_dist, cruise_dist):
    """计算刹车段指标"""
    brake_df = df[df["time_s"] > 40.0]
    done = brake_df[brake_df["vx_truth_m_s"] < 0.1].index
    brake_dist_val = (
        df.loc[done[0], "px_truth_m"] if len(done) > 0 else df["px_truth_m"].iloc[-1]
    )
    brake_dist_val -= accel_dist + cruise_dist

    settling_idx = brake_df[abs(brake_df["vx_truth_m_s"]) < 0.1].index
    settling_time = (
        df.loc[settling_idx[0], "time_s"] - 40.0 if len(settling_idx) > 0 else np.nan
    )

    max_theta = abs(brake_df["theta_truth_rad"]).max() * 180 / np.pi
    max_ax = abs(brake_df["ax_applied_m_s2"]).max()
    return brake_dist_val, settling_time, max_theta, max_ax


def main():
    args = sys.argv[1:]
    show_plot = "--show" in args
    if show_plot:
        args.remove("--show")
        matplotlib.use("TkAgg")
    else:
        matplotlib.use("Agg")

    results_mpc_dir = "results/mpc"
    results_lqr_dir = "results/lqr"

    if len(args) >= 1:
        mpc_csv = args[0]
        lqr_vo_csv = args[1] if len(args) > 1 else f"{results_lqr_dir}/closed_loop_velomega.csv"
    else:
        mpc_csv = f"{results_mpc_dir}/closed_loop_mpc_minomega.csv"
        lqr_vo_csv = f"{results_lqr_dir}/closed_loop_velomega.csv"

    # 加载数据
    df_mpc = pd.read_csv(mpc_csv)
    df_vo = pd.read_csv(lqr_vo_csv)

    # 也加载 Full 和 MinSwing 作为参考（可选）
    has_ref = False
    try:
        df_full = pd.read_csv(f"{results_lqr_dir}/closed_loop_full.csv")
        df_ms = pd.read_csv(f"{results_lqr_dir}/closed_loop_minswing.csv")
        has_ref = True
    except (FileNotFoundError, pd.errors.EmptyDataError):
        pass

    accel_end = 15.0 / 2.0          # 7.5 s
    cruise_dist = 15.0 * (40.0 - accel_end)  # 487.5 m
    accel_dist = 0.5 * 2.0 * accel_end ** 2  # 56.25 m

    # 截取刹车阶段
    b_mpc = load_and_shift_brake_phase(mpc_csv, accel_end, 40.0)
    b_vo = load_and_shift_brake_phase(lqr_vo_csv, accel_end, 40.0)
    if has_ref:
        b_full = load_and_shift_brake_phase(f"{results_lqr_dir}/closed_loop_full.csv", accel_end, 40.0)
        b_ms = load_and_shift_brake_phase(f"{results_lqr_dir}/closed_loop_minswing.csv", accel_end, 40.0)

    # 计算指标
    def metrics(df):
        return compute_brake_metrics(df, accel_dist, cruise_dist)

    m_mpc = metrics(df_mpc)
    m_vo = metrics(df_vo)
    if has_ref:
        m_full = metrics(df_full)
        m_ms = metrics(df_ms)

    # ---------------- 绘图 ----------------
    fig, axes = plt.subplots(5, 1, figsize=(12, 16), sharex=True)

    # 1. 相对位置
    ax = axes[0]
    ax.plot(b_mpc["t_rel"], b_mpc["px_rel"], color=COLOR_MPC, lw=2, label="MPC (min |omega|)")
    ax.plot(b_vo["t_rel"], b_vo["px_rel"], color=COLOR_LQR_VO, lw=2, label="LQR VelocityOmega")
    if has_ref:
        ax.plot(b_full["t_rel"], b_full["px_rel"], color=COLOR_LQR_FULL, lw=1.5, ls="--", alpha=0.6, label="LQR Full")
        ax.plot(b_ms["t_rel"], b_ms["px_rel"], color=COLOR_LQR_MS, lw=1.5, ls="--", alpha=0.6, label="LQR MinSwing")
    ax.set_ylabel("Brake Distance [m]")
    ax.set_title("MPC vs LQR — Brake Phase Comparison (axLimit = ±2 m/s²)")
    ax.legend(loc="right")
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="k", lw=0.5)

    # 2. 速度
    ax = axes[1]
    ax.plot(b_mpc["t_rel"], b_mpc["vx_truth_m_s"], color=COLOR_MPC, lw=2, label="MPC")
    ax.plot(b_vo["t_rel"], b_vo["vx_truth_m_s"], color=COLOR_LQR_VO, lw=2, label="LQR VelocityOmega")
    if has_ref:
        ax.plot(b_full["t_rel"], b_full["vx_truth_m_s"], color=COLOR_LQR_FULL, lw=1.5, ls="--", alpha=0.6)
        ax.plot(b_ms["t_rel"], b_ms["vx_truth_m_s"], color=COLOR_LQR_MS, lw=1.5, ls="--", alpha=0.6)
    ax.set_ylabel("Velocity [m/s]")
    ax.legend(loc="right")
    ax.grid(True, alpha=0.3)

    # 3. Payload 绝对水平速度
    ax = axes[2]
    for b, c, lab in [(b_mpc, COLOR_MPC, "MPC"), (b_vo, COLOR_LQR_VO, "LQR VelocityOmega")]:
        vx = b["vx_truth_m_s"].values
        th = b["theta_truth_rad"].values
        w = b["theta_dot_truth_rad_s"].values
        v_pay = vx + L * w * np.cos(th)
        ax.plot(b["t_rel"], v_pay, color=c, lw=2, label=lab)
    if has_ref:
        for b, c in [(b_full, COLOR_LQR_FULL), (b_ms, COLOR_LQR_MS)]:
            vx = b["vx_truth_m_s"].values
            th = b["theta_truth_rad"].values
            w = b["theta_dot_truth_rad_s"].values
            v_pay = vx + L * w * np.cos(th)
            ax.plot(b["t_rel"], v_pay, color=c, lw=1.5, ls="--", alpha=0.6)
    ax.set_ylabel("Payload |vx| [m/s]")
    ax.legend(loc="right")
    ax.grid(True, alpha=0.3)

    # 4. 摆角
    ax = axes[3]
    ax.plot(b_mpc["t_rel"], np.degrees(b_mpc["theta_truth_rad"]), color=COLOR_MPC, lw=2, label="MPC")
    ax.plot(b_vo["t_rel"], np.degrees(b_vo["theta_truth_rad"]), color=COLOR_LQR_VO, lw=2, label="LQR VelocityOmega")
    if has_ref:
        ax.plot(b_full["t_rel"], np.degrees(b_full["theta_truth_rad"]), color=COLOR_LQR_FULL, lw=1.5, ls="--", alpha=0.6)
        ax.plot(b_ms["t_rel"], np.degrees(b_ms["theta_truth_rad"]), color=COLOR_LQR_MS, lw=1.5, ls="--", alpha=0.6)
    ax.set_ylabel("Pitch Angle [deg]")
    ax.legend(loc="right")
    ax.grid(True, alpha=0.3)

    # 5. 控制输入
    ax = axes[4]
    ax.plot(b_mpc["t_rel"], b_mpc["ax_applied_m_s2"], color=COLOR_MPC, lw=2, label="MPC")
    ax.plot(b_vo["t_rel"], b_vo["ax_applied_m_s2"], color=COLOR_LQR_VO, lw=2, label="LQR VelocityOmega")
    if has_ref:
        ax.plot(b_full["t_rel"], b_full["ax_applied_m_s2"], color=COLOR_LQR_FULL, lw=1.5, ls="--", alpha=0.6)
        ax.plot(b_ms["t_rel"], b_ms["ax_applied_m_s2"], color=COLOR_LQR_MS, lw=1.5, ls="--", alpha=0.6)
    ax.axhline(-2, color="gray", ls="--", alpha=0.3)
    ax.axhline(2, color="gray", ls="--", alpha=0.3)
    ax.set_ylabel("Acceleration [m/s²]")
    ax.set_xlabel("Time since brake start [s]")
    ax.legend(loc="right")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_name = f"{results_mpc_dir}/mpc_vs_lqr.png"
    fig.savefig(out_name, dpi=150)
    print(f"Saved: {out_name}")

    if show_plot:
        print("Showing plot window (close it manually when done)...")
        plt.show()

    # ---------------- 指标表格 ----------------
    print("\n" + "=" * 70)
    print("Brake Phase Metrics")
    print("=" * 70)
    print(f"{'Metric':<25} {'MPC':>12} {'LQR VelOmega':>15} {'Delta':>12}")
    print("-" * 70)
    labels = ["Brake distance [m]", "Settling time [s]", "Max |theta| [deg]", "Max |ax| [m/s²]"]
    for i, lab in enumerate(labels):
        d = m_vo[i] - m_mpc[i]
        print(f"{lab:<25} {m_mpc[i]:>12.2f} {m_vo[i]:>15.2f} {d:>+12.2f}")
    print("=" * 70)


if __name__ == "__main__":
    main()
