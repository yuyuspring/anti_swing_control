#!/usr/bin/env python3
"""绘制 zero_cross 模式下不同 jerk 值的对比图。"""

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

sys.path.insert(0, "scripts/simulation")
from run_offline_trajectory import run_simulation

JERK_VALUES = [0.5, 0.8, 1.0, 1.5, 2.0]
COLORS = ["C0", "C1", "C2", "C3", "C4"]


def generate_csv(jerk):
    path = f"results/trajectory/offline_zero_cross_j{jerk}.csv"
    if not os.path.exists(path):
        run_simulation(jerk, 40.0, "simple", "zero_cross")
        # 重命名输出文件
        src = "results/trajectory/offline_three_phase.csv"
        os.rename(src, path)
    return pd.read_csv(path)


def plot_full_comparison():
    fig, axes = plt.subplots(5, 1, figsize=(12, 16), sharex=True)
    fig.suptitle("Zero-Cross Offline Trajectory — Simple Model Comparison\n(jerk = 0.5, 0.8, 1.0, 1.5, 2.0)",
                 fontsize=14, fontweight="bold")

    stats = []
    for jerk, color in zip(JERK_VALUES, COLORS):
        df = generate_csv(jerk)
        t = df["time_s"]

        axes[0].plot(t, df["px_truth_m"], color=color, label=f"j={jerk}", linewidth=1.5)
        axes[1].plot(t, df["vx_truth_m_s"], color=color, linewidth=1.5)
        axes[2].plot(t, df["theta_truth_rad"] * 180 / np.pi, color=color, linewidth=1.5)
        axes[3].plot(t, df["theta_dot_truth_rad_s"] * 180 / np.pi, color=color, linewidth=1.5)
        axes[4].plot(t, df["ax_applied_m_s2"], color=color, linewidth=1.5)

        # 统计
        braking = df[df["ax_cmd_m_s2"] < -0.1]
        if len(braking) > 0:
            t_brake = braking["time_s"].iloc[0]
            sub = df[df["time_s"] >= t_brake]
            stats.append({
                "jerk": jerk,
                "brake_dist": sub["px_truth_m"].iloc[-1] - sub["px_truth_m"].iloc[0],
                "max_theta": sub["theta_truth_rad"].abs().max() * 180 / np.pi,
                "final_vx": sub["vx_truth_m_s"].iloc[-1],
            })

    axes[0].set_ylabel("Position [m]")
    axes[0].set_title("Horizontal Position")
    axes[0].legend(loc="upper left")
    axes[0].grid(True, alpha=0.3)

    axes[1].set_ylabel("Velocity [m/s]")
    axes[1].set_title("Horizontal Velocity")
    axes[1].axhline(0, color="black", linestyle="-", alpha=0.2)
    axes[1].grid(True, alpha=0.3)

    axes[2].set_ylabel("Angle [deg]")
    axes[2].set_title("Pendulum Pitch Angle")
    axes[2].grid(True, alpha=0.3)

    axes[3].set_ylabel("Rate [deg/s]")
    axes[3].set_title("Pendulum Pitch Rate")
    axes[3].axhline(0, color="black", linestyle="-", alpha=0.2)
    axes[3].grid(True, alpha=0.3)

    axes[4].set_ylabel("Acceleration [m/s²]")
    axes[4].set_title("Applied Acceleration")
    axes[4].axhline(0, color="black", linestyle="-", alpha=0.2)
    axes[4].grid(True, alpha=0.3)
    axes[4].set_xlabel("Time [s]")

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    out = "results/trajectory/zero_cross_comparison_full.png"
    plt.savefig(out, dpi=150)
    print(f"Full comparison saved to: {out}")
    plt.close()

    return stats


def plot_brake_comparison():
    fig, axes = plt.subplots(5, 1, figsize=(12, 16), sharex=True)
    fig.suptitle("Brake Phase Detail — Zero-Cross Model Comparison\n(jerk = 0.5, 0.8, 1.0, 1.5, 2.0)",
                 fontsize=14, fontweight="bold")

    stats = []

    for jerk, color in zip(JERK_VALUES, COLORS):
        df = generate_csv(jerk)
        braking = df[df["ax_cmd_m_s2"] < -0.1]
        if len(braking) == 0:
            continue
        t_brake = braking["time_s"].iloc[0]
        idx = (df["time_s"] - t_brake).abs().idxmin()
        px_brake = df.loc[idx, "px_truth_m"]

        sub = df[df["time_s"] >= t_brake].copy()
        sub["time_rel"] = sub["time_s"] - t_brake
        sub["px_rel"] = sub["px_truth_m"] - px_brake

        axes[0].plot(sub["time_rel"], sub["px_rel"], color=color, label=f"j={jerk}", linewidth=1.5)
        axes[1].plot(sub["time_rel"], sub["vx_truth_m_s"], color=color, linewidth=1.5)
        axes[2].plot(sub["time_rel"], sub["theta_truth_rad"] * 180 / np.pi, color=color, linewidth=1.5)
        axes[3].plot(sub["time_rel"], sub["theta_dot_truth_rad_s"] * 180 / np.pi, color=color, linewidth=1.5)
        axes[4].plot(sub["time_rel"], sub["ax_applied_m_s2"], color=color, linewidth=1.5)

        stats.append({
            "jerk": jerk,
            "brake_dist": sub["px_rel"].iloc[-1],
            "max_theta": sub["theta_truth_rad"].abs().max() * 180 / np.pi,
            "final_vx": sub["vx_truth_m_s"].iloc[-1],
        })

    axes[0].set_ylabel("Brake Distance [m]")
    axes[0].set_title("Relative Horizontal Position")
    axes[0].legend(loc="upper left")
    axes[0].grid(True, alpha=0.3)

    axes[1].set_ylabel("Velocity [m/s]")
    axes[1].set_title("Horizontal Velocity")
    axes[1].axhline(0, color="black", linestyle="-", alpha=0.2)
    axes[1].grid(True, alpha=0.3)

    axes[2].set_ylabel("Angle [deg]")
    axes[2].set_title("Pendulum Pitch Angle")
    axes[2].grid(True, alpha=0.3)

    axes[3].set_ylabel("Rate [deg/s]")
    axes[3].set_title("Pendulum Pitch Rate")
    axes[3].axhline(0, color="black", linestyle="-", alpha=0.2)
    axes[3].grid(True, alpha=0.3)

    axes[4].set_ylabel("Acceleration [m/s²]")
    axes[4].set_title("Applied Acceleration")
    axes[4].axhline(0, color="black", linestyle="-", alpha=0.2)
    axes[4].grid(True, alpha=0.3)
    axes[4].set_xlabel("Time since brake start [s]")

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    out = "results/trajectory/zero_cross_comparison_brake.png"
    plt.savefig(out, dpi=150)
    print(f"Brake comparison saved to: {out}")
    plt.close()

    return stats


if __name__ == "__main__":
    stats_full = plot_full_comparison()
    stats_brake = plot_brake_comparison()

    print("\n" + "=" * 60)
    print("Zero-Cross Comparison Summary")
    print("=" * 60)
    for s in stats_brake:
        print(f"  jerk={s['jerk']}: brake_dist={s['brake_dist']:.2f}m, "
              f"max|θ|={s['max_theta']:.2f}°, final_vx={s['final_vx']:.2f}m/s")
    print("=" * 60)
