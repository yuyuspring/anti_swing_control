#!/usr/bin/env python3
"""绘制 Jerk-Ramp 离线轨迹的刹车段详图。

时间轴以刹车开始时刻为 0，位置轴以刹车开始位置为 0。

用法:
    python3 plot_offline_brake_phase.py [csv_path]
"""

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def plot_brake_phase(csv_path: str, png_path: str = None):
    df = pd.read_csv(csv_path)

    if png_path is None:
        png_path = csv_path.replace(".csv", "_brake.png")

    # 检测刹车开始
    braking = df[df["ax_applied_m_s2"] < -0.1]
    if len(braking) == 0:
        print("No braking detected")
        return
    t_brake = braking["time_s"].iloc[0]
    idx = (df["time_s"] - t_brake).abs().idxmin()
    px_brake = df.loc[idx, "px_truth_m"]

    # 筛选刹车段并归一化
    sub = df[df["time_s"] >= t_brake].copy()
    sub["time_rel"] = sub["time_s"] - t_brake
    sub["px_rel"] = sub["px_truth_m"] - px_brake

    fig, axes = plt.subplots(5, 1, figsize=(12, 16), sharex=True)
    fig.suptitle("Jerk-Ramp Offline Trajectory — Brake Phase Detail",
                 fontsize=14, fontweight="bold")

    color = "C0"

    # 1. Position
    ax = axes[0]
    ax.plot(sub["time_rel"], sub["px_rel"], color=color, linewidth=1.5)
    ax.set_ylabel("Brake Distance [m]")
    ax.grid(True, alpha=0.3)
    ax.set_title("Relative Horizontal Position (start = 0)")

    # 2. Velocity
    ax = axes[1]
    ax.plot(sub["time_rel"], sub["vx_truth_m_s"], color=color, linewidth=1.5)
    ax.axhline(0, color="black", linestyle="-", alpha=0.2)
    ax.set_ylabel("Velocity [m/s]")
    ax.grid(True, alpha=0.3)
    ax.set_title("Horizontal Velocity")

    # 3. Payload Absolute Horizontal Velocity
    ax = axes[2]
    rope_len = 15.0
    v_abs = sub["vx_truth_m_s"] + sub["theta_dot_truth_rad_s"] * rope_len * np.cos(sub["theta_truth_rad"])
    ax.plot(sub["time_rel"], v_abs, color=color, linewidth=1.5)
    ax.axhline(0, color="black", linestyle="-", alpha=0.2)
    ax.set_ylabel("Abs Velocity [m/s]")
    ax.grid(True, alpha=0.3)
    ax.set_title("Payload Absolute Horizontal Velocity")

    # 4. Pendulum angle
    ax = axes[3]
    ax.plot(sub["time_rel"], sub["theta_truth_rad"] * 180 / np.pi,
            color=color, linewidth=1.5)
    ax.axhline(0, color="black", linestyle="-", alpha=0.2)
    ax.set_ylabel("Angle [deg]")
    ax.grid(True, alpha=0.3)
    ax.set_title("Pendulum Pitch Angle")

    # 5. Control input
    ax = axes[4]
    has_pitch = "pitch_applied_rad" in sub.columns and sub["pitch_applied_rad"].abs().max() > 1e-6
    if has_pitch:
        ax.plot(sub["time_rel"], np.degrees(sub["pitch_applied_rad"]), color=color,
                linewidth=1.5, label="Applied pitch")
        ax.plot(sub["time_rel"], np.degrees(sub["pitch_cmd_rad"]), color=color,
                linewidth=1.0, linestyle="--", alpha=0.6, label="Commanded pitch")
        ax.axhline(np.degrees(np.arctan(2/9.81)), color="gray", linestyle="--", alpha=0.3)
        ax.axhline(-np.degrees(np.arctan(2/9.81)), color="gray", linestyle="--", alpha=0.3)
        ax.set_ylabel("Pitch [deg]")
        ax.set_title("Drone Pitch Angle")
    else:
        ax.plot(sub["time_rel"], sub["ax_applied_m_s2"], color=color,
                linewidth=1.5, label="Applied ax")
        ax.plot(sub["time_rel"], sub["ax_cmd_m_s2"], color=color,
                linewidth=1.0, linestyle="--", alpha=0.6, label="Commanded ax")
        ax.axhline(2, color="gray", linestyle="--", alpha=0.3)
        ax.axhline(-2, color="gray", linestyle="--", alpha=0.3)
        ax.set_ylabel("Acceleration [m/s²]")
        ax.set_title("Control Input")
    ax.axhline(0, color="black", linestyle="-", alpha=0.2)
    ax.set_xlabel("Time since brake start [s]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.savefig(png_path, dpi=150)
    print(f"Brake phase plot saved to: {png_path}")

    # 统计
    print("\n" + "=" * 60)
    print("Brake Phase Summary")
    print("=" * 60)
    print(f"Brake start time    : {t_brake:.2f} s")
    print(f"Brake distance      : {sub['px_rel'].iloc[-1]:.2f} m")
    print(f"Max |theta|         : {sub['theta_truth_rad'].abs().max() * 180 / np.pi:.2f} deg")
    print(f"Max |theta_dot|     : {sub['theta_dot_truth_rad_s'].abs().max() * 180 / np.pi:.2f} deg/s")
    settled = sub[sub["vx_truth_m_s"].abs() < 0.1]
    if len(settled) > 0:
        print(f"Settling time       : {settled['time_rel'].iloc[0]:.2f} s (|vx| < 0.1)")
    else:
        print(f"Not settled — final vx: {sub['vx_truth_m_s'].iloc[-1]:.3f} m/s")
    print(f"Final theta         : {sub['theta_truth_rad'].iloc[-1] * 180 / np.pi:.2f} deg")
    print(f"Final omega         : {sub['theta_dot_truth_rad_s'].iloc[-1] * 180 / np.pi:.2f} deg/s")
    print("=" * 60)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        csv_path = "results/trajectory/offline_three_phase.csv"
    else:
        csv_path = sys.argv[1]
    plot_brake_phase(csv_path)
