#!/usr/bin/env python3
"""绘制刹车段（brake phase）的详细对比图。

特点：
- 时间轴以刹车开始时刻为 0
- 位置轴以刹车开始位置为 0（显示相对位移 / 刹车距离）
"""

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def load_and_label(path, label):
    df = pd.read_csv(path)
    df["mode"] = label
    return df


def detect_brake_start(df, ax_threshold=-0.1):
    """从控制输入中检测刹车开始时刻。"""
    braking = df[df["ax_applied_m_s2"] < ax_threshold]
    if len(braking) == 0:
        return None, None
    t_brake = braking["time_s"].iloc[0]
    idx = (df["time_s"] - t_brake).abs().idxmin()
    px_brake = df.loc[idx, "px_truth_m"]
    return t_brake, px_brake


def plot_brake_phase(files, labels, output_path="brake_phase.png"):
    if len(files) != len(labels):
        print("Error: files and labels must have same length")
        sys.exit(1)

    data = [load_and_label(f, l) for f, l in zip(files, labels)]
    colors = {"Diagonal": "#D62728", "Coupled": "#1F77B4"}

    # 刹车开始时间固定为 t=40s（与参考指令一致）
    t_brake = 40.0
    idx = (data[0]["time_s"] - t_brake).abs().idxmin()
    px_brake = data[0].loc[idx, "px_truth_m"]

    # 筛选刹车段数据，并做归一化
    brake_data = []
    for df in data:
        sub = df[df["time_s"] >= t_brake].copy()
        sub["time_rel"] = sub["time_s"] - t_brake
        sub["px_rel"] = sub["px_truth_m"] - px_brake
        brake_data.append(sub)

    G = 9.81
    PEND_GAIN = 0.6
    rope_len = 10.0

    fig, axes = plt.subplots(5, 1, figsize=(12, 16), sharex=True)
    fig.suptitle("Brake Phase Comparison", fontsize=14, fontweight="bold")

    # 1. Position（相对位移 = 刹车距离）
    ax = axes[0]
    for df in brake_data:
        c = colors.get(df["mode"].iloc[0], "black")
        ax.plot(df["time_rel"], df["px_rel"], color=c, label=df["mode"].iloc[0], linewidth=1.5)
    ax.set_ylabel("Brake Distance [m]")
    ax.legend(loc="lower right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Relative Horizontal Position (start = 0)")

    # 2. Velocity: drone vx + payload abs velocity + v_ref
    ax = axes[1]
    # Plot v_ref once (same for both modes)
    ax.plot(brake_data[0]["time_rel"], brake_data[0]["v_ref_m_s"], color="gray", linestyle="-",
            linewidth=1.0, alpha=0.6, label="v_ref")
    for df in brake_data:
        c = colors.get(df["mode"].iloc[0], "black")
        mode_name = df["mode"].iloc[0]
        ax.plot(df["time_rel"], df["vx_truth_m_s"], color=c, label=f"{mode_name} drone vx", linewidth=1.5)
        v_payload = df["vx_truth_m_s"] + df["theta_dot_truth_rad_s"] * rope_len * np.cos(df["theta_truth_rad"])
        ax.plot(df["time_rel"], v_payload, color=c, linestyle="--", linewidth=1.2,
                label=f"{mode_name} payload vx")
    ax.axhline(0, color="black", linestyle="-", alpha=0.2)
    ax.set_ylabel("Velocity [m/s]")
    ax.set_ylim(-5, 20)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_title("Horizontal Velocity (Drone / Payload / Reference)")

    # 3. Pendulum angle
    ax = axes[2]
    for df in brake_data:
        c = colors.get(df["mode"].iloc[0], "black")
        ax.plot(df["time_rel"], df["theta_truth_rad"] * 180 / 3.14159,
                color=c, label=df["mode"].iloc[0], linewidth=1.5)
    ax.axhline(0, color="black", linestyle="-", alpha=0.2)
    ax.set_ylabel("Angle [deg]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Pendulum Pitch Angle")

    # 4. Angular velocity
    ax = axes[3]
    for df in brake_data:
        c = colors.get(df["mode"].iloc[0], "black")
        ax.plot(df["time_rel"], df["theta_dot_truth_rad_s"] * 180 / 3.14159,
                color=c, label=df["mode"].iloc[0], linewidth=1.5)
    ax.axhline(0, color="black", linestyle="-", alpha=0.2)
    ax.set_ylabel("Angular Rate [deg/s]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Pendulum Pitch Angular Rate")

    # 5. Acceleration breakdown: a_total + a1 + a2
    ax = axes[4]
    for df in brake_data:
        c = colors.get(df["mode"].iloc[0], "black")
        mode_name = df["mode"].iloc[0]
        a1 = df["ax_applied_m_s2"]
        a_total = df["ax_truth_m_s2"]
        a2 = a_total - a1
        ax.plot(df["time_rel"], a_total, color=c, linestyle="-", linewidth=1.5,
                label=f"{mode_name} a_total")
        ax.plot(df["time_rel"], a1, color=c, linestyle="--", linewidth=1.2,
                label=f"{mode_name} a1")
        ax.plot(df["time_rel"], a2, color=c, linestyle=":", linewidth=1.0,
                label=f"{mode_name} a2")
    ax.axhline(2, color="gray", linestyle="--", alpha=0.3)
    ax.axhline(-2, color="gray", linestyle="--", alpha=0.3)
    ax.axhline(0, color="black", linestyle="-", alpha=0.2)
    ax.set_ylabel("Acceleration [m/s²]")
    ax.set_xlabel("Time since brake start [s]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_title("Acceleration Breakdown (a_total = a1 + a2)")

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.savefig(output_path, dpi=150)
    print(f"Brake phase plot saved to: {output_path}")
    # plt.show()  # Uncomment for interactive display

    # 打印刹车段统计
    print("\n" + "=" * 60)
    print("Brake Phase Summary")
    print("=" * 60)
    for df in brake_data:
        mode = df["mode"].iloc[0]
        print(f"\n{mode}:")
        print(f"  Brake distance  : {df['px_rel'].iloc[-1]:.2f} m")
        print(f"  Max |theta|     : {df['theta_truth_rad'].abs().max() * 180 / 3.14159:.2f} deg")
        print(f"  Max |omega|     : {df['theta_dot_truth_rad_s'].abs().max() * 180 / 3.14159:.2f} deg/s")
        print(f"  Max |vx|        : {df['vx_truth_m_s'].abs().max():.2f} m/s")
        settled = df[df["vx_truth_m_s"].abs() < 0.1]
        print(f"  Settling time   : {settled['time_rel'].iloc[0]:.2f} s (|vx| < 0.1)" if len(settled) > 0 else "  Not settled")
    print("=" * 60)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 plot_brake_phase.py <diagonal.csv> <coupled.csv>")
        sys.exit(1)
    plot_brake_phase(
        [sys.argv[1], sys.argv[2]],
        ["Diagonal", "Coupled"]
    )
